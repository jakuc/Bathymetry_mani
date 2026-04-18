"""
scan_collector_node – zbiera punkty z sonara i buduje chmurę punktów w układzie world.

Subskrybuje:
  /sonar        (sensor_msgs/LaserScan)   – odczyt sonara (jeden promień)

Publikuje:
  /scan_cloud   (sensor_msgs/PointCloud2) – akumulowana chmura punktów (frame: world)

Serwisy:
  ~/clear       (std_srvs/Empty)          – czyści bufor punktów
  ~/save_csv    (std_srvs/Empty)          – zapisuje bufor do /tmp/scan_<timestamp>.csv

Parametry:
  publish_rate   (float, 10.0) – Hz publikacji chmury
  source_frame   (str,   '')   – override frame_id sonara; pusty = użyj z msg.header
  max_points     (int,   0)    – maks. punktów w buforze, 0 = bez limitu (ring buffer)
  debug_tf       (bool,  False) – loguje szczegóły każdej transformacji (INFO level)
  tf_timeout     (float, 2.0)  – maks. czas oczekiwania na TF dla danego pomiaru [s]
  pending_max_age (float, 5.0) – po tym czasie pomiar bez TF jest odrzucany [s]

Obsługa opóźnionego TF:
  Pomiary trafiają do kolejki pending. Co 50 ms próbujemy dla każdego pomiaru
  wykonać lookup_transform po dokładnym timestamp pomiaru. Gdy TF dotrze,
  punkt jest transformowany i dodawany do chmury. Pomiary starsze niż
  pending_max_age są odrzucane.
"""

import csv
import collections
import math
import os
import struct
import threading
import time as walltime
from datetime import datetime

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState, LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Empty
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 – rejestruje obsługę PointStamped w tf2
from rclpy.time import Time


class ScanCollectorNode(Node):
    def __init__(self):
        super().__init__("scan_collector_node")

        self.declare_parameter("publish_rate",   10.0)
        self.declare_parameter("source_frame",   "")
        self.declare_parameter("max_points",     0)
        self.declare_parameter("debug_tf",       False)
        self.declare_parameter("tf_timeout",     2.0)
        self.declare_parameter("pending_max_age", 5.0)

        self._source_frame    = self.get_parameter("source_frame").value
        self._max_points      = self.get_parameter("max_points").value
        rate                  = self.get_parameter("publish_rate").value
        self._debug_tf        = self.get_parameter("debug_tf").value
        self._tf_timeout      = self.get_parameter("tf_timeout").value
        self._pending_max_age = self.get_parameter("pending_max_age").value

        self._tf_buffer   = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(LaserScan,   "/sim/sonar",    self._cb_sonar,  10)
        self.create_subscription(JointState,  "/joint_states", self._cb_joints, 10)

        self._pub = self.create_publisher(PointCloud2, "/scan_cloud", 10)
        self.create_service(Empty, "~/clear",    self._srv_clear)
        self.create_service(Empty, "~/save_csv", self._srv_save_csv)
        self.create_timer(0.05, self._process_pending)   # co 50 ms sim-czasu próbuj TF

        # Publikacja chmury na wall-clock — RViz działa w czasie rzeczywistym
        threading.Thread(
            target=self._publish_cloud_loop,
            args=(1.0 / rate,),
            daemon=True,
        ).start()

        # Bufor punktów do PointCloud2
        self._points: list[tuple[float, float, float, float]] = []

        # Bufor metadanych – każdy wpis to słownik z pełnym stanem w chwili pomiaru
        self._meta: list[dict] = []

        # Kolejka pomiarów czekających na TF
        # każdy wpis: dict z kluczami: pt_in, d, src_frame, scan_stamp_sec,
        #             received_wall, joints_snapshot, js_stamp
        self._pending: collections.deque = collections.deque()

        # Ostatni odczyt joint states
        self._latest_joints: dict[str, float] = {}
        self._latest_js_stamp: float = 0.0

        self.get_logger().info(
            f"ScanCollectorNode gotowy. "
            f"Publish rate: {rate} Hz, max_points: {self._max_points or '∞'}, "
            f"source_frame override: '{self._source_frame or 'z msg.header'}', "
            f"debug_tf: {self._debug_tf}, "
            f"tf_timeout: {self._tf_timeout}s, pending_max_age: {self._pending_max_age}s"
        )

    # --------------------------------------------------------------- joints

    def _cb_joints(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self._latest_joints[name] = pos
        self._latest_js_stamp = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )

    # ---------------------------------------------------------------- sonar

    def _cb_sonar(self, msg: LaserScan) -> None:
        if not msg.ranges:
            return

        d = msg.ranges[0]
        if not math.isfinite(d) or d <= msg.range_min or d >= msg.range_max:
            return

        src_frame = self._source_frame or msg.header.frame_id

        pt_in = PointStamped()
        pt_in.header.frame_id = src_frame
        pt_in.header.stamp    = msg.header.stamp   # dokładny timestamp pomiaru
        pt_in.point.x = d
        pt_in.point.y = 0.0
        pt_in.point.z = 0.0

        scan_stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self._pending.append({
            "pt_in":           pt_in,
            "d":               d,
            "src_frame":       src_frame,
            "scan_stamp_sec":  scan_stamp_sec,
            "received_wall":   walltime.monotonic(),
            "joints_snapshot": dict(self._latest_joints),
            "js_stamp":        self._latest_js_stamp,
        })

    # --------------------------------------------------------- process pending

    def _process_pending(self) -> None:
        """Próbuje przetworzyć pomiary z kolejki gdy ich TF jest już dostępny."""
        now_wall = walltime.monotonic()
        still_pending: collections.deque = collections.deque()

        while self._pending:
            entry = self._pending.popleft()
            age = now_wall - entry["received_wall"]

            if age > self._pending_max_age:
                self.get_logger().warn(
                    f"Odrzucono pomiar (wiek {age:.2f}s > {self._pending_max_age}s): "
                    f"brak TF dla t={entry['scan_stamp_sec']:.4f}",
                    throttle_duration_sec=2.0,
                )
                continue

            pt_in     = entry["pt_in"]
            src_frame = entry["src_frame"]

            try:
                tf_stamped = self._tf_buffer.lookup_transform(
                    "world", src_frame,
                    pt_in.header.stamp,          # dokładny timestamp pomiaru
                    timeout=Duration(seconds=0), # nieblokujący – tylko to co jest
                )
                pt_out = self._tf_buffer.transform(
                    pt_in, "world", timeout=Duration(seconds=0)
                )
            except tf2_ros.ExtrapolationException:
                # TF jeszcze nie dotarł – zostaw w kolejce
                still_pending.append(entry)
                continue
            except Exception as e:
                self.get_logger().warn(
                    f"TF lookup nieudany ({src_frame} → world): {e}",
                    throttle_duration_sec=2.0,
                )
                continue

            tr = tf_stamped.transform.translation
            ro = tf_stamped.transform.rotation
            tf_stamp_sec = (
                tf_stamped.header.stamp.sec + tf_stamped.header.stamp.nanosec * 1e-9
            )
            now_sec = self.get_clock().now().nanoseconds * 1e-9

            wx, wy, wz = pt_out.point.x, pt_out.point.y, pt_out.point.z
            d = entry["d"]
            scan_stamp_sec = entry["scan_stamp_sec"]

            self._points.append((wx, wy, wz, d))
            self._meta.append({
                "scan_stamp":   scan_stamp_sec,
                "callback_now": now_sec,
                "tf_stamp":     tf_stamp_sec,
                "dt_scan_tf":   scan_stamp_sec - tf_stamp_sec,
                "dt_scan_now":  now_sec - scan_stamp_sec,
                "pending_age":  now_wall - entry["received_wall"],
                "d":            d,
                "sonar_tx":     tr.x,
                "sonar_ty":     tr.y,
                "sonar_tz":     tr.z,
                "sonar_qx":     ro.x,
                "sonar_qy":     ro.y,
                "sonar_qz":     ro.z,
                "sonar_qw":     ro.w,
                "wx":           wx,
                "wy":           wy,
                "wz":           wz,
                **{f"j_{k}": v for k, v in entry["joints_snapshot"].items()},
                "js_stamp":     entry["js_stamp"],
            })

            if self._debug_tf:
                self.get_logger().debug(
                    f"scan_t={scan_stamp_sec:.4f} tf_t={tf_stamp_sec:.4f} "
                    f"dt={scan_stamp_sec - tf_stamp_sec:+.4f}s "
                    f"age={now_wall - entry['received_wall']:.3f}s | "
                    f"sonar_origin=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) | "
                    f"d={d:.4f} → world=({wx:.4f},{wy:.4f},{wz:.4f})"
                )

            if self._max_points > 0 and len(self._points) > self._max_points:
                self._points = self._points[-self._max_points:]
                self._meta   = self._meta[-self._max_points:]

        self._pending = still_pending

    # ---------------------------------------------------------------- clear

    def _srv_clear(self, _request, response):
        self._srv_save_csv(_request, response)
        n = len(self._points)
        p = len(self._pending)
        self._points.clear()
        self._meta.clear()
        self._pending.clear()
        self.get_logger().info(f"Bufor wyczyszczony ({n} punktów, {p} pending usunięto).")
        return response

    # -------------------------------------------------------------- save csv

    def _srv_save_csv(self, _request, response):
        if not self._meta:
            self.get_logger().warn("Bufor pusty – brak danych do zapisu.")
            return response

        ts      = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = os.environ.get("ROS_LOG_DIR", os.path.expanduser("~/.ros/log"))
        path    = os.path.join(log_dir, f"scan_{ts}.csv")

        fieldnames = list(self._meta[0].keys())
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._meta)

        self.get_logger().info(
            f"Zapisano {len(self._meta)} punktów do {path}"
        )
        return response

    # ---------------------------------------------------------- publish cloud

    def _publish_cloud_loop(self, interval: float) -> None:
        while True:
            walltime.sleep(interval)
            self._publish_cloud()

    def _publish_cloud(self) -> None:
        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = "world"

        if not self._points:
            msg = PointCloud2()
            msg.header     = header
            msg.height     = 1
            msg.width      = 0
            msg.fields     = []
            msg.is_dense   = True
            msg.point_step = 0
            msg.row_step   = 0
            msg.data       = b""
            self._pub.publish(msg)
            return

        fields = [
            PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        data = bytearray()
        for x, y, z, intensity in self._points:
            data += struct.pack("ffff", x, y, z, intensity)

        msg = PointCloud2()
        msg.header       = header
        msg.height       = 1
        msg.width        = len(self._points)
        msg.fields       = fields
        msg.is_bigendian = False
        msg.point_step   = 16
        msg.row_step     = msg.point_step * msg.width
        msg.data         = bytes(data)
        msg.is_dense     = True

        self._pub.publish(msg)
        self.get_logger().debug(f"Opublikowano {len(self._points)} punktów.")


def main():
    rclpy.init()
    node = ScanCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
