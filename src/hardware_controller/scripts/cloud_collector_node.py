#!/usr/bin/env python3
"""
cloud_collector_node - składa odczyty dalmierza w chmurę punktów.

Każdy pomiar to punkt (range, 0, 0) w układzie laser_link. Cała rekonstrukcja
sprowadza się do przeniesienia go do układu nieruchomego przez TF - z dokładnym
znacznikiem czasu pomiaru, nie czasem odbioru wiadomości. Przy sweepie głowica
się rusza, więc pomyłka o jeden cykl to pomyłka o kilka stopni.

Subskrybuje:
  /laser/range        (sensor_msgs/Range)       - pomiar; NaN = brak wiarygodnego echa
  /laser/raw          (geometry_msgs/Vector3Stamped) - surowe ADC tego samego pomiaru
  /joint_states       (sensor_msgs/JointState)  - kąty osi zapisywane przy punkcie
  /sweep/collecting   (std_msgs/Bool)           - bramka: zbieramy tylko w bezruchu
  /sweep/state        (std_msgs/String)         - "done" wyzwala automatyczny zapis

Publikuje:
  /laser_cloud        (sensor_msgs/PointCloud2) - akumulowana chmura

Serwisy:
  ~/clear   (std_srvs/Empty) - czyści bufor
  ~/save    (std_srvs/Empty) - zapisuje CSV + PLY

Parametry:
  target_frame    (str, base_link) - układ, w którym budujemy chmurę
  require_gate    (bool, True)     - czy zbierać wyłącznie przy /sweep/collecting = true
  publish_rate    (float, 5.0)     - Hz publikacji chmury
  tf_retry_period (float, 0.05)    - jak często ponawiać nieudane lookupy [s]
  pending_max_age (float, 5.0)     - po tylu sekundach pomiar bez TF jest odrzucany [s]
  output_dir      (str, '')        - katalog zapisu; puste = $ROS_LOG_DIR albo ~/.ros/log
  file_prefix     (str, laser_sweep)
  autosave        (bool, True)     - zapis po otrzymaniu /sweep/state = "done"

W CSV obok metrów lądują dwa kanały diagnostyczne z /laser/raw. ICH NAZWY SĄ
PARAMETREM (raw_columns), bo znaczenie zależy od czujnika: Sharp dawał surowe
ADC i jego rozrzut, dalmierz JRT daje jakość sygnału i kod statusu. Wpisane na
sztywno "adc" nazywałoby te drugie fałszywie - a CSV jest tu materiałem do
analizy offline, więc mylna nazwa kolumny wraca po tygodniach jako zła
interpretacja. Kolumna "d"
jest skutkiem kalibracji wybranej w chwili skanowania - z samego ADC da się
odtworzyć chmurę dla dowolnej innej krzywej bez powtarzania skanu. Parowanie
idzie po znaczniku czasu, który broadcaster wpisuje identyczny w Range i w raw;
odczyt cache'a jest odłożony do _process_pending(), bo tam pomiar i tak czeka na
TF, więc surowa ramka na pewno zdążyła dotrzeć niezależnie od kolejności topików.

Obsługa opóźnionego TF jest tu konieczna, nie kosmetyczna: Range przychodzi ze
znacznikiem czasu pomiaru, a odpowiadający mu TF (z /joint_states przez
robot_state_publisher) potrafi dotrzeć kilkadziesiąt ms później. Lookup zrobiony
od razu poleciałby ExtrapolationException i pomiar by przepadł.
"""

import collections
import csv
import math
import os
import struct
import time as walltime
from datetime import datetime

import rclpy
import tf2_geometry_msgs  # noqa: F401 - rejestruje obsługę PointStamped w tf2
import tf2_ros
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, PointCloud2, PointField, Range
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Empty


class CloudCollectorNode(Node):
    def __init__(self):
        super().__init__("cloud_collector_node")

        # Nazwy kolumn dla dwóch kanałów z /laser/raw - patrz nagłówek.
        self.declare_parameter("raw_columns", "adc,adc_spread,sample_time")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("require_gate", True)
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("tf_retry_period", 0.05)
        self.declare_parameter("pending_max_age", 5.0)
        self.declare_parameter("output_dir", "")
        self.declare_parameter("file_prefix", "laser_sweep")
        self.declare_parameter("autosave", True)

        self._target_frame = self.get_parameter("target_frame").value
        self._require_gate = self.get_parameter("require_gate").value
        self._pending_max_age = self.get_parameter("pending_max_age").value
        self._file_prefix = self.get_parameter("file_prefix").value
        self._autosave = self.get_parameter("autosave").value

        raw_cols = [c.strip() for c in self.get_parameter("raw_columns").value.split(",")]
        if len(raw_cols) != 3 or not all(raw_cols):
            raise ValueError(
                f"raw_columns musi mieć postać '<nazwa>,<nazwa>,<nazwa>', jest {raw_cols!r}")
        self._raw_col0, self._raw_col1, self._raw_col2 = raw_cols

        # Moment otwarcia bramki. Pomiar, którego strzał ZACZĄŁ SIĘ przed tą
        # chwilą, powstawał przy jeszcze ruchomej głowicie - odrzucamy go, bo
        # trafiłby do chmury pod współrzędnymi pozycji docelowej i rozmazał ją
        # wzdłuż toru ruchu. Dalmierz JRT mierzy 0,1-4 s, więc taki strzał
        # spokojnie przeżywa dojazd i wraca już po zatrzymaniu.
        self._gate_open_at = 0.0
        self._n_stale = 0

        out = self.get_parameter("output_dir").value
        self._output_dir = out or os.environ.get(
            "ROS_LOG_DIR", os.path.expanduser("~/.ros/log"))

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(Range, "/laser/range", self._cb_range, 50)
        self.create_subscription(Vector3Stamped, "/laser/raw", self._cb_raw, 50)
        self.create_subscription(JointState, "/joint_states", self._cb_joints, 10)
        self.create_subscription(Bool, "/sweep/collecting", self._cb_gate, 10)
        # Musi pasować do QoS nadawcy (transient_local), inaczej zatrzaśnięte
        # "done" nie dotrze do kolektora, który wstał po sweepie.
        self.create_subscription(
            String, "/sweep/state", self._cb_state,
            QoSProfile(depth=1,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))

        self._pub = self.create_publisher(PointCloud2, "/laser_cloud", 10)
        self.create_service(Empty, "~/clear", self._srv_clear)
        self.create_service(Empty, "~/save", self._srv_save)

        self.create_timer(1.0 / self.get_parameter("publish_rate").value, self._publish_cloud)
        self.create_timer(self.get_parameter("tf_retry_period").value, self._process_pending)

        self._points: list[tuple[float, float, float, float]] = []
        self._meta: list[dict] = []
        self._pending: collections.deque = collections.deque()

        # Surowe ADC pod kluczem stempla w nanosekundach. Ograniczone rozmiarem,
        # bo pomiar bez TF potrafi czekać sekundy, a przy 18,6 Hz 4096 wpisów to
        # ponad 3 minuty historii - z zapasem ponad pending_max_age.
        self._raw: collections.OrderedDict = collections.OrderedDict()
        self._raw_max = 4096

        self._joints: dict[str, float] = {}
        self._collecting = not self._require_gate
        self._saved_for_current_run = False
        self._save_timer = None

        # Liczniki: bez nich "mało punktów" jest nierozstrzygalne - nie wiadomo,
        # czy czujnik nie widział celu, czy bramka była zamknięta, czy padł TF.
        self._n_seen = 0
        self._n_gated = 0
        self._n_invalid = 0
        self._n_no_tf = 0
        self._n_no_raw = 0

        self.get_logger().info(
            f"CloudCollectorNode gotowy. Układ docelowy: {self._target_frame}, "
            f"bramka /sweep/collecting: {'wymagana' if self._require_gate else 'ignorowana'}, "
            f"zapis do: {self._output_dir}")

    # ------------------------------------------------------------------ wejścia

    def _cb_joints(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joints[name] = pos

    def _cb_raw(self, msg: Vector3Stamped):
        key = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        # Przy powtórzonej ramce (patrz komentarz w laser_broadcaster.cpp) klucz
        # jest ten sam, a wartości identyczne - nadpisanie niczego nie psuje.
        self._raw[key] = (msg.vector.x, msg.vector.y, msg.vector.z)
        while len(self._raw) > self._raw_max:
            self._raw.popitem(last=False)

    def _cb_gate(self, msg: Bool):
        if msg.data and not self._collecting:
            self._gate_open_at = self.get_clock().now().nanoseconds * 1e-9
        self._collecting = msg.data

    def _cb_state(self, msg: String):
        if msg.data == "running":
            self._saved_for_current_run = False
        elif msg.data in ("done", "aborted") and self._autosave and not self._saved_for_current_run:
            self._saved_for_current_run = True
            # Pomiary z ostatnich punktów mogą jeszcze czekać na TF; damy im
            # dokończyć, zanim zapiszemy plik.
            self._save_timer = self.create_timer(1.0, self._deferred_save)

    def _deferred_save(self):
        # Timer jednorazowy - anulujemy przed zapisem, żeby wyjątek w save()
        # nie zostawił za sobą timera zapisującego w kółko.
        if self._save_timer is not None:
            self._save_timer.cancel()
            self._save_timer = None
        self._process_pending()
        self.save()

    def _cb_range(self, msg: Range):
        self._n_seen += 1

        if self._require_gate and not self._collecting:
            self._n_gated += 1
            return

        d = msg.range
        # NaN to świadomy sygnał wtyczki: odczyt poza wiarygodnym zakresem
        # (w szczególności strefa zawinięcia krzywej Sharpa poniżej 1 m).
        if not math.isfinite(d) or d < msg.min_range or d > msg.max_range:
            self._n_invalid += 1
            return

        pt = PointStamped()
        pt.header.frame_id = msg.header.frame_id
        pt.header.stamp = msg.header.stamp
        pt.point.x = float(d)
        pt.point.y = 0.0
        pt.point.z = 0.0

        self._pending.append({
            "pt": pt,
            "d": float(d),
            "stamp_key": msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
            "gate_open": self._gate_open_at,
            "stamp_sec": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "received": walltime.monotonic(),
            "joints": dict(self._joints),
        })

    # --------------------------------------------------------- kolejka TF

    def _process_pending(self):
        now = walltime.monotonic()
        still = collections.deque()

        while self._pending:
            entry = self._pending.popleft()
            age = now - entry["received"]

            if age > self._pending_max_age:
                self._n_no_tf += 1
                self.get_logger().warn(
                    f"Odrzucono pomiar bez TF (wiek {age:.2f} s) dla t={entry['stamp_sec']:.4f}",
                    throttle_duration_sec=5.0)
                continue

            pt = entry["pt"]
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._target_frame, pt.header.frame_id, pt.header.stamp,
                    timeout=Duration(seconds=0))
                out = self._tf_buffer.transform(
                    pt, self._target_frame, timeout=Duration(seconds=0))
            except tf2_ros.ExtrapolationException:
                # TF dla tej chwili jeszcze nie dotarł - spróbujemy za moment.
                still.append(entry)
                continue
            except Exception as e:
                self._n_no_tf += 1
                self.get_logger().warn(
                    f"TF {pt.header.frame_id} -> {self._target_frame} nieudany: {e}",
                    throttle_duration_sec=5.0)
                continue

            tr = tf.transform.translation
            ro = tf.transform.rotation
            x, y, z = out.point.x, out.point.y, out.point.z

            raw0, raw1, raw2 = self._raw.pop(entry["stamp_key"], (None, None, None))

            # raw2 = moment rozpoczęcia strzału (patrz komentarz przy _gate_open_at).
            if raw2 is not None and entry["gate_open"] > 0.0 and raw2 <= entry["gate_open"]:
                self._n_stale += 1
                self.get_logger().warn(
                    f"Odrzucono pomiar rozpoczęty {entry['gate_open'] - raw2:.2f} s PRZED "
                    f"otwarciem bramki - powstawał przy ruchomej głowicy "
                    f"({self._n_stale} takich)",
                    throttle_duration_sec=10.0)
                continue

            if raw0 is None:
                self._n_no_raw += 1
                self.get_logger().warn(
                    f"Brak danych surowych dla t={entry['stamp_sec']:.4f} - "
                    f"kolumny {self._raw_col0}/{self._raw_col1} zostaną puste",
                    throttle_duration_sec=5.0)

            self._points.append((x, y, z, entry["d"]))
            self._meta.append({
                "x": x, "y": y, "z": z,
                "d": entry["d"],
                self._raw_col0: raw0,
                self._raw_col1: raw1,
                self._raw_col2: raw2,
                "stamp": entry["stamp_sec"],
                "tf_stamp": tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9,
                "laser_tx": tr.x, "laser_ty": tr.y, "laser_tz": tr.z,
                "laser_qx": ro.x, "laser_qy": ro.y, "laser_qz": ro.z, "laser_qw": ro.w,
                **{f"j_{k}": v for k, v in entry["joints"].items()},
            })

        self._pending = still

    # ------------------------------------------------------------------ zapis

    def _srv_clear(self, _req, resp):
        n = len(self._points)
        self._points.clear()
        self._meta.clear()
        self._pending.clear()
        self._raw.clear()
        self._n_seen = self._n_gated = self._n_invalid = self._n_no_tf = 0
        self._n_no_raw = 0
        self.get_logger().info(f"Bufor wyczyszczony ({n} punktów).")
        return resp

    def _srv_save(self, _req, resp):
        self.save()
        return resp

    def save(self):
        if not self._meta:
            self.get_logger().warn(
                f"Bufor pusty - nie ma czego zapisać. "
                f"Odebrano {self._n_seen} pomiarów: {self._n_gated} poza bramką, "
                f"{self._n_invalid} poza zakresem, {self._n_no_tf} bez TF.")
            return

        os.makedirs(self._output_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.join(self._output_dir, f"{self._file_prefix}_{ts}")

        # Kolumny biorą się z pierwszego wpisu, ale nazwy jointów mogą się
        # pojawić dopiero w kolejnych - stąd suma po wszystkich wpisach.
        fieldnames = []
        for m in self._meta:
            for k in m:
                if k not in fieldnames:
                    fieldnames.append(k)

        with open(f"{base}.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            w.writerows(self._meta)

        # PLY dla CloudCompare - reszta analizy w projekcie i tak jedzie na CSV.
        with open(f"{base}.ply", "w") as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write(f"element vertex {len(self._points)}\n")
            f.write("property float x\nproperty float y\nproperty float z\n")
            f.write("end_header\n")
            for x, y, z, _d in self._points:
                f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

        self.get_logger().info(
            f"Zapisano {len(self._meta)} punktów: {base}.csv oraz {base}.ply")
        self.get_logger().info(
            f"Bilans: {self._n_seen} odebranych, {self._n_gated} poza bramką, "
            f"{self._n_invalid} poza zakresem, {self._n_no_tf} bez TF, "
            f"{self._n_no_raw} bez surowego ADC.")

    # -------------------------------------------------------------- publikacja

    def _publish_cloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._target_frame

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(self._points)
        msg.is_bigendian = False
        msg.is_dense = True

        if not self._points:
            msg.fields = []
            msg.point_step = 0
            msg.row_step = 0
            msg.data = b""
            self._pub.publish(msg)
            return

        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width

        data = bytearray()
        for x, y, z, d in self._points:
            data += struct.pack("ffff", x, y, z, d)
        msg.data = bytes(data)

        self._pub.publish(msg)


def main():
    rclpy.init()
    node = CloudCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
