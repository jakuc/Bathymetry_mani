"""
waypoint_viz_node – Wizualizacja waypointów z pliku CSV w RViz.

Wczytuje waypoints.csv i publikuje MarkerArray na /waypoints/markers
(transient_local QoS – RViz odbiera nawet po późniejszym podłączeniu).

Parametry:
  waypoints_file  (str)   – ścieżka do waypoints.csv
  marker_scale    (float) – rozmiar kulki [m] (domyślnie 1.0)
  marker_r        (float) – kolor R (domyślnie 1.0 – czerwony)
  marker_g        (float) – kolor G (domyślnie 0.5)
  marker_b        (float) – kolor B (domyślnie 0.0)
  marker_a        (float) – przezroczystość (domyślnie 0.8)
"""

import csv
import os
import pathlib

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

_DEFAULT_CSV = os.path.join(
    get_package_share_directory("xm540_bringup"), "waypoints.csv"
)


class WaypointVizNode(Node):
    def __init__(self):
        super().__init__("waypoint_viz_node")

        self.declare_parameter("waypoints_file", _DEFAULT_CSV)
        self.declare_parameter("marker_scale",   1.0)
        self.declare_parameter("marker_r",       1.0)
        self.declare_parameter("marker_g",       0.5)
        self.declare_parameter("marker_b",       0.0)
        self.declare_parameter("marker_a",       0.8)

        csv_path = self.get_parameter("waypoints_file").value
        scale    = self.get_parameter("marker_scale").value
        r = self.get_parameter("marker_r").value
        g = self.get_parameter("marker_g").value
        b = self.get_parameter("marker_b").value
        a = self.get_parameter("marker_a").value

        self._pub = self.create_publisher(MarkerArray, "/waypoints/markers", _LATCHED_QOS)

        waypoints = self._load_csv(csv_path)
        if waypoints:
            self._msg = self._build_marker_array(waypoints, scale, r, g, b, a)
            self._pub.publish(self._msg)
            self.get_logger().info(
                f"Opublikowano {len(waypoints)} waypointów z {csv_path}"
            )
        else:
            self._msg = None
            self.get_logger().warn(
                f"Brak waypointów – sprawdź ścieżkę: {csv_path}"
            )

    def _load_csv(self, path: str) -> list[tuple[float, float, float]]:
        if not os.path.isfile(path):
            self.get_logger().error(f"Plik nie istnieje: {path}")
            return []

        waypoints = []
        try:
            with open(path, newline="") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    waypoints.append((
                        float(row["world_x"]),
                        float(row["world_y"]),
                        float(row["world_z"]),
                    ))
        except Exception as e:
            self.get_logger().error(f"Błąd wczytywania CSV: {e}")

        return waypoints

    def _build_marker_array(self, waypoints, scale, r, g, b, a) -> MarkerArray:
        """Jeden marker SPHERE_LIST = wydajniejsze niż N osobnych markerów."""
        msg = MarkerArray()

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp    = self.get_clock().now().to_msg()
        marker.ns              = "waypoints"
        marker.id              = 0
        marker.type            = Marker.SPHERE_LIST
        marker.action          = Marker.ADD
        marker.scale.x         = scale
        marker.scale.y         = scale
        marker.scale.z         = scale
        marker.color.r         = float(r)
        marker.color.g         = float(g)
        marker.color.b         = float(b)
        marker.color.a         = float(a)
        marker.pose.orientation.w = 1.0

        from geometry_msgs.msg import Point
        for wx, wy, wz in waypoints:
            p = Point()
            p.x = wx
            p.y = wy
            p.z = wz
            marker.points.append(p)

        msg.markers.append(marker)
        return msg


def main():
    rclpy.init()
    node = WaypointVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
