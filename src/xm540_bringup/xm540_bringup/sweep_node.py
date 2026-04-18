"""
Sweep node: logika z example2.py jako node ROS2.

Publikuje:
  /servo/goal_position  (std_msgs/Float64)  – zadana pozycja [°]

Subskrybuje:
  /joint_states         (sensor_msgs/JointState) – pozycja bieżąca

Parametry ROS2 (można nadpisać z launch file):
  range_deg      (int,   90)    – amplituda ruchu [°]
  step_deg       (float, 1.0)   – krok obrotu [°], min 0.1
  rate_hz        (int,   20)    – skoków na sekundę
  tolerance_deg  (float, 0.5)   – próg uznania pozycji za osiągniętą
"""

import time
import math
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


class SweepNode(Node):
    def __init__(self):
        super().__init__("sweep_node")

        self.declare_parameter("range_deg",     90)
        self.declare_parameter("step_deg",      1.0)
        self.declare_parameter("rate_hz",       20)
        self.declare_parameter("tolerance_deg", 0.5)

        range_deg     = self.get_parameter("range_deg").value
        step_deg      = max(0.1, self.get_parameter("step_deg").value)
        rate_hz       = self.get_parameter("rate_hz").value
        tolerance_deg = self.get_parameter("tolerance_deg").value

        self.period      = 1.0 / rate_hz
        self.tol_rad     = math.radians(tolerance_deg)
        self.current_pos = None

        self.pub = self.create_publisher(Float64, "/servo/goal_position", 10)
        self.create_subscription(JointState, "/joint_states", self._cb_js, 10)

        n = int(round(range_deg / step_deg))
        self.angles = [i * step_deg for i in range(-n, n + 1)]

        self.get_logger().info(
            f"SweepNode: zakres ±{range_deg}°, krok {step_deg}°, "
            f"{rate_hz} Hz, tolerance {tolerance_deg}°"
        )

        # sweep w osobnym wątku – nie blokuje executora ROS2
        threading.Thread(target=self._sweep_thread, daemon=True).start()

    def _cb_js(self, msg: JointState):
        if msg.position:
            self.current_pos = msg.position[0]

    def _sweep_thread(self):
        time.sleep(1.0)   # poczekaj aż driver będzie gotowy
        self.get_logger().info("Sweep start.")
        goal_msg = Float64()

        for deg in self.angles:
            target_rad = math.radians(float(deg))
            goal_msg.data = float(deg)
            self.pub.publish(goal_msg)

            deadline = time.monotonic() + self.period

            while time.monotonic() < deadline:
                if (self.current_pos is not None and
                        abs(self.current_pos - target_rad) <= self.tol_rad):
                    break
                time.sleep(0.002)

            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)

        self.get_logger().info("Sweep zakończony.")


def main():
    rclpy.init()
    node = SweepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
