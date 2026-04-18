"""
Manual node: serwis ROS2 do ustawiania pozycji serwa.

Serwisy:
  /set_orientation  (xm540_interfaces/srv/SetOrientation)
    request:  joint_y_deg  – zadana pozycja osi Y [°], zakres [-90, 90]
              joint_z_deg  – ignorowane (brak drugiego serwa w HW)
    response: success, message

Subskrybuje:
  /joint_states  (sensor_msgs/JointState) – pozycja biezaca

Publikuje:
  /servo/goal_position  (std_msgs/Float64)  – zadana pozycja [°]

Parametry ROS2:
  tolerance_deg  (float, 0.5)  – prog uznania pozycji za osiagnieta
  timeout_s      (float, 5.0)  – maks. czas oczekiwania na pozycje
"""

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from xm540_interfaces.srv import SetOrientation

RANGE_MIN = -90.0
RANGE_MAX =  90.0


class ManualNode(Node):
    def __init__(self):
        super().__init__("manual_node")

        self.declare_parameter("tolerance_deg", 0.5)
        self.declare_parameter("timeout_s",     5.0)

        self.tol_rad  = math.radians(self.get_parameter("tolerance_deg").value)
        self.timeout  = self.get_parameter("timeout_s").value

        self.current_pos = None

        cb_group = ReentrantCallbackGroup()

        self.pub = self.create_publisher(Float64, "/servo/goal_position", 10)
        self.create_subscription(JointState, "/joint_states", self._cb_js, 10,
                                 callback_group=cb_group)
        self.create_service(SetOrientation, "/set_orientation", self._cb_set_orientation,
                            callback_group=cb_group)

        self.get_logger().info(
            "ManualNode gotowy. Wywolaj: "
            "ros2 service call /set_orientation xm540_interfaces/srv/SetOrientation "
            '"{joint_z_deg: 0.0, joint_y_deg: 45.0}"'
        )

    def _cb_js(self, msg: JointState):
        if msg.position:
            self.current_pos = msg.position[0]

    def _cb_set_orientation(self, request, response):
        deg = request.joint_y_deg

        if not (RANGE_MIN <= deg <= RANGE_MAX):
            response.success = False
            response.message = (
                f"Pozycja {deg:.1f}° poza zakresem [{RANGE_MIN}, {RANGE_MAX}]°."
            )
            return response

        goal_msg = Float64()
        goal_msg.data = deg
        self.pub.publish(goal_msg)

        target_rad = math.radians(deg)
        self.get_logger().info(f"Jade do {deg:.1f}°...")

        deadline = self.get_clock().now().nanoseconds + int(self.timeout * 1e9)
        while rclpy.ok():
            if (self.current_pos is not None and
                    abs(self.current_pos - target_rad) <= self.tol_rad):
                response.success = True
                response.message = f"Osiagnieto {deg:.1f}°."
                self.get_logger().info(response.message)
                return response

            if self.get_clock().now().nanoseconds > deadline:
                response.success = False
                response.message = (
                    f"Timeout: nie osiagnieto {deg:.1f}° w {self.timeout}s."
                )
                self.get_logger().warn(response.message)
                return response

            threading.Event().wait(0.02)

        response.success = False
        response.message = "Node zakonczyl dzialanie."
        return response


def main():
    rclpy.init()
    node = ManualNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
