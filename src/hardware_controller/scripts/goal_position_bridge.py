#!/usr/bin/env python3
"""
goal_position_bridge – most między starym interfejsem xm540_bringup a ros2_control.

manual_node.py i sweep_node.py (paczka xm540_bringup, celowo nietknięta) publikują
zadaną pozycję na /servo/goal_position (std_msgs/Float64, stopnie) — dokładnie tak
jak wcześniej robił dynamixel_node.py. Ten most zamienia to na komendę dla
forward_position_controller (std_msgs/Float64MultiArray, radiany), żeby stare węzły
mogły sterować prawdziwym serwem przez controller_manager bez żadnej zmiany w swoim kodzie.

Subskrybuje:
  /servo/goal_position  (std_msgs/Float64)        – zadana pozycja [°]

Publikuje:
  /forward_position_controller/commands  (std_msgs/Float64MultiArray)  – zadana pozycja [rad]
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


class GoalPositionBridge(Node):
    def __init__(self):
        super().__init__("goal_position_bridge")

        self.pub = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10)
        self.create_subscription(
            Float64, "/servo/goal_position", self._cb_goal, 10)

        self.get_logger().info("GoalPositionBridge gotowy.")

    def _cb_goal(self, msg: Float64):
        out = Float64MultiArray()
        out.data = [math.radians(msg.data)]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = GoalPositionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
