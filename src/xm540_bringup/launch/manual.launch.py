import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("xm540_bringup")

    urdf_path = os.path.join(pkg_share, "urdf", "xm540_manipulator.urdf")
    with open(urdf_path, "r") as f:
        robot_description = f.read()

    rviz_config = os.path.join(pkg_share, "rviz", "xm540.rviz")

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="xm540_bringup",
            executable="dynamixel_node",
            name="dynamixel_node",
            output="screen",
        ),
        Node(
            package="xm540_bringup",
            executable="manual_node",
            name="manual_node",
            output="screen",
            parameters=[{
                "tolerance_deg": 0.5,
                "timeout_s":     5.0,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
