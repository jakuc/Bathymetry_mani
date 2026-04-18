"""
Launch ROS2 nodes dla symulacji Isaac Sim.

Uruchom oddzielnie (w osobnym terminalu) sam Isaac Sim:
    OMNI_KIT_ALLOW_ROOT=1 python3 src/xm540_bringup/isaac/isaac_sim.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("xm540_bringup")
    urdf_path = os.path.join(pkg_share, "urdf", "xm540_manipulator_isaac.urdf")
    rviz_config = os.path.join(pkg_share, "rviz", "xm540.rviz")

    waypoints_csv       = os.path.join(pkg_share, "waypoints.csv")
    sweep_waypoints_csv = os.path.join(pkg_share, "sweep_waypoints.csv")
    mission_config      = os.path.join(pkg_share, "config", "mission.yaml")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    verbose = LaunchConfiguration("verbose", default="0")

    return LaunchDescription([
        DeclareLaunchArgument("verbose", default_value="0",
                              description="1 = pełne logi (INFO), 0 = tylko WARN+"),

        # Robot State Publisher – publikuje TF z /joint_states
        # Uwaga: world→base_link rozgłasza isaac_sim.py na podstawie rzeczywistej pozycji w Isaac Sim
        # /joint_states pochodzi z sim_driver_node który przepisuje /sim/encoders z fizyki Isaac Sim
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        # Symulator serwomechanizmu – serwisy sterowania + /joint_states
        Node(
            package="xm540_bringup",
            executable="sim_driver_node",
            name="sim_driver_node",
            output="screen",
        ),

        # Kolektor chmury punktów
        Node(
            package="xm540_bringup",
            executable="scan_collector_node",
            name="scan_collector_node",
            output="screen",
            parameters=[{
                "publish_rate": 2.0,
                "source_frame": "sonar_link",
                "max_points":   0,
                "debug_tf":     True,
            }],
        ),

        # Supervisor misji batymetrycznej
        Node(
            package="xm540_bringup",
            executable="mission_supervisor_node",
            name="mission_supervisor_node",
            output="screen",
            parameters=[
                mission_config,
                {
                    "waypoints_file":       waypoints_csv,
                    "sweep_waypoints_file": sweep_waypoints_csv,
                },
            ],
        ),

        # Wizualizacja waypointów (MarkerArray → RViz)
        Node(
            package="xm540_bringup",
            executable="waypoint_viz_node",
            name="waypoint_viz_node",
            output="screen",
            parameters=[{"waypoints_file": waypoints_csv}],
        ),

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            ros_arguments=["--log-level",
                           PythonExpression(["'rviz2:=INFO' if '", verbose,
                                             "' == '1' else 'rviz2:=WARN'"])],
        ),
    ])
