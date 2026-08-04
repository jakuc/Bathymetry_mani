"""
real_hardware.launch.py – manipulator + echosonda na realnym sprzęcie przez ros2_control.

Nie modyfikuje niczego w xm540_bringup: opis robota to bathset_description/urdf/bathset.urdf.xacro,
który xacro:include'uje nietknięty urdf/xm540_manipulator.urdf z xm540_bringup razem z blokiem
<ros2_control> (osobny plik w bathset_description). manual_node.py ze starej paczki jest
uruchamiany bez żadnej zmiany w kodzie; goal_position_bridge tłumaczy jego stary interfejs
(/servo/goal_position) na komendę forward_position_controller.

Argumenty:
  use_servo        (bool, true)  – czy deklarować w URDF serwo XM540 (wymaga /dev/u2d2)
  use_echosounder  (bool, true)  – czy deklarować w URDF echosondę SLD-100 (wymaga /dev/echosounder)
  use_gnss         (bool, true)  – czy deklarować w URDF odbiornik GNSS mosaic-H (wymaga /dev/gnss)
  use_rviz         (bool, true)  – czy odpalać rviz2

use_servo/use_echosounder NIE są tylko kosmetyczne: controller_manager (ta wersja
ros2_control) twardo pada (abort całego procesu), jeśli zadeklarowany w URDF
komponent sprzętowy nie osiągnie stanu "active" przy starcie - co się zawsze
dzieje, gdy dane urządzenie akurat nie jest fizycznie podpięte. Dlatego gdy np.
tylko echosonda jest podłączona: `ros2 launch hardware_controller real_hardware.launch.py use_servo:=false`.

Węzły (w kolejności startu, zależnie od use_servo/use_echosounder/use_rviz):
  1. robot_state_publisher       – TF z URDF + ros2_control
  2. controller_manager/ros2_control_node
  3. spawnery: joint_state_broadcaster + forward_position_controller (jeśli use_servo),
               range_sensor_broadcaster + temperature_broadcaster (jeśli use_echosounder),
               gnss_broadcaster (jeśli use_gnss)
  4. goal_position_bridge        – most /servo/goal_position -> forward_position_controller (jeśli use_servo)
  5. xm540_bringup/manual_node   – nietknięty, sterowanie ręczne przez /set_orientation (jeśli use_servo)
  6. rviz2 (jeśli use_rviz)
"""

import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawner(controller_name):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller_name, "--controller-manager", "/controller_manager"],
        output="screen",
    )


def _launch_setup(context, *args, **kwargs):
    use_servo = LaunchConfiguration("use_servo").perform(context).lower() == "true"
    use_echosounder = LaunchConfiguration("use_echosounder").perform(context).lower() == "true"
    use_gnss = LaunchConfiguration("use_gnss").perform(context).lower() == "true"
    use_rviz = LaunchConfiguration("use_rviz").perform(context).lower() == "true"

    hw_share = get_package_share_directory("hardware_controller")
    bringup_share = get_package_share_directory("xm540_bringup")
    description_share = get_package_share_directory("bathset_description")

    xacro_path = os.path.join(description_share, "urdf", "bathset.urdf.xacro")
    mappings = {
        "use_servo": "true" if use_servo else "false",
        "use_echosounder": "true" if use_echosounder else "false",
        "use_gnss": "true" if use_gnss else "false",
    }
    robot_description_xml = xacro.process_file(xacro_path, mappings=mappings).toxml()

    # robot_description i controllers.yaml scalone w JEDEN plik parametrów.
    # ros2_control_node w Humble nie stosuje poprawnie parametrów dla kontrolerów
    # (np. sensor_name dla range_sensor_broadcaster), gdy dostaje DWA osobne
    # --params-file (co robi domyślnie Node(parameters=[dict, yaml_path])) -
    # kontroler ładuje się, ale "'sensor_name' parameter has to be specified"
    # przy configure. Jeden połączony plik działa poprawnie (zweryfikowane).
    with open(os.path.join(hw_share, "config", "controllers.yaml")) as f:
        merged_params = yaml.safe_load(f)
    merged_params["/**"] = {"ros__parameters": {"robot_description": robot_description_xml}}

    merged_params_file = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", prefix="hardware_controller_params_", delete=False)
    yaml.safe_dump(merged_params, merged_params_file)
    merged_params_file.close()

    rviz_config = os.path.join(bringup_share, "rviz", "xm540.rviz")

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[merged_params_file.name],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            # Celowo bez name="controller_manager": ros2_control_node domyślnie
            # i tak używa tej nazwy, a jawny name= każe launch_ros dokleić
            # `-r __node:=controller_manager`, co dziedziczą wewnętrzne
            # pod-węzły kontrolerów (np. range_sensor_broadcaster) i przez to
            # nie widzą własnej sekcji parametrów ("sensor_name parameter has
            # to be specified" mimo poprawnego controllers.yaml - zweryfikowane).
            output="screen",
            parameters=[merged_params_file.name],
            # remappings= na controller_manager dziedziczą wewnętrzne pod-węzły
            # kontrolerów (ten sam mechanizm co wyżej z __node, tu użyty celowo) -
            # dzięki temu oba broadcastery echosondy publikują pod wspólnym
            # /echosounder/... zamiast pod nazwą kontrolera (zweryfikowane).
            remappings=[
                ("range_sensor_broadcaster/range", "echosounder/range"),
                ("temperature_broadcaster/temperature", "echosounder/temperature"),
                ("gnss_broadcaster/fix", "gnss/fix"),
                ("gnss_broadcaster/heading", "gnss/heading"),
            ],
        ),
    ]

    # Spawnery czekają na usługi controller_manager, ale dajemy mu chwilę
    # na start przed pierwszą próbą (spójne z typowym wzorcem ros2_control_demos).
    if use_servo:
        nodes.append(TimerAction(period=2.0, actions=[_spawner("joint_state_broadcaster")]))
        nodes.append(TimerAction(period=2.5, actions=[_spawner("forward_position_controller")]))
        nodes.append(Node(
            package="hardware_controller",
            executable="goal_position_bridge",
            name="goal_position_bridge",
            output="screen",
        ))
        nodes.append(Node(
            package="xm540_bringup",
            executable="manual_node",
            name="manual_node",
            output="screen",
            parameters=[{
                "tolerance_deg": 0.5,
                "timeout_s":     5.0,
            }],
        ))

    if use_echosounder:
        nodes.append(TimerAction(period=3.0, actions=[_spawner("range_sensor_broadcaster")]))
        nodes.append(TimerAction(period=3.5, actions=[_spawner("temperature_broadcaster")]))

    if use_gnss:
        nodes.append(TimerAction(period=4.0, actions=[_spawner("gnss_broadcaster")]))

    if use_rviz:
        nodes.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_servo", default_value="true",
                              description="Czy deklarować w URDF serwo XM540 (wymaga /dev/u2d2)"),
        DeclareLaunchArgument("use_echosounder", default_value="true",
                              description="Czy deklarować w URDF echosondę SLD-100 (wymaga /dev/echosounder)"),
        DeclareLaunchArgument("use_gnss", default_value="true",
                              description="Czy deklarować w URDF odbiornik GNSS mosaic-H (wymaga /dev/gnss)"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Czy odpalać rviz2"),
        OpaqueFunction(function=_launch_setup),
    ])
