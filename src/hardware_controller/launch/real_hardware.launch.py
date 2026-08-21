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
  use_imu          (bool, true)  – czy deklarować w URDF IMU GY-955 (wymaga /dev/serial0)
  use_rviz         (bool, false) – czy odpalać rviz2
  servo_id         (int, 1)      – adres serwa na magistrali (siedzi w EEPROM serwa)
  profile_velocity (int, 30)     – profil prędkości serwa; 0 = BEZ profilu, czyli
                                   dojazd z maksymalną prędkością (szarpie głowicą)
  profile_acceleration (int, 10) – profil przyspieszenia; 0 = bez profilu

Domyślnie false, bo ten launch odpala się przede wszystkim na płytce (bathset,
Raspberry Pi 3, headless) - obraz ROS-a jest tam budowany BEZ rviz2, więc
use_rviz:=true wysypałoby launch na braku executable'a. Na PC, gdzie rviz2
jest, wołaj jawnie: `ros2 launch ... real_hardware.launch.py use_rviz:=true`.

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
               imu_sensor_broadcaster (jeśli use_imu)
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
    use_imu = LaunchConfiguration("use_imu").perform(context).lower() == "true"
    use_rviz = LaunchConfiguration("use_rviz").perform(context).lower() == "true"
    use_servo_z = LaunchConfiguration("use_servo_z").perform(context).lower() == "true"
    use_laser = LaunchConfiguration("use_laser").perform(context).lower() == "true"

    hw_share = get_package_share_directory("hardware_controller")
    bringup_share = get_package_share_directory("xm540_bringup")
    description_share = get_package_share_directory("bathset_description")

    xacro_path = os.path.join(description_share, "urdf", "bathset.urdf.xacro")
    mappings = {
        "use_servo": "true" if use_servo else "false",
        "use_echosounder": "true" if use_echosounder else "false",
        "use_gnss": "true" if use_gnss else "false",
        "use_imu": "true" if use_imu else "false",
        "servo_id": LaunchConfiguration("servo_id").perform(context),
        "profile_velocity": LaunchConfiguration("profile_velocity").perform(context),
        "profile_acceleration": LaunchConfiguration("profile_acceleration").perform(context),
        "position_p_gain": LaunchConfiguration("position_p_gain").perform(context),
        "position_i_gain": LaunchConfiguration("position_i_gain").perform(context),
        "position_d_gain": LaunchConfiguration("position_d_gain").perform(context),
        "use_servo_z": "true" if use_servo_z else "false",
        "servo_id_z": LaunchConfiguration("servo_id_z").perform(context),
        "center_raw": LaunchConfiguration("center_raw").perform(context),
        "center_raw_z": LaunchConfiguration("center_raw_z").perform(context),
        "use_laser": "true" if use_laser else "false",
        "laser_port": LaunchConfiguration("laser_port").perform(context),
        "laser_xyz": LaunchConfiguration("laser_xyz").perform(context),
        "laser_rpy": LaunchConfiguration("laser_rpy").perform(context),
        "laser_calib_table": LaunchConfiguration("laser_calib_table").perform(context),
        "laser_calib_a": LaunchConfiguration("laser_calib_a").perform(context),
        "laser_calib_b": LaunchConfiguration("laser_calib_b").perform(context),
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

    # Lista jointów kontrolera MUSI zgadzać się z tym, co zadeklarował URDF -
    # inaczej forward_position_controller nie wystartuje ("joint not found").
    # Trzymanie jej na sztywno w controllers.yaml rozjeżdżałoby się z flagą
    # use_servo_z, więc składamy ją tutaj, z tego samego źródła prawdy.
    joints = ["xm540_joint"] + (["xm540_joint_z"] if use_servo_z else [])
    merged_params["forward_position_controller"]["ros__parameters"]["joints"] = joints

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
                ("imu_sensor_broadcaster/imu", "imu/data"),
                ("laser_broadcaster/range", "laser/range"),
                # Surowe ADC pod /laser/raw - kolektor paruje je z Range po
                # stemplu. Bez tego remapu kolektor nie widzi nic i zapisuje
                # CSV z pustymi kolumnami adc/adc_spread (sprawdzone na żywo).
                ("laser_broadcaster/raw", "laser/raw"),
            ],
        ),
    ]

    # Spawnery czekają na usługi controller_manager, ale dajemy mu chwilę
    # na start przed pierwszą próbą (spójne z typowym wzorcem ros2_control_demos).
    if use_servo:
        nodes.append(TimerAction(period=2.0, actions=[_spawner("joint_state_broadcaster")]))
        nodes.append(TimerAction(period=2.5, actions=[_spawner("forward_position_controller")]))
        # Most i manual_node są z założenia JEDNOOSIOWE: most publikuje komendę
        # jednoelementową, bo /servo/goal_position to pojedynczy Float64. Przy
        # dwóch jointach kontroler odrzuca taką komendę (niezgodny rozmiar),
        # więc przy use_servo_z tej pary nie uruchamiamy - dwiema osiami trzeba
        # sterować czymś, co podaje obie wartości naraz.
        if not use_servo_z:
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

    if use_imu:
        nodes.append(TimerAction(period=4.5, actions=[_spawner("imu_sensor_broadcaster")]))

    if use_laser:
        nodes.append(TimerAction(period=5.0, actions=[_spawner("laser_broadcaster")]))

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
        DeclareLaunchArgument("use_imu", default_value="true",
                              description="Czy deklarować w URDF IMU GY-955 (wymaga /dev/serial0)"),
        DeclareLaunchArgument("use_gnss", default_value="true",
                              description="Czy deklarować w URDF odbiornik GNSS mosaic-H (wymaga /dev/gnss)"),
        DeclareLaunchArgument("use_rviz", default_value="false",
                              description="Czy odpalać rviz2 (na płytce nie ma go w ogóle - patrz docstring)"),
        DeclareLaunchArgument("servo_id", default_value="1",
                              description="Adres serwa XM540 na magistrali Dynamixel (EEPROM serwa, fabrycznie 1)"),
        DeclareLaunchArgument("profile_velocity", default_value="30",
                              description="Profile velocity serwa (0 = bez profilu, maksymalna prędkość!)"),
        DeclareLaunchArgument("profile_acceleration", default_value="10",
                              description="Profile acceleration serwa (0 = bez profilu)"),
        # Nastawy pętli położenia. Fabryczne 800/0/0 zostawiają uchyb ustalony
        # 0,31 st (elewacja) i 0,75 st (azymut) - przy czystym P oś staje tam, gdzie
        # moment P równoważy tarcie. 1600/150 zbija to do 0,02 / 0,15 st, bez
        # przeregulowania i bez wzrostu prądu (pomiar 2026-08-17).
        DeclareLaunchArgument("position_p_gain", default_value="1600",
                              description="Position P Gain serwa (fabrycznie 800)"),
        DeclareLaunchArgument("position_i_gain", default_value="150",
                              description="Position I Gain serwa (fabrycznie 0) - usuwa uchyb ustalony"),
        DeclareLaunchArgument("position_d_gain", default_value="0",
                              description="Position D Gain serwa (fabrycznie 0)"),
        DeclareLaunchArgument("use_servo_z", default_value="false",
                              description="Czy deklarować drugą oś xm540_joint_z (wyłącza most i manual_node)"),
        DeclareLaunchArgument("servo_id_z", default_value="2",
                              description="Adres serwa drugiej osi na magistrali"),
        DeclareLaunchArgument("center_raw", default_value="1154",
                              description="Zero xm540_joint (elewacja); okno EEPROM serwa 130..2178 = +/-90 st. Ustawione 2026-08-20."),
        DeclareLaunchArgument("center_raw_z", default_value="2048",
                              description="Zero xm540_joint_z (azymut). MUSI być 2048: +/-180 st mieści się w trybie position tylko wokół środka enkodera."),
        DeclareLaunchArgument("use_laser", default_value="false",
                              description="Czy deklarować dalmierz Sharp na głowicy (wymaga /dev/laser)"),
        DeclareLaunchArgument("laser_port", default_value="/dev/laser",
                              description="Port szeregowy Arduino Nano z dalmierzem"),
        DeclareLaunchArgument("laser_xyz", default_value="0 0 0",
                              description="Położenie punktu pomiaru dalmierza względem link_2 - NIEZMIERZONE"),
        DeclareLaunchArgument("laser_rpy", default_value="0 3.14159 0",
                              description="Orientacja osi optycznej względem link_2; domyślnie jak sonar w symulacji"),
        DeclareLaunchArgument("laser_calib_table",
                              default_value="328.77:3.69 336.00:2.93 344.05:2.60 371.40:1.94 437.60:1.34 477.22:1.14 503.03:1.05",
                              description="Kalibracja dalmierza: węzły ADC:metry, zmierzone 2026-08-20; "
                                          "między nimi PCHIP, poza skrajnymi NaN. Puste = ścieżka zapasowa calib_a/calib_b"),
        DeclareLaunchArgument("laser_calib_a", default_value="134.44",
                              description="ZAPASOWE (tylko przy pustym laser_calib_table): stała A modelu V=A/L_cm+B z karty katalogowej"),
        DeclareLaunchArgument("laser_calib_b", default_value="1.1556",
                              description="ZAPASOWE (tylko przy pustym laser_calib_table): stała B modelu V=A/L_cm+B z karty katalogowej"),
        OpaqueFunction(function=_launch_setup),
    ])
