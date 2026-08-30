"""
sweep_laser.launch.py - eksperyment weryfikacyjny: sweep sferyczny dalmierzem
laserowym na głowicy, z rekonstrukcją odczytów w chmurę punktów.

Cel jest metodologiczny, nie pomiarowy: sprawdzić, czy łańcuch
serwo -> TF -> pomiar -> chmura odtwarza znany obiekt, ZANIM zestaw pójdzie pod
wodę, gdzie nie ma jak porównać wyniku z prawdą.

Składa trzy rzeczy:
  1. real_hardware.launch.py z use_laser:=true i obiema osiami (use_servo_z:=true),
  2. spherical_sweep_node - przechodzi siatkę azymut x elewacja metodą
     dojedź-ustój-zbieraj i bramkuje zbieranie topikiem /sweep/collecting,
  3. cloud_collector_node - przenosi pomiary przez TF do base_link i zapisuje
     CSV + PLY po zakończeniu sweepu.

Przykłady:
  ros2 launch hardware_controller sweep_laser.launch.py
  ros2 launch hardware_controller sweep_laser.launch.py az_max_deg:=30 el_max_deg:=30 az_min_deg:=-30 el_min_deg:=-30
  ros2 launch hardware_controller sweep_laser.launch.py laser_xyz:="-0.137 -0.0555 0"

UWAGA na dwie rzeczy niezmierzone (stan 2026-08-17):
  - laser_xyz domyślnie zerowe, czyli model udaje, że promień wychodzi z osi
    obrotu głowicy. Kształt chmury będzie z grubsza poprawny, ale położenie
    bezwzględne nie - do wniosków ilościowych trzeba zmierzyć montaż.
  - stałe kalibracyjne to karta katalogowa, nie ten egzemplarz. Odległości
    będą obarczone systematycznym błędem rosnącym z dystansem.
Do sprawdzenia SAMEJ poprawności rekonstrukcji (czy płaska ściana wychodzi
płaska, czy kąty się zgadzają) oba braki są akceptowalne.

Domyślnie use_imu:=false i use_echosounder:=false: controller_manager pada,
gdy zadeklarowane urządzenie nie odpowiada, a do tego eksperymentu potrzebne są
tylko serwa i dalmierz.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hw_share = get_package_share_directory("hardware_controller")

    forwarded = [
        "use_imu", "use_echosounder", "use_gnss", "use_rviz",
        "servo_id", "servo_id_z", "center_raw", "center_raw_z",
        "profile_velocity", "profile_acceleration",
        "position_p_gain", "position_i_gain", "position_d_gain",
        "laser_port", "laser_xyz", "laser_rpy",
        "laser_calib_table", "laser_calib_a", "laser_calib_b",
        "laser_type", "jrt_module_baud", "jrt_measure_mode", "jrt_period_ms",
        "jrt_min_range", "jrt_max_range", "jrt_timeout_ms", "jrt_recover_after_misses",
    ]

    real_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_share, "launch", "real_hardware.launch.py")),
        launch_arguments=[
            ("use_servo", "true"),
            ("use_servo_z", "true"),
            ("use_laser", "true"),
        ] + [(name, LaunchConfiguration(name)) for name in forwarded],
    )

    sweep = Node(
        package="hardware_controller",
        executable="spherical_sweep_node",
        name="spherical_sweep_node",
        output="screen",
        parameters=[{
            "az_min_deg":      LaunchConfiguration("az_min_deg"),
            "az_max_deg":      LaunchConfiguration("az_max_deg"),
            "az_step_deg":     LaunchConfiguration("az_step_deg"),
            "el_min_deg":      LaunchConfiguration("el_min_deg"),
            "el_max_deg":      LaunchConfiguration("el_max_deg"),
            "el_step_deg":     LaunchConfiguration("el_step_deg"),
            "settle_time":     LaunchConfiguration("settle_time"),
            "dwell_time":      LaunchConfiguration("dwell_time"),
            "wait_for_measurement": LaunchConfiguration("wait_for_measurement"),
            "max_wait":        LaunchConfiguration("max_wait"),
            "tolerance_deg":   LaunchConfiguration("tolerance_deg"),
            "return_to_zero":  LaunchConfiguration("return_to_zero"),
        }],
    )

    collector = Node(
        package="hardware_controller",
        executable="cloud_collector_node",
        name="cloud_collector_node",
        output="screen",
        parameters=[{
            "target_frame": LaunchConfiguration("target_frame"),
            "output_dir":   LaunchConfiguration("output_dir"),
            "file_prefix":  LaunchConfiguration("file_prefix"),
            # Nazwy kolumn CSV dla dwóch kanałów z /laser/raw. Muszą pasować do
            # czujnika: JRT daje jakość sygnału i kod statusu, Sharp dawał
            # surowe ADC i jego rozrzut.
            "raw_columns":  LaunchConfiguration("raw_columns"),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_imu", default_value="false"),
        DeclareLaunchArgument("use_echosounder", default_value="false"),
        DeclareLaunchArgument("use_gnss", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("servo_id", default_value="1"),
        DeclareLaunchArgument("servo_id_z", default_value="2"),
        # Zera z 2026-08-20; azymut MUSI być 2048, bo +/-180 st mieści się
        # w trybie position tylko wokół środka enkodera. Patrz xm540.ros2_control.xacro.
        DeclareLaunchArgument("center_raw", default_value="1154"),
        DeclareLaunchArgument("center_raw_z", default_value="2048"),
        DeclareLaunchArgument("profile_velocity", default_value="30"),
        DeclareLaunchArgument("profile_acceleration", default_value="10"),
        DeclareLaunchArgument("position_p_gain", default_value="1600"),
        DeclareLaunchArgument("position_i_gain", default_value="150"),
        DeclareLaunchArgument("position_d_gain", default_value="0"),

        DeclareLaunchArgument("laser_port", default_value="/dev/laser"),
        DeclareLaunchArgument("laser_xyz", default_value="0 0 0",
                              description="Montaż dalmierza względem link_2 - NIEZMIERZONY"),
        DeclareLaunchArgument("laser_rpy", default_value="0 3.14159 0"),
        DeclareLaunchArgument("laser_calib_table",
                              default_value="328.77:3.69 336.00:2.93 344.05:2.60 371.40:1.94 437.60:1.34 477.22:1.14 503.03:1.05"),
        DeclareLaunchArgument("laser_calib_a", default_value="134.44"),
        DeclareLaunchArgument("laser_calib_b", default_value="1.1556"),
        # Który dalmierz siedzi na głowicy: "jrt" (obecny) albo "sharp".
        # Szczegóły i progi zakresu w laser.xacro.
        DeclareLaunchArgument("laser_type", default_value="jrt"),
        DeclareLaunchArgument("jrt_module_baud", default_value="38400"),
        DeclareLaunchArgument("jrt_measure_mode", default_value="fast"),
        DeclareLaunchArgument("jrt_period_ms", default_value="0"),
        DeclareLaunchArgument("jrt_min_range", default_value="0.03"),
        DeclareLaunchArgument("jrt_max_range", default_value="100.0"),
        DeclareLaunchArgument("jrt_timeout_ms", default_value="4500",
                              description="BEZPIECZNIK, nie synchronizacja: pomiar modulu trwa wg instrukcji 0,1-4 s. O tym, czy odczyt powstal przy nieruchomej glowicy, decyduje interfejs shot_start"),
        DeclareLaunchArgument("jrt_recover_after_misses", default_value="10"),
        DeclareLaunchArgument("raw_columns", default_value="signal_quality,status_code,shot_start",
                              description="Nazwy kolumn CSV dla /laser/raw; dla laser_type:=sharp podać 'adc,adc_spread,sample_time'"),

        # Domyślna siatka 30x30 stopni co 2 stopnie = 256 punktów, ok. 3,5 min
        # przy settle+dwell = 0,8 s. Świadomie wąska: przy szerokim sweepie
        # większość promieni trafia w podłogę i sufit, gdzie i tak nie ma czego
        # weryfikować, a czas rośnie kwadratowo.
        DeclareLaunchArgument("az_min_deg", default_value="-15.0"),
        DeclareLaunchArgument("az_max_deg", default_value="15.0"),
        DeclareLaunchArgument("az_step_deg", default_value="2.0"),
        DeclareLaunchArgument("el_min_deg", default_value="-15.0"),
        DeclareLaunchArgument("el_max_deg", default_value="15.0"),
        DeclareLaunchArgument("el_step_deg", default_value="2.0"),
        DeclareLaunchArgument("settle_time", default_value="0.4",
                              description="Uspokojenie konstrukcji po dojeździe [s]"),
        DeclareLaunchArgument("dwell_time", default_value="0.4",
                              description="DODATKOWE zbieranie po pierwszym świeżym pomiarze [s]; 0 = ruszaj od razu"),
        DeclareLaunchArgument("wait_for_measurement", default_value="true",
                              description="Czekać na pomiar rozpoczęty po dojeździe zamiast odmierzać dwell zegarem"),
        DeclareLaunchArgument("max_wait", default_value="5.0",
                              description="Górna granica czekania na pomiar w punkcie [s]"),
        DeclareLaunchArgument("tolerance_deg", default_value="0.5"),
        DeclareLaunchArgument("return_to_zero", default_value="true"),

        DeclareLaunchArgument("target_frame", default_value="base_link"),
        DeclareLaunchArgument("output_dir", default_value="",
                              description="Puste = $ROS_LOG_DIR albo ~/.ros/log"),
        DeclareLaunchArgument("file_prefix", default_value="laser_sweep"),

        real_hardware,
        collector,
        # Sweep dopiero, gdy kontrolery są aktywne - spawnery w real_hardware
        # startują do 5 s po launchu, a węzeł od razu zaczyna zadawać pozycje.
        TimerAction(period=8.0, actions=[sweep]),
    ])
