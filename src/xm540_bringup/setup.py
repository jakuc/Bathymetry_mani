from setuptools import setup, find_packages
import os
from glob import glob

package_name = "xm540_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"),
            glob("urdf/*.urdf") + glob("urdf/*.urdf.xacro")),
        (os.path.join("share", package_name, "meshes"),
            glob("meshes/*.stl")),
        (os.path.join("share", package_name, "rviz"),
            glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "waypoints"),
            glob("waypoints/*.csv")),
        (os.path.join("share", package_name, "config"),
            glob("config/*.yaml")),
        (os.path.join("share", package_name, "isaac"),
            glob("isaac/*.py")),
    ],
    scripts=["scripts/sim_mani.sh"],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "dynamixel_node = xm540_bringup.dynamixel_node:main",
            "sweep_node     = xm540_bringup.sweep_node:main",
            "manual_node    = xm540_bringup.manual_node:main",
            "sim_driver_node      = xm540_bringup.sim_driver_node:main",
            "scan_collector_node  = xm540_bringup.scan_collector_node:main",
            "waypoint_viz_node        = xm540_bringup.waypoint_viz_node:main",
            "mission_supervisor_node  = xm540_bringup.mission_supervisor_node:main",
            "operator_panel           = xm540_bringup.operator_panel:main",
            "batch_sweep_node         = xm540_bringup.batch_sweep_node:main",
        ],
    },
)
