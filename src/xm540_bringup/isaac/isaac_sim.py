"""
isaac_sim.py – Symulacja manipulatora XM540 w Isaac Sim.

Uruchomienie (w kontenerze):
    OMNI_KIT_ALLOW_ROOT=1 python3 src/xm540_bringup/isaac/isaac_sim.py

Równolegle uruchom węzły ROS2:
    ros2 launch xm540_bringup isaac.launch.py

Subskrybuje:
    /xm540_joint_z/cmd_pos  (std_msgs/Float64) – komenda pozycji osi Z [rad]
    /xm540_joint/cmd_pos    (std_msgs/Float64) – komenda pozycji osi Y [rad]

Publikuje:
    /sonar  (sensor_msgs/LaserScan) – pomiar czujnika odległości, 20 Hz
"""

import math
import pathlib
import time

# SimulationApp musi być wywołana przed wszystkimi importami Isaac Sim
from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "width": 1280,
    "height": 720,
    "renderer": "RayTracedLighting",  # lżejszy od domyślnego PathTracing – mniej shaderów
})

import carb
import numpy as np
import omni.kit.commands
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.prims import XFormPrim
try:
    from isaacsim.core.utils.rotations import euler_angles_to_quat
except ImportError:
    from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.physx import get_physx_scene_query_interface
from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, Gf, Sdf

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64
from xm540_interfaces.srv import SetBoatPose
import tf2_ros
from ament_index_python.packages import get_package_share_directory

# ---------------------------------------------------------------------------
_PKG_SHARE    = pathlib.Path(get_package_share_directory("xm540_bringup"))
URDF_PATH     = str(_PKG_SHARE / "urdf" / "xm540_manipulator_isaac.urdf")
LAKE_OBJ_PATH = str(_PKG_SHARE / "meshes" / "big_lake_simp.obj")
LAKE_TILES_DIR = _PKG_SHARE / "meshes" / "big_lake_simp_tiles"
WAYPOINTS_CSV  = _PKG_SHARE / "waypoints.csv"
SONAR_RATE_HZ          = 20.0
SONAR_RANGE_MIN        = 0.1
SONAR_RANGE_MAX        = 500.0
SONAR_BEAM_HALF_DEG    = 1.0    # półkąt stożka wiązki [°]
SONAR_BEAM_RAYS        = 37     # 1 centralny + 6 + 12 + 18 (trzy pierścienie)

# Transform jeziora
LAKE_TRANSLATE        = (0.0, 0.0, -30.0)
LAKE_SCALE            = (10.0, 10.0, 10.0)
LAKE_VISUAL_ROTATE_X  = 90.0
LAKE_TILES_ROTATE_X   = 90.0


# ---------------------------------------------------------------------------
class IsaacRosNode(Node):
    def __init__(self):
        super().__init__("isaac_sim_node")
        self.cmd_z = 0.0
        self.cmd_y = 0.0
        # Teleportacja łódki — ustawiana przez serwis, czytana w głównej pętli
        self.pending_pose: tuple[float, float] | None = None
        self.create_subscription(Float64, "/xm540_joint_z/cmd_pos", self._cb_z, 10)
        self.create_subscription(Float64, "/xm540_joint/cmd_pos",   self._cb_y, 10)
        self._pub_clock    = self.create_publisher(Clock,       "/clock",        10)
        self._pub_sonar    = self.create_publisher(LaserScan,   "/sim/sonar",    10)
        self._pub_encoders = self.create_publisher(JointState,  "/joint_states", 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_service(SetBoatPose, "/set_boat_pose", self._srv_set_boat_pose)
        self.get_logger().info("IsaacRosNode gotowy.")

    def _srv_set_boat_pose(self, request: SetBoatPose.Request,
                           response: SetBoatPose.Response) -> SetBoatPose.Response:
        """Zleca teleportację łódki — wykonanie w głównej pętli fizyki."""
        self.pending_pose = (request.x, request.y)
        self.get_logger().debug(f"SetBoatPose: x={request.x:.2f} y={request.y:.2f}")
        response.success = True
        return response

    def _cb_z(self, msg: Float64): self.cmd_z = msg.data
    def _cb_y(self, msg: Float64): self.cmd_y = msg.data

    def publish_clock(self, sim_time_sec: float) -> None:
        msg = Clock()
        msg.clock.sec     = int(sim_time_sec)
        msg.clock.nanosec = int((sim_time_sec % 1.0) * 1e9)
        self._pub_clock.publish(msg)

    def broadcast_base_link_tf(self, stage, base_link_prim_path: str, stamp) -> None:
        """Rozgłasza TF world→base_link na podstawie rzeczywistej pozycji w Isaac Sim."""
        prim = stage.GetPrimAtPath(base_link_prim_path)
        if not prim.IsValid():
            return
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t_vec  = matrix.ExtractTranslation()
        q      = matrix.ExtractRotationQuat()
        qi     = q.GetImaginary()

        msg = TransformStamped()
        msg.header.stamp    = stamp.to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id  = "base_link"
        msg.transform.translation.x = float(t_vec[0])
        msg.transform.translation.y = float(t_vec[1])
        msg.transform.translation.z = float(t_vec[2])
        msg.transform.rotation.x    = float(qi[0])
        msg.transform.rotation.y    = float(qi[1])
        msg.transform.rotation.z    = float(qi[2])
        msg.transform.rotation.w    = float(q.GetReal())
        self._tf_broadcaster.sendTransform(msg)

    def publish_encoders(self, pos_z: float, pos_y: float, stamp) -> None:
        msg = JointState()
        msg.header.stamp = stamp.to_msg()
        msg.name         = ["xm540_joint_z", "xm540_joint"]
        msg.position     = [pos_z, pos_y]
        msg.velocity     = [0.0, 0.0]
        msg.effort       = [0.0, 0.0]
        self._pub_encoders.publish(msg)

    def publish_sonar(self, distance: float, stamp) -> None:
        msg = LaserScan()
        msg.header.stamp    = stamp.to_msg()
        msg.header.frame_id = "sonar_link"
        msg.angle_min       = 0.0
        msg.angle_max       = 0.0
        msg.angle_increment = 0.0
        msg.time_increment  = 0.0
        msg.scan_time       = 1.0 / SONAR_RATE_HZ
        msg.range_min       = SONAR_RANGE_MIN
        msg.range_max       = SONAR_RANGE_MAX
        msg.ranges          = [float(distance)]
        msg.intensities     = []
        self._pub_sonar.publish(msg)


# ---------------------------------------------------------------------------
# sonar_joint: rpy="0 3.14159 0" → Ry(π) → oś +X sonar_link = -X link_2
# Pobieramy orientację link_2 (revolute, poprawnie aktualizowany przez fizykę)
# i aplikujemy statyczny transform sonar_joint.
_link2_xform: XFormPrim = None
_sonar_xform: XFormPrim = None

def sonar_ray(sonar_prim_path: str) -> tuple:
    """Zwraca (origin, direction) wiązki sonara pobrane z fizyki Isaac Sim."""
    global _link2_xform, _sonar_xform
    link2_path = sonar_prim_path.rsplit("/", 1)[0] + "/link_2"  # /xm540_manipulator/link_2
    if _link2_xform is None:
        _link2_xform = XFormPrim(link2_path)
    if _sonar_xform is None:
        _sonar_xform = XFormPrim(sonar_prim_path)

    # Pozycja sonara z fizyki (position jest poprawne nawet dla fixed joint)
    position, _ = _sonar_xform.get_world_pose()

    # Orientacja link_2 z fizyki (revolute joint — poprawnie aktualizowana)
    _, orientation = _link2_xform.get_world_pose()
    w, x, y, z = float(orientation[0]), float(orientation[1]), float(orientation[2]), float(orientation[3])

    # Oś +X link_2 w world
    link2_x = np.array([
        1.0 - 2.0*(y*y + z*z),
        2.0*(x*y + w*z),
        2.0*(x*z - w*y),
    ])
    # sonar_joint Ry(π): +X sonar_link = -X link_2
    d = -link2_x

    origin    = carb.Float3(float(position[0]), float(position[1]), float(position[2]))
    direction = carb.Float3(float(d[0]), float(d[1]), float(d[2]))
    return origin, direction


def sonar_cone_cast(physx, origin: carb.Float3, direction: carb.Float3) -> float:
    """Rzuca stożek promieni i zwraca minimum odległości (pierwsze silne echo).

    Wzorzec: 1 promień centralny + 6 promieni równomiernie na krawędzi stożka
    o półkącie SONAR_BEAM_HALF_DEG. Zwraca minimalną odległość z trafionych
    promieni, ograniczoną do [SONAR_RANGE_MIN, SONAR_RANGE_MAX].
    """
    d = np.array([direction.x, direction.y, direction.z], dtype=np.float64)
    d /= np.linalg.norm(d)

    # Wektor prostopadły do d
    ref = np.array([1.0, 0.0, 0.0]) if abs(d[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(d, ref); u /= np.linalg.norm(u)
    v = np.cross(d, u)

    half_rad = math.radians(SONAR_BEAM_HALF_DEG)

    # Kierunki: 1 centralny + 3 pierścienie (6, 12, 18 promieni) = 37 łącznie
    directions = [d]
    rings = [(6, 1/3), (12, 2/3), (18, 1)]   # (liczba promieni, ułamek półkąta)
    for n_rays, frac in rings:
        theta = half_rad * frac
        for i in range(n_rays):
            phi = 2.0 * math.pi * i / n_rays
            ray = math.cos(theta) * d + math.sin(theta) * (math.cos(phi) * u + math.sin(phi) * v)
            ray /= np.linalg.norm(ray)
            directions.append(ray)

    min_d = SONAR_RANGE_MAX
    for ray in directions:
        r = carb.Float3(float(ray[0]), float(ray[1]), float(ray[2]))
        hit = physx.raycast_closest(origin, r, SONAR_RANGE_MAX)
        if hit["hit"]:
            min_d = min(min_d, float(hit["distance"]))

    return max(SONAR_RANGE_MIN, min_d)


def _find_robot_prim_path() -> str:
    """Szuka primu robota w aktualnym stage — sprawdza /World/xm540_manipulator i root."""
    stage = omni.usd.get_context().get_stage()
    candidates = ["/World/xm540_manipulator", "/xm540_manipulator"]
    for path in candidates:
        if stage.GetPrimAtPath(path).IsValid():
            return path
    # ostateczny fallback: pierwsze dziecko /World
    world = stage.GetPrimAtPath("/World")
    if world.IsValid():
        for child in world.GetChildren():
            return child.GetPath().pathString
    return "/World/xm540_manipulator"


def import_urdf(urdf_path: str) -> str:
    """Importuje URDF do aktualnego stage. Zwraca prim_path robota."""
    try:
        from omni.importer.urdf import _urdf as urdf_mod
    except ImportError:
        from isaacsim.asset.importer.urdf import _urdf as urdf_mod

    cfg = urdf_mod.ImportConfig()
    cfg.merge_fixed_joints    = False
    cfg.fix_base              = True
    cfg.import_inertia_tensor = True
    cfg.distance_scale        = 1.0

    omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=cfg,
    )

    return _find_robot_prim_path()


# ---------------------------------------------------------------------------
def _apply_lake_transform(xf: UsdGeom.Xformable, rotate_x: float) -> None:
    """Aplikuje transform jeziora (translate / rotateX / scale)."""
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*LAKE_TRANSLATE))
    xf.AddRotateXOp().Set(rotate_x)
    xf.AddScaleOp().Set(Gf.Vec3f(*LAKE_SCALE))


def add_lake(stage) -> None:
    """Dodaje jezioro do sceny.

    Struktura w drzewie USD:
      /World/big_lake          – Xform nadrzędny (brak transformu – tylko grupowanie)
        /visual                – big_lake.obj + transform, bez kolizji
        /collision             – Xform grupujący kafelki kolizyjne
          /tile_00 … /tile_NN  – kafelki OBJ + transform, CollisionAPI, niewidoczne
    """
    # Nadrzędny Xform – czysty kontener, bez transformu
    UsdGeom.Xform.Define(stage, "/World/big_lake")

    # Warstwa wizualna – transform bezpośrednio na primie z referencją
    visual = UsdGeom.Xform.Define(stage, "/World/big_lake/visual").GetPrim()
    visual.GetReferences().AddReference(LAKE_OBJ_PATH)
    _apply_lake_transform(UsdGeom.Xformable(visual), LAKE_VISUAL_ROTATE_X)
    print("[isaac_sim] Załadowano big_lake/visual (bez kolizji)")

    # Folder kolizyjny
    UsdGeom.Xform.Define(stage, "/World/big_lake/collision")

    tile_files = sorted(LAKE_TILES_DIR.glob("*.obj"))
    if not tile_files:
        print(f"[isaac_sim] WARN: brak kafelków w {LAKE_TILES_DIR}")
        return

    for i, tile_path in enumerate(tile_files):
        prim = UsdGeom.Xform.Define(stage, f"/World/big_lake/collision/tile_{i:02d}").GetPrim()
        prim.GetReferences().AddReference(str(tile_path))
        _apply_lake_transform(UsdGeom.Xformable(prim), LAKE_TILES_ROTATE_X)
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdGeom.Imageable(prim).MakeInvisible()

    print(f"[isaac_sim] Załadowano {len(tile_files)} kafelków kolizyjnych z {LAKE_TILES_DIR}")


# ---------------------------------------------------------------------------
def add_ground_plane(stage, z: float = -10.0) -> None:
    """Dodaje płaszczyznę 100×100 m na wysokości z w world (domyślnie 10 m poniżej manipulatora)."""
    cube = UsdGeom.Cube.Define(stage, "/World/ground_plane")
    cube.GetSizeAttr().Set(1.0)

    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, z))
    xf.AddScaleOp().Set(Gf.Vec3f(100.0, 100.0, 0.1))

    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    cube.GetPrim().SetActive(False)

    print(f"[isaac_sim] Płaszczyzna 100×100 m dodana na z={z} (nieaktywna)")


# ---------------------------------------------------------------------------
def add_waypoint_markers(stage, csv_path: pathlib.Path) -> None:
    """Rysuje waypoints w viewporcie Isaaca jako UsdGeom.Points (jeden prim).

    Nie dodaje kolizji ani fizyki – tylko wizualizacja.
    Jeśli plik CSV nie istnieje, cicho pomija.
    """
    import csv as _csv

    if not csv_path.is_file():
        print(f"[isaac_sim] Brak waypoints.csv ({csv_path}) – pomijam wizualizację.")
        return

    points = []
    with open(csv_path, newline="") as f:
        for row in _csv.DictReader(f):
            points.append(Gf.Vec3f(float(row["world_x"]),
                                   float(row["world_y"]),
                                   float(row["world_z"])))

    if not points:
        return

    prim_path = "/World/waypoint_markers"
    pts = UsdGeom.Points.Define(stage, prim_path)
    pts.GetPointsAttr().Set(points)
    pts.GetWidthsAttr().Set([1.0] * len(points))   # rozmiar punktu [m]

    # Żółty kolor
    display = UsdGeom.Gprim(pts.GetPrim())
    display.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.9, 0.0)] * len(points))

    print(f"[isaac_sim] Załadowano {len(points):,} waypointów do viewportu.")


# ---------------------------------------------------------------------------
def main():
    rclpy.init()
    ros_node = IsaacRosNode()

    print("[isaac_sim] Importuję URDF...")
    world = World(stage_units_in_meters=1.0)
    robot_prim_path = import_urdf(URDF_PATH)
    print(f"[isaac_sim] Import zakończony.")

    print(f"[isaac_sim] Robot: {robot_prim_path}")
    sonar_prim_path     = f"{robot_prim_path}/sonar_link"
    base_link_prim_path = f"{robot_prim_path}/base_link"

    # DomeLight – równomierne oświetlenie ze wszystkich kierunków, brak cieni kierunkowych
    stage = omni.usd.get_context().get_stage()
    dome = UsdLux.DomeLight.Define(stage, "/World/dome_light")
    dome.GetIntensityAttr().Set(300.0)

    add_lake(stage)
    add_ground_plane(stage)
    add_waypoint_markers(stage, WAYPOINTS_CSV)

    robot = world.scene.add(Articulation(prim_path=robot_prim_path))

    world.reset()

    # Robot w origin (0,0,0), obrócony twarzą w dół (180° wokół osi X)
    robot.set_world_pose(
        position=np.array([0.0, 0.0, 0.0]),
        orientation=euler_angles_to_quat(np.array([np.pi, 0.0, 0.0])),
    )

    dof_names = list(robot.dof_names)
    print(f"[isaac_sim] DOFs: {dof_names}")
    idx_z = dof_names.index("xm540_joint_z")
    idx_y = dof_names.index("xm540_joint")

    stage = omni.usd.get_context().get_stage()
    physx = get_physx_scene_query_interface()

    last_sonar_sim_t = 0.0
    sonar_dt         = 1.0 / SONAR_RATE_HZ

    while simulation_app.is_running():
        world.step(render=True)

        rclpy.spin_once(ros_node, timeout_sec=0.0)

        sim_time_sec = world.current_time

        # Publikuj czas symulacji → nody z use_sim_time używają tego jako zegara
        ros_node.publish_clock(sim_time_sec)

        # Teleportacja łódki (zlecona przez /set_boat_pose)
        if ros_node.pending_pose is not None:
            x, y = ros_node.pending_pose
            ros_node.pending_pose = None
            robot.set_world_pose(
                position=np.array([x, y, 0.0]),
                orientation=euler_angles_to_quat(np.array([np.pi, 0.0, 0.0])),
            )

        # Zadaj pozycje jointów z komend ROS2
        positions         = np.zeros(robot.num_dof)
        positions[idx_z]  = ros_node.cmd_z
        positions[idx_y]  = ros_node.cmd_y
        robot.set_joint_positions(positions)

        # Timestamp oparty na czasie symulacji Isaaca
        ros_now = ros_node.get_clock().now()

        # TF world→base_link z rzeczywistej pozycji w Isaac Sim
        ros_node.broadcast_base_link_tf(stage, base_link_prim_path, ros_now)

        # Enkodery z fizyki Isaac Sim → /joint_states
        actual = robot.get_joint_positions()
        ros_node.publish_encoders(float(actual[idx_z]), float(actual[idx_y]), ros_now)

        # Raycast → /sim/sonar (20 Hz sim-czasu)
        if sim_time_sec - last_sonar_sim_t >= sonar_dt:
            last_sonar_sim_t = sim_time_sec
            origin, direction = sonar_ray(sonar_prim_path)
            if origin is not None:
                d = sonar_cone_cast(physx, origin, direction)
                ros_node.publish_sonar(d, ros_now)

    ros_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
