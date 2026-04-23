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
from collections import deque

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
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, Float64
from xm540_interfaces.srv import SetBoatPose
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
# Nadpisywane w main() na podstawie parametru lake_scale
LAKE_SCALE            = (1.0, 1.0, 1.0)
LAKE_TRANSLATE        = (0.0, 0.0, -3.0)
LAKE_VISUAL_ROTATE_X  = 90.0    # taki sam jak kafelki — big_lake.obj wyrównany do world
LAKE_TILES_ROTATE_X   = 90.0


# ---------------------------------------------------------------------------
BOAT_SPEED             = 1.0   # m/s — domyślna prędkość łódki (nadpisywana przez config)
BOAT_ARRIVAL_TOLERANCE = 0.05  # m — tolerancja dojazdu


class IsaacRosNode(Node):
    def __init__(self):
        super().__init__("isaac_sim_node")
        self.cmd_z = 0.0
        self.cmd_y = 0.0
        self.boat_queue: deque[tuple[float, float]] = deque()
        self.boat_vel:   list[float] = [0.0, 0.0]   # [vx, vy] zadane do jointów
        self._wp_target_prev: tuple[float, float] | None = None
        self._wp_t_enter:   float = 0.0
        self._wp_timeout:   float = 0.0

        # Wypełniane przez main() po imporcie URDF
        self.robot       = None
        self.idx_boat_x  = 0
        self.idx_boat_y  = 1

        self.declare_parameter("world_scale",            1.0)
        self.declare_parameter("boat_speed",            BOAT_SPEED)
        self.declare_parameter("boat_arrival_tolerance", BOAT_ARRIVAL_TOLERANCE)
        self.declare_parameter("boat_waypoint_timeout_factor", 3.0)  # timeout = dist/speed * factor
        self.declare_parameter("sonar_rate_hz",          SONAR_RATE_HZ)
        self.declare_parameter("sonar_range_max",        SONAR_RANGE_MAX)
        self.declare_parameter("sonar_beam_half_deg",    SONAR_BEAM_HALF_DEG)

        self.create_subscription(Float64, "/xm540_joint_z/cmd_pos", self._cb_z, 10)
        self.create_subscription(Float64, "/xm540_joint/cmd_pos",   self._cb_y, 10)
        self._pub_sonar    = self.create_publisher(LaserScan,   "/sim/sonar",    10)
        self._pub_encoders = self.create_publisher(JointState,  "/joint_states", 10)
        self._pub_arrived  = self.create_publisher(Bool,        "/boat_arrived", 10)
        self.create_service(SetBoatPose, "/set_boat_pose",    self._srv_set_boat_pose)
        self.create_service(SetBoatPose, "/teleport_boat",    self._srv_teleport_boat)
        self.get_logger().info("IsaacRosNode gotowy.")

    def _srv_teleport_boat(self, request: SetBoatPose.Request,
                           response: SetBoatPose.Response) -> SetBoatPose.Response:
        """Teleportuje łódkę natychmiast do (x, y) przez direct set_joint_positions."""
        if self.robot is not None:
            self.boat_queue.clear()
            self.robot.set_joint_positions(
                np.array([request.x, request.y]),
                joint_indices=np.array([self.idx_boat_x, self.idx_boat_y]),
            )
            self.boat_vel = [0.0, 0.0]
            self.get_logger().info(f"Teleport: x={request.x:.1f} y={request.y:.1f}")
        response.success = True
        return response

    def _srv_set_boat_pose(self, request: SetBoatPose.Request,
                           response: SetBoatPose.Response) -> SetBoatPose.Response:
        """Dodaje waypoint do kolejki nawigacyjnej łódki."""
        self.boat_queue.append((request.x, request.y))
        self.get_logger().debug(
            f"SetBoatPose (kolejka +1={len(self.boat_queue)}): x={request.x:.2f} y={request.y:.2f}"
        )
        response.success = True
        return response

    def update_boat_velocity(self, pos_x: float, pos_y: float) -> None:
        """Liczy prędkości jointów łódki w kierunku aktualnego waypointu.

        Po dotarciu: publikuje /boat_arrived, usuwa waypoint z kolejki i płynie
        do następnego bez zatrzymywania. Jeśli kolejka pusta — stoi.
        """
        if not self.boat_queue:
            self.boat_vel = [0.0, 0.0]
            return
        target_x, target_y = self.boat_queue[0]
        dx   = target_x - pos_x
        dy   = target_y - pos_y
        dist = math.sqrt(dx * dx + dy * dy)

        if (target_x, target_y) != self._wp_target_prev:
            self._wp_target_prev = (target_x, target_y)
            self._wp_t_enter     = time.monotonic()
            speed  = self.get_parameter("boat_speed").value
            factor = self.get_parameter("boat_waypoint_timeout_factor").value
            self._wp_timeout = (dist / speed) * factor

        elapsed = time.monotonic() - self._wp_t_enter
        if (dist <= self.get_parameter("boat_arrival_tolerance").value or
                elapsed >= self._wp_timeout):
            if elapsed >= self._wp_timeout:
                self.get_logger().warn(
                    f"Waypoint timeout ({self._wp_timeout:.1f}s), skip: "
                    f"({target_x:.2f}, {target_y:.2f})  dist={dist:.2f} m"
                )
            self.boat_queue.popleft()
            self._wp_target_prev = None
            self._pub_arrived.publish(Bool(data=True))
            return
        speed = self.get_parameter("boat_speed").value
        self.boat_vel = [speed * dx / dist, speed * dy / dist]

    def _cb_z(self, msg: Float64): self.cmd_z = msg.data
    def _cb_y(self, msg: Float64): self.cmd_y = msg.data

    def publish_encoders(self, pos_z: float, pos_y: float,
                         boat_x: float, boat_y: float, stamp) -> None:
        msg = JointState()
        msg.header.stamp = stamp.to_msg()
        msg.name         = ["xm540_joint_z", "xm540_joint", "joint_boat_x", "joint_boat_y"]
        msg.position     = [pos_z, pos_y, boat_x, boat_y]
        msg.velocity     = [0.0, 0.0, 0.0, 0.0]
        msg.effort       = [0.0, 0.0, 0.0, 0.0]
        self._pub_encoders.publish(msg)

    def publish_sonar(self, distance: float, stamp) -> None:
        rate_hz   = self.get_parameter("sonar_rate_hz").value
        range_max = self.get_parameter("sonar_range_max").value
        msg = LaserScan()
        msg.header.stamp    = stamp.to_msg()
        msg.header.frame_id = "sonar_link"
        msg.angle_min       = 0.0
        msg.angle_max       = 0.0
        msg.angle_increment = 0.0
        msg.time_increment  = 0.0
        msg.scan_time       = 1.0 / rate_hz
        msg.range_min       = SONAR_RANGE_MIN
        msg.range_max       = range_max
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


def sonar_cone_cast(physx, origin: carb.Float3, direction: carb.Float3,
                    beam_half_deg: float, range_max: float) -> float:
    """Rzuca stożek promieni i zwraca minimum odległości (pierwsze silne echo)."""
    d = np.array([direction.x, direction.y, direction.z], dtype=np.float64)
    d /= np.linalg.norm(d)

    ref = np.array([1.0, 0.0, 0.0]) if abs(d[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(d, ref); u /= np.linalg.norm(u)
    v = np.cross(d, u)

    half_rad = math.radians(beam_half_deg)

    directions = [d]
    for n_rays, frac in [(6, 1/3), (12, 2/3), (18, 1)]:
        theta = half_rad * frac
        for i in range(n_rays):
            phi = 2.0 * math.pi * i / n_rays
            ray = math.cos(theta) * d + math.sin(theta) * (math.cos(phi) * u + math.sin(phi) * v)
            ray /= np.linalg.norm(ray)
            directions.append(ray)

    min_d = range_max
    for ray in directions:
        r = carb.Float3(float(ray[0]), float(ray[1]), float(ray[2]))
        hit = physx.raycast_closest(origin, r, range_max)
        if hit["hit"]:
            min_d = min(min_d, float(hit["distance"]))

    return max(SONAR_RANGE_MIN, min_d)   # SONAR_RANGE_MIN pozostaje stałą (fizyczny limit sprzętu)


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
    marker_size = max(0.05, LAKE_SCALE[0] * 0.1)
    pts.GetWidthsAttr().Set([marker_size] * len(points))

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
    sonar_prim_path = f"{robot_prim_path}/sonar_link"

    # DomeLight – równomierne oświetlenie ze wszystkich kierunków, brak cieni kierunkowych
    stage = omni.usd.get_context().get_stage()
    dome = UsdLux.DomeLight.Define(stage, "/World/dome_light")
    dome.GetIntensityAttr().Set(300.0)

    global LAKE_SCALE, LAKE_TRANSLATE
    s = ros_node.get_parameter("world_scale").value
    LAKE_SCALE     = (s, s, s)
    LAKE_TRANSLATE = (0.0, 0.0, -s * 3.0)
    print(f"[isaac_sim] lake_scale={s}  translate_z={-s*3.0:.1f}")

    add_lake(stage)
    add_ground_plane(stage)
    add_waypoint_markers(stage, WAYPOINTS_CSV)

    robot = world.scene.add(Articulation(prim_path=robot_prim_path))

    world.reset()

    # Wyłącz position drive dla wszystkich kontrolowanych jointów — Isaac Sim tworzy go
    # domyślnie przy imporcie URDF (bo <limit effort=...> jest zdefiniowany). Z niezerowym
    # stiffness drive walczy z set_joint_velocities/set_joint_positions, uniemożliwiając ruch.
    stage_after_reset = omni.usd.get_context().get_stage()
    drives_to_disable = {
        "joint_boat_x": "linear",
        "joint_boat_y": "linear",
        "xm540_joint_z": "angular",
        "xm540_joint":   "angular",
    }
    for joint_name, drive_type in drives_to_disable.items():
        found = False
        for prim in stage_after_reset.Traverse():
            if prim.GetName() == joint_name and prim.IsA(UsdPhysics.Joint):
                drive = UsdPhysics.DriveAPI.Apply(prim, drive_type)
                drive.GetStiffnessAttr().Set(0.0)
                drive.GetDampingAttr().Set(0.0)
                print(f"[isaac_sim] Drive wyłączony: {prim.GetPath()}")
                found = True
                break
        if not found:
            print(f"[isaac_sim] WARN: nie znaleziono primu joint dla {joint_name}")

    dof_names = list(robot.dof_names)
    print(f"[isaac_sim] DOFs: {dof_names}")
    idx_boat_x = dof_names.index("joint_boat_x")
    idx_boat_y = dof_names.index("joint_boat_y")
    idx_z      = dof_names.index("xm540_joint_z")
    idx_y      = dof_names.index("xm540_joint")

    ros_node.robot      = robot
    ros_node.idx_boat_x = idx_boat_x
    ros_node.idx_boat_y = idx_boat_y

    stage = omni.usd.get_context().get_stage()
    physx = get_physx_scene_query_interface()

    last_sonar_t = time.monotonic()
    sonar_dt     = 1.0 / ros_node.get_parameter("sonar_rate_hz").value
    step_dt      = 1.0 / 60.0  # limit 60 fps

    while simulation_app.is_running():
        t0 = time.monotonic()
        world.step(render=True)

        rclpy.spin_once(ros_node, timeout_sec=0.0)

        # Odczytaj aktualną pozycję łódki z fizyki i zaktualizuj prędkości
        actual = robot.get_joint_positions()
        ros_node.update_boat_velocity(float(actual[idx_boat_x]), float(actual[idx_boat_y]))

        # Prędkości łódki — tylko jointy łódki, żeby nie nadpisywać pozycji manipulatora
        robot.set_joint_velocities(
            np.array([ros_node.boat_vel[0], ros_node.boat_vel[1]]),
            joint_indices=np.array([idx_boat_x, idx_boat_y]),
        )

        # Pozycje manipulatora — tylko jointy manipulatora, żeby nie resetować pozycji łódki
        robot.set_joint_positions(
            np.array([ros_node.cmd_z, ros_node.cmd_y]),
            joint_indices=np.array([idx_z, idx_y]),
        )

        # Jeden timestamp dla wszystkich wiadomości tej iteracji fizyki
        ros_now = ros_node.get_clock().now()

        # Enkodery z fizyki Isaac Sim → /joint_states (wszystkie 4 jointy)
        # robot_state_publisher buduje pełny łańcuch TF na podstawie tych pozycji
        ros_node.publish_encoders(float(actual[idx_z]), float(actual[idx_y]),
                                  float(actual[idx_boat_x]), float(actual[idx_boat_y]), ros_now)

        # Raycast → /sim/sonar (20 Hz)
        now_wall = time.monotonic()
        if now_wall - last_sonar_t >= sonar_dt:
            last_sonar_t = now_wall
            origin, direction = sonar_ray(sonar_prim_path)
            if origin is not None:
                d = sonar_cone_cast(physx, origin, direction,
                                    ros_node.get_parameter("sonar_beam_half_deg").value,
                                    ros_node.get_parameter("sonar_range_max").value)
                ros_node.publish_sonar(d, ros_now)

        elapsed = time.monotonic() - t0
        if elapsed < step_dt:
            time.sleep(step_dt - elapsed)

    ros_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
