#!/usr/bin/env python3
"""
bathymetry_sweep_isaac.py – Symulacja batymetrii w strategii sweep.

Dla każdego waypointa skanuje 2D siatkę (θ_z × θ_y) i rzuca wiązkę sonara
z uwzględnieniem kinematyki prostej manipulatora XM540.

FK (uproszczona, tylko rotacje):
  base Rx(π) × Rz(θ_z) × Ry(π/2 + θ_y)
  kierunek sonara = -X_link2 = [sin(θ_y)*cos(θ_z), -sin(θ_y)*sin(θ_z), -cos(θ_y)]

Uruchomienie (w kontenerze):
    OMNI_KIT_ALLOW_ROOT=1 python3 src/xm540_bringup/scripts/bathymetry_sweep_isaac.py

Wymagania: Isaac Sim (kontener)
"""

import argparse
import csv
import math
import pathlib
import sys
import time

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "renderer": "RayTracedLighting"})

import carb
import numpy as np
import omni.usd
from omni.isaac.core import World
from omni.physx import get_physx_scene_query_interface
from pxr import UsdGeom, UsdPhysics, Gf
from ament_index_python.packages import get_package_share_directory

# ---------------------------------------------------------------------------
_PKG_SHARE         = pathlib.Path(get_package_share_directory("xm540_bringup"))
_DEFAULT_TILES_DIR = _PKG_SHARE / "meshes" / "big_lake_simp_tiles"
_DEFAULT_LAKE_OBJ  = _PKG_SHARE / "meshes" / "big_lake_simp.obj"

LAKE_TRANSLATE      = (0.0, 0.0, -30.0)
LAKE_SCALE          = 10.0
LAKE_SCALE_VEC      = (LAKE_SCALE, LAKE_SCALE, LAKE_SCALE)
LAKE_ROTATE_X_DEG   = 90.0
LAKE_TRANSLATE_Z    = LAKE_TRANSLATE[2]
SONAR_RANGE_MIN     = 0.1
SONAR_RANGE_MAX     = 500.0
SONAR_BEAM_HALF_DEG = 1.0
ROBOT_Z             = 0.0


# ---------------------------------------------------------------------------
# Generowanie waypointów (przeniesione z generate_waypoints.py)
# ---------------------------------------------------------------------------

def load_mesh(path: pathlib.Path):
    try:
        import trimesh
    except ImportError:
        print("BŁĄD: brak trimesh. Zainstaluj: pip install trimesh", file=sys.stderr)
        sys.exit(1)

    print(f"[waypoints] Wczytywanie mesha: {path}")
    mesh = trimesh.load(str(path), force="mesh")
    print(f"[waypoints]   Wierzchołki: {len(mesh.vertices):,}  Trójkąty: {len(mesh.faces):,}")
    return mesh


def get_contour_polygon(mesh, water_y: float, grid_size: int = 1024):
    """Kontur jeziora z rzutu wierzchołków poniżej water_y na płaszczyznę XZ (raster)."""
    from scipy.ndimage import binary_closing, binary_fill_holes

    print(f"[waypoints] Kontur jeziora (raster Y < {water_y:.4f}, siatka {grid_size}×{grid_size})")

    verts = mesh.vertices[mesh.vertices[:, 1] < water_y]
    if len(verts) == 0:
        print("BŁĄD: brak wierzchołków poniżej water_y.", file=sys.stderr)
        sys.exit(1)

    xs, zs = verts[:, 0], verts[:, 2]
    x_min, x_max = xs.min(), xs.max()
    z_min, z_max = zs.min(), zs.max()

    xi = np.clip(((xs - x_min) / (x_max - x_min) * (grid_size - 1)).astype(int), 0, grid_size - 1)
    zi = np.clip(((zs - z_min) / (z_max - z_min) * (grid_size - 1)).astype(int), 0, grid_size - 1)
    grid = np.zeros((grid_size, grid_size), dtype=bool)
    grid[zi, xi] = True

    closing_px = max(2, grid_size // 100)
    grid = binary_closing(grid, iterations=closing_px)
    grid = binary_fill_holes(grid)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots()
    cs = ax.contour(grid.astype(float), levels=[0.5])
    paths = cs.collections[0].get_paths()
    plt.close(fig)

    if not paths:
        print("BŁĄD: contour nie zwrócił żadnej ścieżki.", file=sys.stderr)
        sys.exit(1)

    main_path = max(paths, key=lambda p: len(p.vertices))
    pix = main_path.vertices

    obj_x = pix[:, 0] / (grid_size - 1) * (x_max - x_min) + x_min
    obj_z = pix[:, 1] / (grid_size - 1) * (z_max - z_min) + z_min

    from shapely.geometry import Polygon as ShapelyPolygon
    lake = ShapelyPolygon(zip(obj_x, obj_z))
    print(f"[waypoints] Powierzchnia konturu (OBJ): {lake.area:.2f}")
    return lake


def generate_grid(polygon, step_obj: float):
    """Siatka punktów wewnątrz konturu, serpentyna wzdłuż X. Zwraca listę shapely.Point."""
    from shapely.geometry import Point

    minx, minz, maxx, maxz = polygon.bounds
    xs = np.arange(minx + step_obj / 2, maxx, step_obj)
    zs = np.arange(minz + step_obj / 2, maxz, step_obj)

    waypoints = []
    for col_idx, x in enumerate(xs):
        col = [Point(x, z) for z in zs if polygon.contains(Point(x, z))]
        if col_idx % 2 == 1:
            col = col[::-1]
        waypoints.extend(col)

    print(f"[waypoints] Waypointów wewnątrz konturu: {len(waypoints):,}")
    return waypoints


def build_waypoints(step_m: float, n_grid: int | None,
                    water_y_arg: str, boat_z: float) -> list[tuple[float, float]]:
    """Generuje listę (world_x, world_y) — punkty trasy łódki."""
    mesh = load_mesh(_DEFAULT_LAKE_OBJ)

    if water_y_arg == "auto":
        water_y = float(mesh.bounds[1][1])
        print(f"[waypoints] Poziom wody (auto): Y = {water_y:.4f}")
    else:
        water_y = float(water_y_arg)

    polygon = get_contour_polygon(mesh, water_y)

    if n_grid is not None:
        minx, minz, maxx, maxz = polygon.bounds
        step_obj = min((maxx - minx), (maxz - minz)) / n_grid
        print(f"[waypoints] Tryb --n-grid {n_grid}: krok OBJ = {step_obj:.4f}")
    else:
        step_obj = step_m / LAKE_SCALE
        print(f"[waypoints] Krok siatki: {step_m} m = {step_obj:.4f} OBJ")

    pts = generate_grid(polygon, step_obj)
    if not pts:
        print("BŁĄD: brak waypointów — sprawdź --water-y i --step", file=sys.stderr)
        sys.exit(1)

    # Transformacja OBJ(x, z) → Isaac Sim world(x, y)
    waypoints = [(LAKE_SCALE * p.x, -LAKE_SCALE * p.y) for p in pts]

    area_world = polygon.area * (LAKE_SCALE ** 2)
    print(f"[waypoints] Powierzchnia jeziora: ~{area_world:.0f} m²  "
          f"gęstość: 1 wp / {area_world/len(waypoints):.1f} m²")
    return waypoints


# ---------------------------------------------------------------------------
# Sweep / raycast
# ---------------------------------------------------------------------------

def fk_directions_all(theta_z_rad: np.ndarray, theta_y_rad: np.ndarray) -> np.ndarray:
    """Wektoryzowane FK dla wszystkich kombinacji (θ_z, θ_y). Zwraca (n_z, n_y, 3)."""
    sz = np.sin(theta_z_rad)[:, None]
    cz = np.cos(theta_z_rad)[:, None]
    sy = np.sin(theta_y_rad)[None, :]
    cy = np.cos(theta_y_rad)[None, :]

    dirs = np.stack([sy * cz, -sy * sz, -cy * np.ones_like(sz)], axis=-1)
    return dirs


def build_cone_offsets(beam_half_deg: float) -> np.ndarray:
    """Prekomputuje 37 offsetów stożka wokół [0,0,-1] jako macierz (37, 3)."""
    d = np.array([0.0, 0.0, -1.0])
    u = np.array([1.0, 0.0,  0.0])
    v = np.array([0.0, 1.0,  0.0])

    half_rad = math.radians(beam_half_deg)
    rays = [d.copy()]
    for n_rays, frac in [(6, 1/3), (12, 2/3), (18, 1)]:
        theta = half_rad * frac
        for i in range(n_rays):
            phi = 2.0 * math.pi * i / n_rays
            ray = math.cos(theta) * d + math.sin(theta) * (math.cos(phi) * u + math.sin(phi) * v)
            rays.append(ray / np.linalg.norm(ray))

    return np.array(rays)


def rotation_to(d: np.ndarray) -> np.ndarray:
    """Macierz obrotu R taka że R @ [0,0,-1] = d (znormalizowany)."""
    d = d / np.linalg.norm(d)
    src = np.array([0.0, 0.0, -1.0])
    cross = np.cross(src, d)
    sin_a = np.linalg.norm(cross)
    cos_a = float(np.dot(src, d))
    if sin_a < 1e-9:
        return np.eye(3) if cos_a > 0 else np.diag([1.0, -1.0, -1.0])
    k = cross / sin_a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + sin_a * K + (1 - cos_a) * K @ K


def apply_cone(central_dir: np.ndarray, offsets: np.ndarray) -> list:
    R = rotation_to(central_dir)
    rotated = (R @ offsets.T).T
    return [carb.Float3(float(r[0]), float(r[1]), float(r[2])) for r in rotated]


def cone_raycast(physx, origin: carb.Float3, dirs: list) -> float:
    min_d = SONAR_RANGE_MAX
    for d in dirs:
        hit = physx.raycast_closest(origin, d, SONAR_RANGE_MAX)
        if hit["hit"]:
            min_d = min(min_d, float(hit["distance"]))
    return max(SONAR_RANGE_MIN, min_d)


# ---------------------------------------------------------------------------

def _apply_transform(xf: UsdGeom.Xformable) -> None:
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*LAKE_TRANSLATE))
    xf.AddRotateXOp().Set(LAKE_ROTATE_X_DEG)
    xf.AddScaleOp().Set(Gf.Vec3f(*LAKE_SCALE_VEC))


def add_lake(stage, tiles_dir: pathlib.Path) -> None:
    UsdGeom.Xform.Define(stage, "/World/lake")
    tile_files = sorted(tiles_dir.glob("*.obj"))
    if not tile_files:
        raise RuntimeError(f"Brak kafelków w {tiles_dir}")
    for i, tile_path in enumerate(tile_files):
        prim = UsdGeom.Xform.Define(stage, f"/World/lake/tile_{i:02d}").GetPrim()
        prim.GetReferences().AddReference(str(tile_path))
        _apply_transform(UsdGeom.Xformable(prim))
        UsdPhysics.CollisionAPI.Apply(prim)
    print(f"[sweep_isaac] Załadowano {len(tile_files)} kafelków kolizyjnych.")


def save_pcd(pts: np.ndarray, path: pathlib.Path) -> None:
    n = len(pts)
    header = (
        f"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
        f"TYPE F F F\nCOUNT 1 1 1\nWIDTH {n}\nHEIGHT 1\n"
        f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA ascii\n"
    )
    with open(path, "w") as f:
        f.write(header)
        for x, y, z in pts:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")


def build_sweep_angles(range_deg: float, step_deg: float) -> list:
    n = int(round(range_deg / step_deg))
    return [i * step_deg for i in range(-n, n + 1)]


# ---------------------------------------------------------------------------
def main(tiles_dir: pathlib.Path,
         step_m: float, n_grid: int | None, water_y: str, boat_z: float,
         out_path: pathlib.Path, range_deg: float, step_deg: float,
         sweep_z: bool, save_csv_flag: bool) -> None:

    # 1. Generuj waypoints
    waypoints = build_waypoints(step_m, n_grid, water_y, boat_z)

    # 2. Inicjalizacja sceny Isaac Sim
    print(f"\n[sweep_isaac] Ładowanie kafelków: {tiles_dir}")
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    add_lake(stage, tiles_dir)
    world.reset()
    world.step(render=False)

    physx = get_physx_scene_query_interface()

    theta_y_deg = build_sweep_angles(range_deg, step_deg)
    theta_z_deg = build_sweep_angles(range_deg, step_deg) if sweep_z else [0.0]

    theta_y_rad = np.radians(theta_y_deg)
    theta_z_rad = np.radians(theta_z_deg)

    all_dirs     = fk_directions_all(theta_z_rad, theta_y_rad)
    cone_offsets = build_cone_offsets(SONAR_BEAM_HALF_DEG)

    rays_per_wp = len(theta_z_deg) * len(theta_y_deg)
    total_rays  = len(waypoints) * rays_per_wp
    print(f"[sweep_isaac] {len(waypoints)} waypointów × {rays_per_wp} pozycji = {total_rays:,} raycasts")
    print(f"[sweep_isaac]   θ_z: {theta_z_deg[0]:.0f}°..{theta_z_deg[-1]:.0f}°  "
          f"θ_y: {theta_y_deg[0]:.0f}°..{theta_y_deg[-1]:.0f}°  krok {step_deg}°")

    results = []
    t0      = time.monotonic()
    ray_idx = 0
    origin_arr = np.array([0.0, 0.0, boat_z])

    for wp_idx, (wx, wy) in enumerate(waypoints):
        origin = carb.Float3(wx, wy, boat_z)
        origin_arr[0], origin_arr[1] = wx, wy

        for iz, tz_deg in enumerate(theta_z_deg):
            for iy, ty_deg in enumerate(theta_y_deg):
                d_arr = all_dirs[iz, iy]
                dirs  = apply_cone(d_arr, cone_offsets)
                dist  = cone_raycast(physx, origin, dirs)

                hit = origin_arr + d_arr * dist
                results.append((wx, wy, tz_deg, ty_deg, dist,
                                float(hit[0]), float(hit[1]), float(hit[2])))
                ray_idx += 1

        if wp_idx == 0 or (wp_idx + 1) % max(1, len(waypoints) // 10) == 0 or wp_idx + 1 == len(waypoints):
            elapsed   = time.monotonic() - t0
            done_frac = ray_idx / total_rays if total_rays else 1
            remaining = (elapsed / done_frac) * (1 - done_frac) if done_frac > 0 else 0
            print(f"  [{wp_idx+1:>4}/{len(waypoints)}] {100*done_frac:5.1f}%  "
                  f"{ray_idx:,} raycasts  ETA: {remaining:.0f}s")

    elapsed = time.monotonic() - t0
    print(f"[sweep_isaac] Gotowe. Czas: {elapsed:.1f}s  ({ray_idx:,} raycasts)")

    pts = np.array([[r[5], r[6], r[7]] for r in results], dtype=np.float32)

    pcd_path = out_path.with_suffix(".pcd")
    save_pcd(pts, pcd_path)
    print(f"[sweep_isaac] PCD → {pcd_path}")

    try:
        import trimesh
        ply_path = out_path.with_suffix(".ply")
        trimesh.PointCloud(pts).export(str(ply_path))
        print(f"[sweep_isaac] PLY → {ply_path}")
    except ImportError:
        pass

    if save_csv_flag:
        csv_path = out_path.with_suffix(".csv")
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(
                f, fieldnames=["world_x", "world_y", "theta_z_deg", "theta_y_deg",
                               "distance", "hit_x", "hit_y", "hit_z"])
            writer.writeheader()
            writer.writerows([{
                "world_x": r[0], "world_y": r[1],
                "theta_z_deg": r[2], "theta_y_deg": r[3],
                "distance": r[4],
                "hit_x": r[5], "hit_y": r[6], "hit_z": r[7],
            } for r in results])
        print(f"[sweep_isaac] CSV → {csv_path}")

    simulation_app.close()


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Batymetria sweep: generuje waypoints, następnie skanuje raycasting w Isaac Sim."
    )

    # Parametry waypoints
    wp = parser.add_argument_group("waypoints")
    wp.add_argument("--step",      type=float, default=2.0,
                    help="Krok siatki waypointów [m] w przestrzeni Isaac Sim (domyślnie 2.0)")
    wp.add_argument("--n-grid",    type=int, default=None,
                    help="Zamiast --step: generuj ~N×N waypointów w jeziorze")
    wp.add_argument("--water-y",   default="auto",
                    help='Poziom wody w OBJ (oś Y). "auto" = max Y mesha.')
    wp.add_argument("--boat-z",    type=float, default=0.0,
                    help="Wysokość łódki w świecie Isaac Sim [m] (domyślnie 0.0)")

    # Parametry symulacji
    sim = parser.add_argument_group("simulation")
    sim.add_argument("--tiles",      default=str(_DEFAULT_TILES_DIR),
                     help="Katalog z kafelkami .obj jeziora")
    sim.add_argument("--out",        default="/workspace/log/bathymetry_sweep_isaac",
                     help="Ścieżka wyjściowa (bez rozszerzenia)")
    sim.add_argument("--range_deg",  type=float, default=90.0,
                     help="Połowa zakresu sweepowania [°]")
    sim.add_argument("--step_deg",   type=float, default=5.0,
                     help="Krok sweepowania [°]")
    sim.add_argument("--no_sweep_z", action="store_true",
                     help="Sweepuj tylko oś Y (domyślnie: obie osie)")
    sim.add_argument("--csv",        action="store_true",
                     help="Zapisz wyniki do CSV oprócz PCD/PLY")

    args = parser.parse_args()

    main(
        tiles_dir    = pathlib.Path(args.tiles),
        step_m       = args.step,
        n_grid       = args.n_grid,
        water_y      = args.water_y,
        boat_z       = args.boat_z,
        out_path     = pathlib.Path(args.out),
        range_deg    = args.range_deg,
        step_deg     = args.step_deg,
        sweep_z      = not args.no_sweep_z,
        save_csv_flag= args.csv,
    )
