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
_DEFAULT_WP        = _PKG_SHARE / "sweep_waypoints.csv"

LAKE_TRANSLATE      = (0.0, 0.0, -30.0)
LAKE_SCALE          = (10.0, 10.0, 10.0)
LAKE_ROTATE_X_DEG   = 90.0
SONAR_RANGE_MIN     = 0.1
SONAR_RANGE_MAX     = 500.0
SONAR_BEAM_HALF_DEG = 1.0
ROBOT_Z             = 0.0


# ---------------------------------------------------------------------------
def fk_directions_all(theta_z_rad: np.ndarray, theta_y_rad: np.ndarray) -> np.ndarray:
    """Wektoryzowane FK dla wszystkich kombinacji (θ_z, θ_y).

    Zwraca tablicę (n_z, n_y, 3) z kierunkami sonara.
    """
    sz = np.sin(theta_z_rad)[:, None]   # (n_z, 1)
    cz = np.cos(theta_z_rad)[:, None]
    sy = np.sin(theta_y_rad)[None, :]   # (1, n_y)
    cy = np.cos(theta_y_rad)[None, :]

    dirs = np.stack([sy * cz, -sy * sz, -cy * np.ones_like(sz)], axis=-1)  # (n_z, n_y, 3)
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

    return np.array(rays)   # (37, 3)


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
    """Obraca prekomputowane offsety do kierunku central_dir.

    Zwraca listę carb.Float3 — jeden raycast per ray.
    """
    R = rotation_to(central_dir)
    rotated = (R @ offsets.T).T   # (37, 3)
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
    xf.AddScaleOp().Set(Gf.Vec3f(*LAKE_SCALE))


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


def load_waypoints(csv_path: pathlib.Path) -> list:
    waypoints = []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            waypoints.append((float(row["world_x"]), float(row["world_y"])))
    print(f"[sweep_isaac] Wczytano {len(waypoints)} waypointów.")
    return waypoints


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
def main(tiles_dir: pathlib.Path, waypoints_path: pathlib.Path,
         out_path: pathlib.Path, range_deg: float, step_deg: float,
         sweep_z: bool, save_csv: bool) -> None:

    print(f"[sweep_isaac] Ładowanie kafelków: {tiles_dir}")
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    add_lake(stage, tiles_dir)
    world.reset()
    world.step(render=False)

    physx    = get_physx_scene_query_interface()
    waypoints = load_waypoints(waypoints_path)

    theta_y_deg = build_sweep_angles(range_deg, step_deg)
    theta_z_deg = build_sweep_angles(range_deg, step_deg) if sweep_z else [0.0]

    theta_y_rad = np.radians(theta_y_deg)
    theta_z_rad = np.radians(theta_z_deg)

    # (n_z, n_y, 3) — wszystkie kierunki FK, prekomputowane
    all_dirs = fk_directions_all(theta_z_rad, theta_y_rad)
    # (n_z, n_y, 37, 3) — stożki, jeden per pozycja jointa
    cone_offsets = build_cone_offsets(SONAR_BEAM_HALF_DEG)   # (37, 3)

    rays_per_wp = len(theta_z_deg) * len(theta_y_deg)
    total_rays  = len(waypoints) * rays_per_wp
    print(f"[sweep_isaac] {len(waypoints)} waypointów × {rays_per_wp} pozycji = {total_rays:,} raycasts")
    print(f"[sweep_isaac]   θ_z: {theta_z_deg[0]:.0f}°..{theta_z_deg[-1]:.0f}°  "
          f"θ_y: {theta_y_deg[0]:.0f}°..{theta_y_deg[-1]:.0f}°  krok {step_deg}°")

    results = []
    t0      = time.monotonic()
    ray_idx = 0
    origin_arr = np.array([0.0, 0.0, ROBOT_Z])

    for wp_idx, (wx, wy) in enumerate(waypoints):
        origin = carb.Float3(wx, wy, ROBOT_Z)
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

    if save_csv:
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
    parser = argparse.ArgumentParser()
    parser.add_argument("--tiles",     default=str(_DEFAULT_TILES_DIR))
    parser.add_argument("--waypoints", default=str(_DEFAULT_WP))
    parser.add_argument("--out",       default="/workspace/log/bathymetry_sweep_isaac")
    parser.add_argument("--range_deg", type=float, default=90.0,
                        help="Połowa zakresu sweepowania [°]")
    parser.add_argument("--step_deg",  type=float, default=5.0,
                        help="Krok sweepowania [°]")
    parser.add_argument("--no_sweep_z", action="store_true",
                        help="Sweepuj tylko oś Y (domyślnie: obie osie)")
    parser.add_argument("--csv",       action="store_true")
    args = parser.parse_args()

    main(pathlib.Path(args.tiles),
         pathlib.Path(args.waypoints),
         pathlib.Path(args.out),
         range_deg=args.range_deg,
         step_deg=args.step_deg,
         sweep_z=not args.no_sweep_z,
         save_csv=args.csv)
