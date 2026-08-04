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
from datetime import datetime

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

LAKE_WATER_OBJ_Y  = 3.0   # poziom wody w przestrzeni OBJ; translate_z = -LAKE_WATER_OBJ_Y * lake_scale
LAKE_ROTATE_X_DEG = 90.0
MESH_NATURAL_REDUCTION = 100.0   # OBJ jest pomniejszony 100× względem skali rzeczywistej
SONAR_RANGE_MIN        = 0.1
SONAR_RANGE_MAX        = 500.0
SONAR_BEAM_HALF_DEG    = 0.0   # 0 = single ray (jak w isaac_sim.yaml)

# Kalibracja czasu sweepowania: 7 min na waypoint przy step_deg=2°, range_deg=90° (91²=8281 pozycji)
_SWEEP_TIME_REF_MIN  = 7.0
_RAYS_PER_WP_REF     = 91 * 91


# ---------------------------------------------------------------------------
# Waypoints z CSV
# ---------------------------------------------------------------------------

def load_waypoints_csv(path: pathlib.Path, scale: float) -> list[tuple[float, float]]:
    """Wczytuje waypoints z CSV (skala x1) i skaluje współrzędne przez scale."""
    waypoints = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            waypoints.append((float(row['world_x']) * scale,
                              float(row['world_y']) * scale))
    print(f"[sweep_isaac] Załadowano {len(waypoints)} waypointów → ×{scale:g}")
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
    """Prekomputuje offsety stożka wokół [0,0,-1]. Gdy beam_half_deg=0: jeden promień."""
    d = np.array([0.0, 0.0, -1.0])
    if beam_half_deg == 0.0:
        return np.array([d])
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


def cone_raycast(physx, origin: carb.Float3, dirs: list) -> float | None:
    min_d = float("inf")
    for d in dirs:
        hit = physx.raycast_closest(origin, d, SONAR_RANGE_MAX)
        if hit["hit"]:
            min_d = min(min_d, float(hit["distance"]))
    if min_d == float("inf") or min_d < SONAR_RANGE_MIN:
        return None
    return min_d


# ---------------------------------------------------------------------------

def _apply_transform(xf: UsdGeom.Xformable, lake_scale: float) -> None:
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -LAKE_WATER_OBJ_Y * lake_scale))
    xf.AddRotateXOp().Set(LAKE_ROTATE_X_DEG)
    xf.AddScaleOp().Set(Gf.Vec3f(lake_scale, lake_scale, lake_scale))


def add_lake(stage, tiles_dir: pathlib.Path, lake_scale: float) -> None:
    UsdGeom.Xform.Define(stage, "/World/lake")
    tile_files = sorted(tiles_dir.glob("*.obj"))
    if not tile_files:
        raise RuntimeError(f"Brak kafelków w {tiles_dir}")
    for i, tile_path in enumerate(tile_files):
        prim = UsdGeom.Xform.Define(stage, f"/World/lake/tile_{i:02d}").GetPrim()
        prim.GetReferences().AddReference(str(tile_path))
        _apply_transform(UsdGeom.Xformable(prim), lake_scale)
        UsdPhysics.CollisionAPI.Apply(prim)
    print(f"[sweep_isaac] Załadowano {len(tile_files)} kafelków kolizyjnych.")


def rescale_lake(stage, n_tiles: int, new_scale: float) -> None:
    """Aktualizuje ScaleOp na istniejących primach jeziora (bez przeładowywania OBJ)."""
    for i in range(n_tiles):
        prim = stage.GetPrimAtPath(f"/World/lake/tile_{i:02d}")
        _apply_transform(UsdGeom.Xformable(prim), new_scale)
    print(f"[sweep_isaac] Przeskalowano jezioro → ×{new_scale:g}")


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
         scan_specs: list[dict],
         boat_z: float,
         out_dir: pathlib.Path,
         range_deg: float,
         sweep_z: bool,
         boat_speed: float,
         save_csv_flag: bool) -> None:
    """scan_specs: lista dict z kluczami scale, waypoints_file, step_deg."""

    # Inicjalizacja sceny Isaac Sim — raz dla wszystkich skanów
    tile_files = sorted(tiles_dir.glob("*.obj"))
    n_tiles = len(tile_files)
    print(f"\n[sweep_isaac] Ładowanie {n_tiles} kafelków: {tiles_dir}")
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    add_lake(stage, tiles_dir, scan_specs[0]["scale"])
    world.reset()
    world.step(render=False)
    physx = get_physx_scene_query_interface()

    cone_offsets    = build_cone_offsets(SONAR_BEAM_HALF_DEG)
    ts              = datetime.now().strftime("%Y%m%d_%H%M%S")
    prev_scale      = scan_specs[0]["scale"]
    cached_step_deg = None
    theta_y_deg = theta_z_deg = theta_y_rad = theta_z_rad = all_dirs = None
    wp_params: list[dict] = []

    for spec_idx, spec in enumerate(scan_specs):
        lake_scale     = spec["scale"]
        waypoints_file = spec["waypoints_file"]
        step_deg       = spec["step_deg"]

        print(f"\n{'='*60}")
        print(f"[sweep_isaac] Skan {spec_idx+1}/{len(scan_specs)}  "
              f"×{lake_scale:g}  wp={waypoints_file.name}  step={step_deg}°")
        print(f"{'='*60}")

        if spec_idx > 0 and lake_scale != prev_scale:
            rescale_lake(stage, n_tiles, lake_scale)
            world.reset()
            world.step(render=False)
            prev_scale = lake_scale

        if step_deg != cached_step_deg:
            theta_y_deg     = build_sweep_angles(range_deg, step_deg)
            theta_z_deg     = build_sweep_angles(range_deg, step_deg) if sweep_z else [0.0]
            theta_y_rad     = np.radians(theta_y_deg)
            theta_z_rad     = np.radians(theta_z_deg)
            all_dirs        = fk_directions_all(theta_z_rad, theta_y_rad)
            cached_step_deg = step_deg
            print(f"[sweep_isaac]   θ_z: {theta_z_deg[0]:.0f}°..{theta_z_deg[-1]:.0f}°  "
                  f"θ_y: {theta_y_deg[0]:.0f}°..{theta_y_deg[-1]:.0f}°  krok {step_deg}°  "
                  f"({len(theta_z_deg)*len(theta_y_deg)} pozycji/wp)")

        rays_per_wp       = len(theta_z_deg) * len(theta_y_deg)
        waypoints         = load_waypoints_csv(waypoints_file, lake_scale)
        sweep_time_per_wp = _SWEEP_TIME_REF_MIN / _RAYS_PER_WP_REF * rays_per_wp
        path_length       = sum(
            math.sqrt((waypoints[i+1][0] - waypoints[i][0])**2 +
                      (waypoints[i+1][1] - waypoints[i][1])**2)
            for i in range(len(waypoints) - 1)
        )
        transport_time    = path_length / (boat_speed * 60.0)
        time_min          = sweep_time_per_wp * len(waypoints) + transport_time
        print(f"[sweep_isaac] sweep_time_per_wp={sweep_time_per_wp:.2f} min  "
              f"total_sweep={sweep_time_per_wp*len(waypoints):.2f} min  "
              f"transport={transport_time:.2f} min  path={path_length:.1f} m")
        total_rays  = len(waypoints) * rays_per_wp
        print(f"[sweep_isaac] {len(waypoints)} wp × {rays_per_wp} pozycji = {total_rays:,} raycasts  "
              f"(czas misji: {time_min:.1f} min)")

        results    = []
        t0         = time.monotonic()
        ray_idx    = 0
        origin_arr = np.array([0.0, 0.0, boat_z])

        for wp_idx, (wx, wy) in enumerate(waypoints):
            origin = carb.Float3(wx, wy, boat_z)
            origin_arr[0], origin_arr[1] = wx, wy

            for iz, tz_deg in enumerate(theta_z_deg):
                for iy, ty_deg in enumerate(theta_y_deg):
                    d_arr = all_dirs[iz, iy]
                    dirs  = apply_cone(d_arr, cone_offsets)
                    dist  = cone_raycast(physx, origin, dirs)
                    ray_idx += 1
                    if dist is None:
                        continue
                    hit = origin_arr + d_arr * dist
                    results.append((wx, wy, tz_deg, ty_deg, dist,
                                    float(hit[0]), float(hit[1]), float(hit[2])))

            if wp_idx == 0 or (wp_idx + 1) % max(1, len(waypoints) // 10) == 0 or wp_idx + 1 == len(waypoints):
                elapsed   = time.monotonic() - t0
                done_frac = ray_idx / total_rays if total_rays else 1
                remaining = (elapsed / done_frac) * (1 - done_frac) if done_frac > 0 else 0
                print(f"  [{wp_idx+1:>4}/{len(waypoints)}] {100*done_frac:5.1f}%  "
                      f"{ray_idx:,} raycasts  ETA: {remaining:.0f}s")

        elapsed = time.monotonic() - t0
        print(f"[sweep_isaac] skan {spec_idx+1} gotowy. Czas: {elapsed:.1f}s  ({ray_idx:,} raycasts)")
        wp_params.append({"lake_scale": lake_scale, "time_min": time_min})

        pts  = np.array([[r[5], r[6], r[7]] for r in results], dtype=np.float32)
        stem = (f"sweep_x{lake_scale:g}_wp{len(waypoints)}_r{range_deg:g}s{step_deg:g}"
                f"_{time_min:.1f}min_{ts}")

        pcd_path = out_dir / (stem + ".pcd")
        save_pcd(pts, pcd_path)
        print(f"[sweep_isaac] PCD → {pcd_path}")

        try:
            import trimesh
            ply_path = out_dir / (stem + ".ply")
            trimesh.PointCloud(pts).export(str(ply_path))
            print(f"[sweep_isaac] PLY → {ply_path}")
        except ImportError:
            pass

        if save_csv_flag:
            csv_path = out_dir / (stem + ".csv")
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

    wp_params_path = pathlib.Path.cwd() / "baseline_waypoints.csv"
    with open(wp_params_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["lake_scale", "time_min"])
        writer.writeheader()
        writer.writerows(wp_params)
    print(f"[sweep_isaac] Params dla generate_waypoints → {wp_params_path}")

    simulation_app.close()


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    _DEFAULT_WP_FILE = _PKG_SHARE / "waypoints" / "waypoints_sweep_2pt_x1.csv"

    parser = argparse.ArgumentParser(
        description="Batymetria sweep: wczytuje waypoints z CSV, skanuje raycasting dla wielu skal jeziora."
    )
    parser.add_argument("--scales", type=float, nargs="+", default=[10.0],
                        help="Lista skal jeziora (domyślnie [10]). "
                             "Mesh ładowany raz, między skalami tylko zmiana ScaleOp + world.reset().")
    parser.add_argument("--max-scale", type=int, default=None,
                        help="Skrót: uruchom dla skal 1, 2, ..., N (nadpisuje --scales).")
    parser.add_argument("--waypoints-file", type=pathlib.Path, default=_DEFAULT_WP_FILE,
                        help="CSV z waypointami w skali x1 (kolumny: world_x, world_y). "
                             "Współrzędne mnożone przez scale dla każdej iteracji.")
    parser.add_argument("--boat-z",        type=float, default=-0.05,
                        help="Wysokość sonara [m] (domyślnie -0.05: z URDF sonar_link przy joint=0)")
    parser.add_argument("--tiles",         default=str(_DEFAULT_TILES_DIR),
                        help="Katalog z kafelkami .obj jeziora")
    parser.add_argument("--out",           default="/workspace/log",
                        help="Katalog wyjściowy")
    parser.add_argument("--range_deg",     type=float, default=90.0,
                        help="Połowa zakresu sweepowania [°] (domyślnie 90)")
    parser.add_argument("--step_deg",      type=float, default=2.0,
                        help="Krok sweepowania [°] (domyślnie 2, jak mission.yaml sweep_step_deg)")
    parser.add_argument("--no_sweep_z",    action="store_true",
                        help="Sweepuj tylko oś Y (domyślnie: obie osie)")
    parser.add_argument("--sweep-time",    type=float, default=14.0,
                        help="Stały czas sweepowania [min], niezależny od skali (domyślnie 14.0)")
    parser.add_argument("--boat-speed", type=float, default=1.0,
                        help="Prędkość łódki [m/s] używana do obliczenia czasu transportu (domyślnie 1.0)")
    parser.add_argument("--csv",           action="store_true",
                        help="Zapisz wyniki do CSV oprócz PCD/PLY")
    parser.add_argument("--params-csv", type=pathlib.Path, default=None,
                        help="CSV z parametrami skanów (kolumny: scale, waypoints_file, step_deg). "
                             "Każdy wiersz = jeden skan, posortowane rosnąco po scale. "
                             "Nadpisuje --scales/--max-scale, --waypoints-file i --step_deg.")

    args = parser.parse_args()

    if args.params_csv is not None:
        import csv as _csv
        with open(args.params_csv) as f:
            rows = list(_csv.DictReader(f))
        scan_specs = sorted([
            {
                "scale":          float(r["scale"]),
                "waypoints_file": pathlib.Path(r["waypoints_file"]),
                "step_deg":       float(r["step_deg"]),
            }
            for r in rows
        ], key=lambda s: s["scale"])
        print(f"[sweep_isaac] --params-csv: {len(scan_specs)} skanów z {args.params_csv}")
    else:
        scales = list(range(1, args.max_scale + 1)) if args.max_scale is not None else args.scales
        scan_specs = [
            {
                "scale":          float(s),
                "waypoints_file": args.waypoints_file,
                "step_deg":       args.step_deg,
            }
            for s in scales
        ]

    main(
        tiles_dir      = pathlib.Path(args.tiles),
        scan_specs     = scan_specs,
        boat_z         = args.boat_z,
        out_dir        = pathlib.Path(args.out),
        range_deg      = args.range_deg,
        sweep_z        = not args.no_sweep_z,
        boat_speed     = args.boat_speed,
        save_csv_flag  = args.csv,
    )
