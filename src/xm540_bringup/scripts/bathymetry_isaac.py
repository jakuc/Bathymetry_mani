#!/usr/bin/env python3
"""
bathymetry_isaac.py – Szybka symulacja batymetryczna używając PhysX GPU z Isaac Sim.

Nie używa ROS ani nodów — Isaac służy wyłącznie jako silnik raycastingu.

Uruchomienie (w kontenerze):
    OMNI_KIT_ALLOW_ROOT=1 python3 src/xm540_bringup/scripts/bathymetry_isaac.py

Wymagania: Isaac Sim (kontener)
"""

import argparse
import csv
import math
import pathlib
import re
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
from pxr import UsdGeom, UsdPhysics, Gf, Sdf
from ament_index_python.packages import get_package_share_directory

# ---------------------------------------------------------------------------
_PKG_SHARE              = pathlib.Path(get_package_share_directory("xm540_bringup"))
_DEFAULT_TILES_DIR      = _PKG_SHARE / "meshes" / "big_lake_simp_tiles"
_DEFAULT_LAKE_OBJ       = _PKG_SHARE / "meshes" / "big_lake_simp.obj"
_DEFAULT_WAYPOINTS_DIR  = _PKG_SHARE / "waypoints"

LAKE_WATER_OBJ_Y       = 3.0     # poziom wody w przestrzeni OBJ; translate_z = -LAKE_WATER_OBJ_Y * lake_scale
LAKE_ROTATE_X_DEG      = 90.0
MESH_NATURAL_REDUCTION = 100.0   # OBJ jest pomniejszony 100× względem skali rzeczywistej
SONAR_RANGE_MIN        = 0.1
SONAR_RANGE_MAX        = 500.0
SONAR_BEAM_HALF_DEG    = 1.0


# ---------------------------------------------------------------------------
# Generowanie waypointów
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
                    water_y_arg: str, boat_z: float,
                    lake_scale: float) -> list[tuple[float, float]]:
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
        step_obj = step_m / lake_scale
        print(f"[waypoints] Krok siatki: {step_m} m = {step_obj:.4f} OBJ")

    pts = generate_grid(polygon, step_obj)
    if not pts:
        print("BŁĄD: brak waypointów — sprawdź --water-y i --step", file=sys.stderr)
        sys.exit(1)

    waypoints = [(lake_scale * p.x, -lake_scale * p.y) for p in pts]

    area_world = polygon.area * (lake_scale ** 2)
    print(f"[waypoints] Powierzchnia jeziora: ~{area_world:.0f} m²  "
          f"gęstość: 1 wp / {area_world/len(waypoints):.1f} m²")
    return waypoints


def load_waypoints_csv(path: pathlib.Path) -> list[tuple[float, float]]:
    """Wczytuje waypoints z CSV (format: idx,world_x,world_y,world_z)."""
    waypoints = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            waypoints.append((float(row["world_x"]), float(row["world_y"])))
    print(f"[waypoints] Wczytano {len(waypoints):,} waypointów z {path.name}")
    return waypoints


def generate_boundary_grid(polygon, step_obj: float):
    """Waypoints jako pary (wejście, wyjście) na każdym wierszu konturu — serpentyna."""
    from shapely.geometry import LineString, Point

    minx, minz, maxx, maxz = polygon.bounds
    zs = np.arange(minz + step_obj / 2, maxz, step_obj)
    waypoints = []
    for row_idx, z in enumerate(zs):
        row_line = LineString([(minx - 1.0, z), (maxx + 1.0, z)])
        isect = polygon.intersection(row_line)
        if isect.is_empty or isect.geom_type == "Point":
            continue
        if isect.geom_type == "LineString":
            segs = [isect]
        elif isect.geom_type == "MultiLineString":
            segs = sorted(isect.geoms, key=lambda s: min(c[0] for c in s.coords))
        else:
            continue
        seg_endpoints = []
        for seg in segs:
            coords = sorted(seg.coords, key=lambda c: c[0])
            seg_endpoints.append((Point(coords[0]), Point(coords[-1])))
        if row_idx % 2 == 1:
            seg_endpoints = [(e, s) for s, e in reversed(seg_endpoints)]
        for start, end in seg_endpoints:
            waypoints.append(start)
            waypoints.append(end)
    return waypoints


def path_length_world(waypoints, lake_scale: float) -> float:
    total = 0.0
    for i in range(len(waypoints) - 1):
        dx = (waypoints[i + 1].x - waypoints[i].x) * lake_scale
        dz = (waypoints[i + 1].y - waypoints[i].y) * lake_scale
        total += math.sqrt(dx * dx + dz * dz)
    return total


def find_step_for_time(polygon, target_time_s: float, boat_speed: float,
                       lake_scale: float, tol_frac: float = 0.005) -> list:
    """Bisection: szuka step_obj tak by długość trasy ≈ target_time_s * boat_speed."""
    target_dist = target_time_s * boat_speed
    print(f"[waypoints] Bisection: cel={target_dist:.1f} m  "
          f"({target_time_s/60:.1f} min × {boat_speed} m/s)")

    minx, minz, maxx, maxz = polygon.bounds
    step_lo = 0.05 / lake_scale
    step_hi = min(maxx - minx, maxz - minz) * 0.95

    best_wps   = []
    best_delta = float("inf")

    for it in range(60):
        step_mid = (step_lo + step_hi) / 2.0
        wps = generate_boundary_grid(polygon, step_mid)
        if not wps:
            step_hi = step_mid
            continue
        dist  = path_length_world(wps, lake_scale)
        delta = (dist - target_dist) / target_dist
        print(f"  [{it+1:2d}] step={step_mid*lake_scale:.4f} m  "
              f"dist={dist:.1f} m  δ={delta*100:+.2f}%  n={len(wps)}")
        if abs(delta) < abs(best_delta):
            best_delta = delta
            best_wps   = wps
        if abs(delta) <= tol_frac:
            break
        if dist > target_dist:
            step_lo = step_mid
        else:
            step_hi = step_mid

    if best_wps:
        print(f"[waypoints] Wynik: dist={path_length_world(best_wps, lake_scale):.1f} m  "
              f"n={len(best_wps)}  δ={best_delta*100:+.2f}%")
    return best_wps


def interpolate_waypoints(waypoints, boat_speed: float, sonar_hz: float, lake_scale: float):
    """Wstawia punkty co boat_speed/sonar_hz metrów wzdłuż każdego odcinka trasy."""
    from shapely.geometry import Point

    interval_m = boat_speed / sonar_hz
    result = []
    for i, p in enumerate(waypoints):
        result.append(p)
        if i + 1 >= len(waypoints):
            break
        q = waypoints[i + 1]
        dx = (q.x - p.x) * lake_scale
        dy = (q.y - p.y) * lake_scale
        dist = math.sqrt(dx * dx + dy * dy)
        n_steps = int(dist / interval_m)
        for k in range(1, n_steps):
            t = k * interval_m / dist
            result.append(Point(p.x + t * (q.x - p.x), p.y + t * (q.y - p.y)))
    print(f"[waypoints] Interpolacja: {len(waypoints)} → {len(result)} wp "
          f"(co {interval_m:.3f} m)")
    return result


def waypoints_from_time(polygon, lake_scale: float, time_min: float,
                        boat_speed: float, sonar_hz: float,
                        do_interpolate: bool,
                        tol_frac: float) -> list[tuple[float, float]]:
    """Generuje waypoints przez bisection i zwraca jako listę (world_x, world_y)."""
    wps_obj = find_step_for_time(polygon, time_min * 60.0, boat_speed, lake_scale, tol_frac)
    if not wps_obj:
        return []
    if do_interpolate:
        wps_obj = interpolate_waypoints(wps_obj, boat_speed, sonar_hz, lake_scale)
    return [(lake_scale * p.x, -lake_scale * p.y) for p in wps_obj]


def extract_scale_from_filename(path: pathlib.Path) -> float | None:
    """Wyciąga skalę z nazwy pliku (*_x{N}.csv). Zwraca None jeśli nie znaleziono."""
    m = re.search(r'_x(\d+(?:\.\d+)?)\.csv$', path.name)
    return float(m.group(1)) if m else None


def find_all_waypoints_grouped(wp_dir: pathlib.Path) -> dict[float, list[pathlib.Path]]:
    """Zwraca {scale: [files]} dla wszystkich waypoints_time_*_interp_x*.csv w katalogu."""
    groups: dict[float, list[pathlib.Path]] = {}
    for f in sorted(wp_dir.glob("waypoints_time_*_interp_x*.csv")):
        scale = extract_scale_from_filename(f)
        if scale is not None:
            groups.setdefault(scale, []).append(f)
    print(f"[baseline] Znaleziono {sum(len(v) for v in groups.values())} plików "
          f"dla {len(groups)} skal w {wp_dir}: {sorted(groups)}")
    return groups


# ---------------------------------------------------------------------------
# Raycast / fizyka
# ---------------------------------------------------------------------------

def build_cone_dirs(beam_half_deg: float) -> list:
    """Zwraca listę carb.Float3 — kierunki promieni stożka (oś -Z w dół)."""
    half_rad = math.radians(beam_half_deg)
    d  = np.array([0.0, 0.0, -1.0])
    u  = np.array([1.0, 0.0,  0.0])
    v  = np.array([0.0, 1.0,  0.0])

    rays = [d.copy()]
    for n_rays, frac in [(6, 1/3), (12, 2/3), (18, 1)]:
        theta = half_rad * frac
        for i in range(n_rays):
            phi = 2.0 * math.pi * i / n_rays
            ray = (math.cos(theta) * d
                   + math.sin(theta) * (math.cos(phi) * u + math.sin(phi) * v))
            rays.append(ray / np.linalg.norm(ray))

    return [carb.Float3(float(r[0]), float(r[1]), float(r[2])) for r in rays]


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


def rescale_lake(stage, n_tiles: int, new_scale: float) -> None:
    """Aktualizuje ScaleOp na istniejących primach jeziora."""
    for i in range(n_tiles):
        prim = stage.GetPrimAtPath(f"/World/lake/tile_{i:02d}")
        _apply_transform(UsdGeom.Xformable(prim), new_scale)
    print(f"[bathymetry_isaac] Przeskalowano jezioro → ×{new_scale:g}")


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
    print(f"[bathymetry_isaac] Załadowano {len(tile_files)} kafelków kolizyjnych.")


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


# ---------------------------------------------------------------------------

def _init_world(tiles_dir: pathlib.Path, lake_scale: float):
    """Ładuje jezioro do Isaac Sim. Zwraca (world, stage, physx, cone_dirs, n_tiles). Wywołać raz."""
    tile_files = sorted(tiles_dir.glob("*.obj"))
    n_tiles    = len(tile_files)
    print(f"\n[bathymetry_isaac] Ładowanie {n_tiles} kafelków: {tiles_dir}")
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    add_lake(stage, tiles_dir, lake_scale)
    world.reset()
    world.step(render=False)
    physx     = get_physx_scene_query_interface()
    cone_dirs = build_cone_dirs(SONAR_BEAM_HALF_DEG)
    return world, stage, physx, cone_dirs, n_tiles


def _scan(waypoints: list[tuple[float, float]], physx, cone_dirs: list,
          boat_z: float) -> list[tuple]:
    """Skan raycasting. Zwraca listę (wx, wy, world_z, depth)."""
    n       = len(waypoints)
    results = []
    t0      = time.monotonic()
    misses  = 0

    print(f"[bathymetry_isaac] Startuję skan {n:,} waypointów...")
    for i, (wx, wy) in enumerate(waypoints):
        origin = carb.Float3(wx, wy, boat_z)
        d      = cone_raycast(physx, origin, cone_dirs)
        if d is None:
            misses += 1
            continue
        world_z = boat_z - d
        if world_z >= 0.0:
            misses += 1
            continue
        results.append((wx, wy, world_z, d))

        if i == 0 or (i + 1) % max(1, n // 20) == 0 or i + 1 == n:
            elapsed   = time.monotonic() - t0
            remaining = (elapsed / (i + 1)) * (n - i - 1) if i > 0 else 0
            print(f"  [{i+1:>6}/{n}] {100*(i+1)/n:5.1f}%  ETA: {remaining:.0f}s")

    elapsed = time.monotonic() - t0
    print(f"[bathymetry_isaac] Gotowe. Czas: {elapsed:.1f}s  "
          f"trafień: {len(results):,}  chybień: {misses:,} ({100*misses/max(n,1):.1f}%)")
    return results


def _save(results: list[tuple], out_dir: pathlib.Path, stem: str,
          save_csv_flag: bool) -> None:
    pts = np.array([[r[0], r[1], r[2]] for r in results], dtype=np.float32)

    pcd_path = out_dir / (stem + ".pcd")
    save_pcd(pts, pcd_path)
    print(f"[bathymetry_isaac] PCD → {pcd_path}")

    try:
        import trimesh
        ply_path = out_dir / (stem + ".ply")
        trimesh.PointCloud(pts).export(str(ply_path))
        print(f"[bathymetry_isaac] PLY → {ply_path}")
    except ImportError:
        pass

    if save_csv_flag:
        csv_path = out_dir / (stem + ".csv")
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["world_x", "world_y", "world_z", "depth"])
            writer.writeheader()
            writer.writerows([{"world_x": r[0], "world_y": r[1],
                               "world_z": r[2], "depth": r[3]} for r in results])
        print(f"[bathymetry_isaac] CSV → {csv_path}")


# ---------------------------------------------------------------------------
def _stem_from_wp_path(wp_path: pathlib.Path, lake_scale: float) -> str:
    scale_str = str(int(lake_scale)) if lake_scale == int(lake_scale) else str(lake_scale)
    info = (wp_path.stem
            .removeprefix("waypoints_time_")
            .removesuffix(f"_x{scale_str}")
            .removesuffix("_interp"))
    return f"baseline_x{lake_scale:g}_{info}"


def main(tiles_dir: pathlib.Path,
         step_m: float, n_grid: int | None, water_y: str, boat_z: float,
         mesh_reduction: float,
         out_dir: pathlib.Path, time_min: float | None, save_csv_flag: bool,
         waypoints_file: pathlib.Path | None,
         waypoints_dir: pathlib.Path | None,
         params_csv: pathlib.Path | None = None,
         boat_speed: float = 1.0,
         sonar_hz: float = 20.0,
         tol_frac: float = 0.005) -> None:

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    # ── tryb --params-csv: generuj waypoints z tabeli lake_scale+time_min ──
    if params_csv is not None:
        with open(params_csv) as f:
            rows = list(csv.DictReader(f))
        rows_sorted = sorted(rows, key=lambda r: float(r["lake_scale"]))
        print(f"[bathymetry_isaac] --params-csv: {len(rows_sorted)} skanów z {params_csv}")

        mesh = load_mesh(_DEFAULT_LAKE_OBJ)
        water_y_val = float(mesh.bounds[1][1])
        polygon = get_contour_polygon(mesh, water_y_val)

        first_scale = float(rows_sorted[0]["lake_scale"])
        world, stage, physx, cone_dirs, n_tiles = _init_world(tiles_dir, first_scale)
        prev_scale = first_scale

        for scan_idx, row in enumerate(rows_sorted):
            lake_scale  = float(row["lake_scale"])
            t_min       = float(row["time_min"])
            speed       = float(row.get("speed",       boat_speed))
            do_interp   = bool(int(row.get("interpolate", 1)))
            s_hz        = float(row.get("sonar_hz",    sonar_hz))

            print(f"\n{'='*60}")
            print(f"[bathymetry_isaac] Skan {scan_idx+1}/{len(rows_sorted)}  "
                  f"×{lake_scale:g}  time={t_min:.1f} min")
            print(f"{'='*60}")

            if lake_scale != prev_scale:
                rescale_lake(stage, n_tiles, lake_scale)
                world.reset()
                world.step(render=False)
                prev_scale = lake_scale

            waypoints = waypoints_from_time(polygon, lake_scale, t_min,
                                            speed, s_hz, do_interp, tol_frac)
            if not waypoints:
                print(f"WARN: brak waypointów dla ×{lake_scale:g} — pomijam", file=sys.stderr)
                continue

            results    = _scan(waypoints, physx, cone_dirs, boat_z)
            interp_str = "_interp" if do_interp else ""
            stem       = f"baseline_x{lake_scale:g}_{t_min:.1f}min{interp_str}_{ts}"
            _save(results, out_dir, stem, save_csv_flag)

        simulation_app.close()
        return

    # ── tryb --waypoints-dir: auto-wykryj skale z nazw plików ─────────────
    if waypoints_dir is not None:
        groups = find_all_waypoints_grouped(waypoints_dir)
        if not groups:
            print(f"BŁĄD: brak plików waypoints_time_*_interp_x*.csv w {waypoints_dir}",
                  file=sys.stderr)
            sys.exit(1)
        scales = sorted(groups.keys())
        world, stage, physx, cone_dirs, n_tiles = _init_world(tiles_dir, scales[0])
        for scale_idx, lake_scale in enumerate(scales):
            print(f"\n{'='*60}")
            print(f"[bathymetry_isaac] Skala ×{lake_scale:g}  ({scale_idx+1}/{len(scales)})")
            print(f"{'='*60}")
            if scale_idx > 0:
                rescale_lake(stage, n_tiles, lake_scale)
                world.reset()
                world.step(render=False)
            for wp_path in groups[lake_scale]:
                waypoints = load_waypoints_csv(wp_path)
                results   = _scan(waypoints, physx, cone_dirs, boat_z)
                stem      = _stem_from_wp_path(wp_path, lake_scale) + f"_{ts}"
                _save(results, out_dir, stem, save_csv_flag)
        simulation_app.close()
        return

    # ── tryb --waypoints-file: jeden plik, skala z nazwy lub --mesh-reduction
    if waypoints_file is not None:
        lake_scale = extract_scale_from_filename(waypoints_file)
        if lake_scale is None:
            lake_scale = MESH_NATURAL_REDUCTION / mesh_reduction
            print(f"[bathymetry_isaac] Skala z --mesh-reduction: ×{lake_scale:.4g}")
        else:
            print(f"[bathymetry_isaac] Skala z nazwy pliku: ×{lake_scale:g}")
        world, stage, physx, cone_dirs, n_tiles = _init_world(tiles_dir, lake_scale)
        waypoints = load_waypoints_csv(waypoints_file)
        results   = _scan(waypoints, physx, cone_dirs, boat_z)
        stem      = _stem_from_wp_path(waypoints_file, lake_scale) + f"_{ts}"
        _save(results, out_dir, stem, save_csv_flag)
        simulation_app.close()
        return

    # ── tryb legacy: generuj waypoints z parametrów ────────────────────────
    if time_min is None:
        print("BŁĄD: podaj --time, --waypoints-file lub --waypoints-dir", file=sys.stderr)
        sys.exit(1)
    lake_scale = MESH_NATURAL_REDUCTION / mesh_reduction
    print(f"[bathymetry_isaac] Tryb legacy. Skala: ×{lake_scale:.4g}")
    world, stage, physx, cone_dirs, n_tiles = _init_world(tiles_dir, lake_scale)
    waypoints = build_waypoints(step_m, n_grid, water_y, boat_z, lake_scale)
    results   = _scan(waypoints, physx, cone_dirs, boat_z)
    stem      = f"baseline_x{lake_scale:g}_wp{len(waypoints)}_{time_min:.1f}min_{ts}"
    _save(results, out_dir, stem, save_csv_flag)
    simulation_app.close()


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Batymetria: generuje waypoints, następnie skanuje raycasting w Isaac Sim."
    )

    wp = parser.add_argument_group("waypoints")
    wp.add_argument("--step",     type=float, default=2.0,
                    help="Krok siatki waypointów [m] w przestrzeni Isaac Sim (domyślnie 2.0)")
    wp.add_argument("--n-grid",   type=int, default=None,
                    help="Zamiast --step: generuj ~N×N waypointów w jeziorze")
    wp.add_argument("--water-y",  default="auto",
                    help='Poziom wody w OBJ (oś Y). "auto" = max Y mesha.')
    wp.add_argument("--boat-z",         type=float, default=0.0,
                    help="Wysokość łódki w świecie Isaac Sim [m] (domyślnie 0.0)")
    wp.add_argument("--mesh-reduction", type=float, default=10.0,
                    help="Ile razy mesh jest pomniejszony (domyślnie 10). "
                         "Skala w Isaac = 100 / mesh-reduction")
    wp.add_argument("--waypoints-file", type=pathlib.Path, default=None,
                    help="Gotowy CSV z waypointami (idx,world_x,world_y,world_z). "
                         "Zastępuje --step i --n-grid.")
    wp.add_argument("--waypoints-dir",  type=pathlib.Path,
                    default=None,
                    help=f"Katalog z plikami waypoints_time_*_x{{scale}}.csv. "
                         f"Przetwarza wszystkie pasujące pliki w jednej sesji Isaac Sim. "
                         f"Domyślnie: {_DEFAULT_WAYPOINTS_DIR}")

    sim = parser.add_argument_group("simulation")
    sim.add_argument("--tiles", default=str(_DEFAULT_TILES_DIR),
                     help="Katalog z kafelkami .obj jeziora")
    sim.add_argument("--out",   default="/workspace/log",
                     help="Katalog wyjściowy (nazwa pliku generowana automatycznie)")
    sim.add_argument("--time",  type=float, default=None,
                     help="Planowany czas trwania misji [min] — używany tylko przy generowaniu "
                          "waypointów (tryb legacy bez --waypoints-file/--waypoints-dir)")
    sim.add_argument("--csv",   action="store_true",
                     help="Zapisz wyniki do CSV oprócz PCD/PLY")
    sim.add_argument("--params-csv", type=pathlib.Path, default=None,
                     help="CSV z parametrami skanów (kolumny: lake_scale, time_min; opcjonalne: "
                          "speed, interpolate, sonar_hz). Każdy wiersz = jeden skan baseline. "
                          "Nadpisuje --waypoints-file/--waypoints-dir.")
    sim.add_argument("--speed",    type=float, default=1.0,
                     help="Prędkość łódki [m/s] używana przy --params-csv (domyślnie 1.0)")
    sim.add_argument("--sonar-hz", type=float, default=20.0,
                     help="Częstotliwość sonaru [Hz] przy --params-csv (domyślnie 20.0)")
    sim.add_argument("--step-tol", type=float, default=0.5,
                     help="Tolerancja bisection [%%] przy --params-csv (domyślnie 0.5)")

    args = parser.parse_args()

    # Jeśli --waypoints-dir nie podano explicite, użyj domyślnego katalogu gdy nie ma --waypoints-file
    wp_dir = args.waypoints_dir
    if wp_dir is None and args.waypoints_file is None and args.params_csv is None:
        wp_dir = _DEFAULT_WAYPOINTS_DIR

    main(
        tiles_dir      = pathlib.Path(args.tiles),
        step_m         = args.step,
        n_grid         = args.n_grid,
        water_y        = args.water_y,
        boat_z         = args.boat_z,
        mesh_reduction = args.mesh_reduction,
        out_dir        = pathlib.Path(args.out),
        time_min       = args.time,
        save_csv_flag  = args.csv,
        waypoints_file = args.waypoints_file,
        waypoints_dir  = wp_dir,
        params_csv     = args.params_csv,
        boat_speed     = args.speed,
        sonar_hz       = args.sonar_hz,
        tol_frac       = args.step_tol / 100.0,
    )
