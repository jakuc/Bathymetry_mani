#!/usr/bin/env python3
"""
generate_waypoints.py — Generator siatki waypointów wewnątrz jeziora.

Użycie:
    python3 generate_waypoints.py [--step 2.0] [--water-y auto] [--output waypoints.csv]

Argumenty:
    --step      Krok siatki w metrach świata Isaac Sim (domyślnie 2.0)
    --water-y   Poziom wody w przestrzeni OBJ (oś Y). Domyślnie "auto" = max Y mesha.
    --output    Plik wyjściowy (domyślnie waypoints.csv obok skryptu)
    --boat-z    Wysokość łódki w świecie Isaac Sim (domyślnie 0.0)
    --preview   Pokaż wykres konturu i waypointów (wymaga matplotlib)

Algorytm:
  1. Wczytuje big_lake_simp.obj przez trimesh
  2. Przekrój mesha płaszczyzną Y = water_y (OBJ) → odcinki w przestrzeni XZ
  3. shapely polygonize → kontur jeziora w przestrzeni OBJ (XZ)
  4. Regularna siatka punktów wewnątrz konturu, krok = step_m / LAKE_SCALE
  5. Sortowanie w serpentynę wzdłuż X
  6. Transformacja OBJ(XZ) → Isaac Sim world(XY):
       world_x = LAKE_SCALE * obj_x
       world_y = -LAKE_SCALE * obj_z
  7. Zapis do CSV: idx, world_x, world_y, world_z

Zależności:
    pip install trimesh shapely
    pip install matplotlib  (opcjonalnie, do --preview)
"""

import argparse
import math
import pathlib
import sys

import numpy as np

SCRIPT_DIR = pathlib.Path(__file__).parent.resolve()
MESHES_DIR = SCRIPT_DIR.parent / "meshes"
LAKE_OBJ   = MESHES_DIR / "big_lake_simp.obj"

# Transformacja matching isaac_sim.py (LAKE_TILES_ROTATE_X = 90°, LAKE_SCALE = 1, LAKE_TRANSLATE_Z = -30)
LAKE_SCALE       = 1.0
LAKE_TRANSLATE_Z = -3.0    # world_z = LAKE_SCALE * obj_y + LAKE_TRANSLATE_Z


def parse_args():
    p = argparse.ArgumentParser(description="Generator waypointów wewnątrz jeziora")
    p.add_argument("--step",    type=float, default=2.0,
                   help="Krok siatki [m] w przestrzeni świata Isaac Sim (domyślnie 2.0)")
    p.add_argument("--n-grid",  type=int, default=None,
                   help="Zamiast --step: generuj N×N punktów w jeziorze (np. --n-grid 4)")
    p.add_argument("--water-y", type=str, default="auto",
                   help='Poziom wody w OBJ (oś Y). "auto" = max Y mesha. '
                        'Dla world_z=0: obj_y = (0 - LAKE_TRANSLATE_Z) / LAKE_SCALE = 3.0')
    p.add_argument("--output",  type=pathlib.Path,
                   default=None,
                   help="Plik wyjściowy CSV (domyślnie: waypoints.csv / waypoints_sweep.csv / waypoints_time.csv)")
    p.add_argument("--sweep", action="store_true",
                   help="Generuje waypoints dla scenariusza sweep (domyślny output: waypoints_sweep.csv)")
    p.add_argument("--boat-z",  type=float, default=0.0,
                   help="Wysokość łódki w świecie Isaac Sim [m] (domyślnie 0.0)")
    p.add_argument("--margin", type=float, default=0.0,
                   help="Margines od brzegu jeziora [m] (domyślnie 0.0). "
                        "Przy --all-points warto ustawić >0 by uniknąć płytkich waypointów.")
    p.add_argument("--time",  type=float, default=None,
                   help="Docelowy czas trwania misji [min]. Zastępuje --step i --n-grid; "
                        "wymaga --speed.")
    p.add_argument("--speed", type=float, default=None,
                   help="Prędkość łódki [m/s]. Wymagane gdy podano --time.")
    p.add_argument("--all-points", action="store_true",
                   help="Gęsta siatka wewnątrz jeziora (domyślnie: przecięcia kolumn z obrysem).")
    p.add_argument("--step-tol", type=float, default=0.5,
                   help="Tolerancja bisection [%%] (domyślnie 0.5)")
    return p.parse_args()


def load_mesh(path: pathlib.Path):
    try:
        import trimesh
    except ImportError:
        print("BŁĄD: brak trimesh. Zainstaluj: pip install trimesh", file=sys.stderr)
        sys.exit(1)

    print(f"Wczytywanie mesha: {path}")
    mesh = trimesh.load(str(path), force="mesh")
    print(f"  Wierzchołki: {len(mesh.vertices):,}  Trójkąty: {len(mesh.faces):,}")
    bounds = mesh.bounds
    print(f"  Zakres X: [{bounds[0][0]:.3f}, {bounds[1][0]:.3f}]")
    print(f"  Zakres Y: [{bounds[0][1]:.3f}, {bounds[1][1]:.3f}]")
    print(f"  Zakres Z: [{bounds[0][2]:.3f}, {bounds[1][2]:.3f}]")
    return mesh


def get_contour_polygon(mesh, water_y: float, grid_size: int = 1024):
    """Kontur jeziora z rzutu wierzchołków poniżej water_y na płaszczyznę XZ.

    Podejście rastrowe — unika problemów z krawędziami siatki mesha:
      1. Filtruj wierzchołki Y < water_y → chmura punktów XZ
      2. Rasteryzuj na siatkę grid_size × grid_size
      3. binary_closing + binary_fill_holes (scipy) → wypełniony obszar jeziora
      4. matplotlib contour na poziomie 0.5 → kontur w pikselach
      5. Przekształć piksele → OBJ XZ
    """
    from scipy.ndimage import binary_closing, binary_fill_holes

    print(f"\nKontur jeziora (raster Y < {water_y:.4f}, siatka {grid_size}×{grid_size})")

    verts = mesh.vertices[mesh.vertices[:, 1] < water_y]
    if len(verts) == 0:
        print("BŁĄD: brak wierzchołków poniżej water_y.", file=sys.stderr)
        sys.exit(1)
    print(f"  Wierzchołków poniżej poziomu wody: {len(verts):,}")

    xs, zs = verts[:, 0], verts[:, 2]
    x_min, x_max = xs.min(), xs.max()
    z_min, z_max = zs.min(), zs.max()

    # Rasteryzacja XZ → binary grid [z_idx, x_idx]
    xi = np.clip(((xs - x_min) / (x_max - x_min) * (grid_size - 1)).astype(int), 0, grid_size - 1)
    zi = np.clip(((zs - z_min) / (z_max - z_min) * (grid_size - 1)).astype(int), 0, grid_size - 1)
    grid = np.zeros((grid_size, grid_size), dtype=bool)
    grid[zi, xi] = True

    # Wypełnij luki morfologicznie i zamknij wnętrze jeziora
    closing_px = max(2, grid_size // 100)
    grid = binary_closing(grid, iterations=closing_px)
    grid = binary_fill_holes(grid)
    print(f"  Wypełniony obszar: {grid.sum():,} px² / {grid_size**2:,} px²  "
          f"({100*grid.sum()/grid_size**2:.1f}%)")

    # Kontur jako ścieżka matplotlib (nie wymaga skimage/rtree)
    import matplotlib
    matplotlib.use("Agg")   # bez wyświetlania — tylko dane
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots()
    cs = ax.contour(grid.astype(float), levels=[0.5])
    paths = cs.collections[0].get_paths()
    plt.close(fig)

    if not paths:
        print("BŁĄD: contour nie zwrócił żadnej ścieżki.", file=sys.stderr)
        sys.exit(1)

    # Wybierz najdłuższą ścieżkę (główny kontur jeziora)
    main_path = max(paths, key=lambda p: len(p.vertices))
    pix = main_path.vertices   # (N, 2): kolumna x_pix, wiersz z_pix

    # Piksele → OBJ XZ
    obj_x = pix[:, 0] / (grid_size - 1) * (x_max - x_min) + x_min
    obj_z = pix[:, 1] / (grid_size - 1) * (z_max - z_min) + z_min

    from shapely.geometry import Polygon as ShapelyPolygon
    lake = ShapelyPolygon(zip(obj_x, obj_z))
    print(f"  Powierzchnia konturu (OBJ): {lake.area:.2f}")

    return lake


def generate_grid(polygon, step_obj: float, turns_only: bool = False):
    """Siatka punktów wewnątrz konturu, serpentyna wzdłuż X.

    step_obj:   krok w przestrzeni OBJ (= step_m / LAKE_SCALE)
    turns_only: zachowaj tylko punkty wejścia/wyjścia z jeziora na każdej kolumnie
    Zwraca listę (obj_x, obj_z).
    """
    from shapely.geometry import Point

    minx, minz, maxx, maxz = polygon.bounds

    xs = np.arange(minx + step_obj / 2, maxx, step_obj)
    zs = np.arange(minz + step_obj / 2, maxz, step_obj)

    print(f"\nSiatka w przestrzeni OBJ: {len(xs)} × {len(zs)} = {len(xs)*len(zs):,} kandydatów")

    waypoints = []
    for col_idx, x in enumerate(xs):
        if turns_only:
            z_in = [i for i, z in enumerate(zs) if polygon.contains(Point(x, z))]
            if not z_in:
                continue
            ep = []
            run_start = z_in[0]
            prev = z_in[0]
            for idx in z_in[1:]:
                if idx != prev + 1:
                    ep.append(run_start)
                    if run_start != prev:
                        ep.append(prev)
                    run_start = idx
                prev = idx
            ep.append(run_start)
            if run_start != prev:
                ep.append(prev)
            col = [Point(x, zs[i]) for i in ep]
        else:
            col = [Point(x, z) for z in zs if polygon.contains(Point(x, z))]

        if col_idx % 2 == 1:
            col = col[::-1]   # serpentyna — co druga kolumna odwrócona
        waypoints.extend(col)

    print(f"Waypointów wewnątrz konturu: {len(waypoints):,}")
    return waypoints


def generate_boundary_grid(polygon, step_obj: float):
    """Waypoints jako przecięcia kolumn z obrysem jeziora.

    Każda kolumna (stałe X) daje punkty dokładnie na granicy konturu.
    Ścieżka: serpentyna po kolumnach, przejścia między kolumnami po prostej.
    """
    from shapely.geometry import LineString, Point

    minx, minz, maxx, maxz = polygon.bounds
    xs = np.arange(minx + step_obj / 2, maxx, step_obj)

    print(f"\nSiatka w przestrzeni OBJ: {len(xs)} kolumn")

    waypoints = []
    for col_idx, x in enumerate(xs):
        col_line = LineString([(x, minz - 1.0), (x, maxz + 1.0)])
        isect = polygon.intersection(col_line)

        if isect.is_empty or isect.geom_type == 'Point':
            continue

        if isect.geom_type == 'LineString':
            segs = [isect]
        elif isect.geom_type == 'MultiLineString':
            segs = sorted(isect.geoms, key=lambda s: min(c[1] for c in s.coords))
        else:
            continue

        seg_endpoints = []
        for seg in segs:
            coords = sorted(seg.coords, key=lambda c: c[1])
            seg_endpoints.append((Point(coords[0]), Point(coords[-1])))

        if col_idx % 2 == 1:
            seg_endpoints = [(e, s) for s, e in reversed(seg_endpoints)]

        for start, end in seg_endpoints:
            waypoints.append(start)
            waypoints.append(end)

    print(f"Waypointów: {len(waypoints):,}")
    return waypoints


def path_length_world(waypoints) -> float:
    """Dokładna długość trasy w metrach świata (suma odcinków między kolejnymi wp)."""
    total = 0.0
    for i in range(len(waypoints) - 1):
        dx = (waypoints[i + 1].x - waypoints[i].x) * LAKE_SCALE
        dz = (waypoints[i + 1].y - waypoints[i].y) * LAKE_SCALE
        total += math.sqrt(dx * dx + dz * dz)
    return total


def find_step_for_time(polygon, target_time_s: float, boat_speed: float,
                       tol_frac: float = 0.005,
                       all_points: bool = False) -> tuple[float, list]:
    """Bisection: szuka step_obj dającego target_dist = target_time_s * boat_speed."""
    target_dist = target_time_s * boat_speed
    print(f"\nBisection: cel={target_dist:.1f} m  ({target_time_s/60:.1f} min × {boat_speed} m/s)")
    print(f"Tolerancja: {tol_frac*100:.2f}%")

    grid_fn = generate_grid if all_points else generate_boundary_grid

    minx, minz, maxx, maxz = polygon.bounds
    step_lo = 0.05 / LAKE_SCALE                           # ~5 cm — dolna granica
    step_hi = min(maxx - minx, maxz - minz) * 0.95       # prawie cała oś — górna granica

    best_wps   = []
    best_step  = step_lo
    best_delta = float("inf")

    for it in range(60):
        step_mid = (step_lo + step_hi) / 2.0
        wps      = grid_fn(polygon, step_mid)
        if not wps:
            step_hi = step_mid
            continue

        dist  = path_length_world(wps)
        delta = (dist - target_dist) / target_dist
        print(f"  [{it+1:2d}] step={step_mid*LAKE_SCALE:.4f} m  "
              f"dist={dist:.1f} m  δ={delta*100:+.2f}%  n={len(wps)}")

        if abs(delta) < abs(best_delta):
            best_delta = delta
            best_step  = step_mid
            best_wps   = wps

        if abs(delta) <= tol_frac:
            break

        if dist > target_dist:
            step_lo = step_mid
        else:
            step_hi = step_mid

    print(f"\nWynik: step={best_step*LAKE_SCALE:.4f} m  "
          f"dist={path_length_world(best_wps):.1f} m  "
          f"δ={best_delta*100:+.2f}%  n={len(best_wps)}")
    return best_step, best_wps


def obj_to_world(obj_x: float, obj_z: float, boat_z: float):
    """Transformacja OBJ(x, z) → Isaac Sim world(x, y, z)."""
    world_x = LAKE_SCALE * obj_x
    world_y = -LAKE_SCALE * obj_z   # RotateX 90°: Z_obj → -Y_world
    world_z = boat_z
    return world_x, world_y, world_z


def save_csv(waypoints, boat_z: float, output: pathlib.Path):
    """Zapisuje waypoints do CSV."""
    with open(output, "w") as f:
        f.write("idx,world_x,world_y,world_z\n")
        for i, p in enumerate(waypoints):
            wx, wy, wz = obj_to_world(p.x, p.y, boat_z)
            f.write(f"{i},{wx:.4f},{wy:.4f},{wz:.4f}\n")
    print(f"\nZapisano {len(waypoints)} waypointów → {output}")


def save_preview(polygon, waypoints, output_csv: pathlib.Path):
    """Zapisuje widok z góry (PNG) obok pliku CSV."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("WARN: brak matplotlib, pomijam zapis PNG", file=sys.stderr)
        return

    from shapely.geometry import MultiPolygon

    fig, ax = plt.subplots(figsize=(10, 8))

    geoms = list(polygon.geoms) if isinstance(polygon, MultiPolygon) else [polygon]
    for geom in geoms:
        xs, zs = geom.exterior.xy
        ax.fill(xs, zs, alpha=0.2, color="steelblue")
        ax.plot(xs, zs, color="steelblue", linewidth=1)

    if waypoints:
        wx = [p.x for p in waypoints]
        wz = [p.y for p in waypoints]
        ax.plot(wx, wz, color="orange", linewidth=0.8, alpha=0.8, zorder=4, label="trasa")
        ax.scatter(wx, wz, s=10, color="red", zorder=5, label=f"waypoints (n={len(waypoints)})")
        ax.scatter([wx[0]], [wz[0]], s=60, color="green",  zorder=6, label="start")
        ax.scatter([wx[-1]], [wz[-1]], s=60, color="black", zorder=6, label="koniec")

    ax.set_xlabel("OBJ X [m]")
    ax.set_ylabel("OBJ Z [m]")
    ax.set_title(output_csv.stem)
    ax.set_aspect("equal")
    ax.legend(fontsize=8)
    plt.tight_layout()

    png_path = output_csv.with_suffix(".png")
    fig.savefig(png_path, dpi=150)
    plt.close(fig)
    print(f"Podgląd zapisany → {png_path}")


def main():
    args = parse_args()

    if args.output is None:
        if args.time is not None:
            stem = f"waypoints_time_{args.time}min_{args.speed}mps"
        elif args.n_grid is not None:
            base = "waypoints_sweep" if args.sweep else "waypoints"
            stem = f"{base}_ngrid_{args.n_grid}"
        else:
            base = "waypoints_sweep" if args.sweep else "waypoints"
            stem = f"{base}_step_{args.step}"
        args.output = SCRIPT_DIR.parent / f"{stem}.csv"

    mode_count = sum([args.time is not None, args.n_grid is not None, args.step != 2.0])
    if mode_count > 1:
        active = []
        if args.time    is not None: active.append("--time")
        if args.n_grid  is not None: active.append("--n-grid")
        if args.step    != 2.0:      active.append("--step")
        print(f"WARN: podano kolidujące tryby {active} — używam pierwszego z listy "
              f"(--time > --n-grid > --step)", file=sys.stderr)

    mesh = load_mesh(LAKE_OBJ)

    # Poziom wody
    if args.water_y == "auto":
        water_y = -LAKE_TRANSLATE_Z / LAKE_SCALE   # world_z=0 → obj_y
        print(f"\nPoziom wody (auto): Y = {water_y:.4f}")
    else:
        try:
            water_y = float(args.water_y)
        except ValueError:
            print(f"BŁĄD: --water-y musi być liczbą lub 'auto'", file=sys.stderr)
            sys.exit(1)

    polygon = get_contour_polygon(mesh, water_y)

    if args.margin > 0.0:
        margin_obj = args.margin / LAKE_SCALE
        polygon = polygon.buffer(-margin_obj)
        if polygon.is_empty:
            print("BŁĄD: po erozji kontur jest pusty — zmniejsz --margin", file=sys.stderr)
            sys.exit(1)
        print(f"Erozja brzegu: {args.margin} m (OBJ: {margin_obj:.4f})")

    # Krok siatki w przestrzeni OBJ
    if args.time is not None:
        if args.speed is None:
            print("BŁĄD: --time wymaga --speed", file=sys.stderr)
            sys.exit(1)
        tol_frac = args.step_tol / 100.0
        _, waypoints = find_step_for_time(polygon, args.time * 60.0, args.speed, tol_frac,
                                          all_points=args.all_points)
    elif args.n_grid is not None:
        minx, minz, maxx, maxz = polygon.bounds
        step_m   = min((maxx - minx), (maxz - minz)) * LAKE_SCALE / args.n_grid
        step_obj = step_m / LAKE_SCALE
        print(f"Tryb --n-grid {args.n_grid}: krok {step_m:.2f} m = {step_obj:.4f} OBJ")
        grid_fn  = generate_grid if args.all_points else generate_boundary_grid
        waypoints = grid_fn(polygon, step_obj)
    else:
        step_obj = args.step / LAKE_SCALE
        print(f"Krok siatki: {args.step} m (Isaac Sim) = {step_obj:.4f} (OBJ)")
        grid_fn  = generate_grid if args.all_points else generate_boundary_grid
        waypoints = grid_fn(polygon, step_obj)

    if not waypoints:
        print("BŁĄD: brak waypointów — sprawdź --water-y i --step", file=sys.stderr)
        sys.exit(1)

    # Podsumowanie pokrycia
    area_world = polygon.area * (LAKE_SCALE ** 2)
    print(f"Powierzchnia jeziora: ~{area_world:.0f} m²  (~{area_world/1e6:.3f} km²)")
    print(f"Gęstość: 1 waypoint / {area_world/len(waypoints):.1f} m²")

    save_csv(waypoints, args.boat_z, args.output)
    save_preview(polygon, waypoints, args.output)


if __name__ == "__main__":
    main()
