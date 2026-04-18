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
import pathlib
import sys

import numpy as np

SCRIPT_DIR = pathlib.Path(__file__).parent.resolve()
MESHES_DIR = SCRIPT_DIR.parent / "meshes"
LAKE_OBJ   = MESHES_DIR / "big_lake_simp.obj"

# Transformacja matching isaac_sim.py (LAKE_TILES_ROTATE_X = 90°, LAKE_SCALE = 10, LAKE_TRANSLATE_Z = -30)
LAKE_SCALE       = 10.0
LAKE_TRANSLATE_Z = -30.0   # world_z = LAKE_SCALE * obj_y + LAKE_TRANSLATE_Z


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
                   default=SCRIPT_DIR.parent / "waypoints.csv",
                   help="Plik wyjściowy CSV (domyślnie ../waypoints.csv)")
    p.add_argument("--boat-z",  type=float, default=0.0,
                   help="Wysokość łódki w świecie Isaac Sim [m] (domyślnie 0.0)")
    p.add_argument("--preview", action="store_true",
                   help="Pokaż wykres konturu i waypointów (wymaga matplotlib)")
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


def generate_grid(polygon, step_obj: float):
    """Siatka punktów wewnątrz konturu, serpentyna wzdłuż X.

    step_obj: krok w przestrzeni OBJ (= step_m / LAKE_SCALE)
    Zwraca listę (obj_x, obj_z).
    """
    from shapely.geometry import Point

    minx, minz, maxx, maxz = polygon.bounds

    xs = np.arange(minx + step_obj / 2, maxx, step_obj)
    zs = np.arange(minz + step_obj / 2, maxz, step_obj)

    print(f"\nSiatka w przestrzeni OBJ: {len(xs)} × {len(zs)} = {len(xs)*len(zs):,} kandydatów")

    waypoints = []
    for col_idx, x in enumerate(xs):
        col = [Point(x, z) for z in zs if polygon.contains(Point(x, z))]
        if col_idx % 2 == 1:
            col = col[::-1]   # serpentyna — co druga kolumna odwrócona
        waypoints.extend(col)

    print(f"Waypointów wewnątrz konturu: {len(waypoints):,}")
    return waypoints


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


def preview(polygon, waypoints, step_obj: float):
    """Wykres konturu jeziora i waypointów."""
    try:
        import matplotlib
        matplotlib.use("TkAgg")   # przełącz z Agg (użytego do konturu) na interaktywny
        import matplotlib.pyplot as plt
    except ImportError:
        print("WARN: brak matplotlib, pomijam --preview", file=sys.stderr)
        return

    fig, ax = plt.subplots(figsize=(10, 8))

    # Kontur jeziora
    from shapely.geometry import MultiPolygon
    if isinstance(polygon, MultiPolygon):
        for geom in polygon.geoms:
            xs, zs = geom.exterior.xy
            ax.fill(xs, zs, alpha=0.2, color="steelblue")
            ax.plot(xs, zs, color="steelblue", linewidth=1)
    else:
        xs, zs = polygon.exterior.xy
        ax.fill(xs, zs, alpha=0.2, color="steelblue")
        ax.plot(xs, zs, color="steelblue", linewidth=1)

    # Waypoints
    if waypoints:
        wx = [p.x for p in waypoints]
        wz = [p.y for p in waypoints]
        ax.scatter(wx, wz, s=2, color="red", zorder=5, label=f"waypoints (n={len(waypoints)})")
        # Trasa serpentynowa — pierwsze 200 punktów
        n_show = min(200, len(waypoints))
        ax.plot([p.x for p in waypoints[:n_show]],
                [p.y for p in waypoints[:n_show]],
                color="orange", linewidth=0.5, alpha=0.7, label=f"trasa (pierwsze {n_show})")

    ax.set_xlabel("OBJ X")
    ax.set_ylabel("OBJ Z")
    ax.set_title(f"Kontur jeziora + waypoints (krok OBJ = {step_obj:.3f})")
    ax.set_aspect("equal")
    ax.legend()
    plt.tight_layout()
    plt.show()


def main():
    args = parse_args()

    mesh = load_mesh(LAKE_OBJ)

    # Poziom wody
    if args.water_y == "auto":
        water_y = float(mesh.bounds[1][1])   # max Y
        print(f"\nPoziom wody (auto): Y = {water_y:.4f}")
    else:
        try:
            water_y = float(args.water_y)
        except ValueError:
            print(f"BŁĄD: --water-y musi być liczbą lub 'auto'", file=sys.stderr)
            sys.exit(1)

    polygon = get_contour_polygon(mesh, water_y)

    # Krok siatki w przestrzeni OBJ
    if args.n_grid is not None:
        # --n-grid N: krok dobrany tak by uzyskać ~N×N punktów
        minx, minz, maxx, maxz = polygon.bounds
        step_m   = min((maxx - minx), (maxz - minz)) * LAKE_SCALE / args.n_grid
        step_obj = step_m / LAKE_SCALE
        print(f"Tryb --n-grid {args.n_grid}: krok {step_m:.2f} m = {step_obj:.4f} OBJ")
    else:
        step_obj = args.step / LAKE_SCALE
        print(f"Krok siatki: {args.step} m (Isaac Sim) = {step_obj:.4f} (OBJ)")

    waypoints = generate_grid(polygon, step_obj)

    if not waypoints:
        print("BŁĄD: brak waypointów — sprawdź --water-y i --step", file=sys.stderr)
        sys.exit(1)

    # Podsumowanie pokrycia
    area_world = polygon.area * (LAKE_SCALE ** 2)
    print(f"Powierzchnia jeziora: ~{area_world:.0f} m²  (~{area_world/1e6:.3f} km²)")
    print(f"Gęstość: 1 waypoint / {area_world/len(waypoints):.1f} m²")

    save_csv(waypoints, args.boat_z, args.output)

    if args.preview:
        preview(polygon, waypoints, step_obj)


if __name__ == "__main__":
    main()
