#!/usr/bin/env python3
"""
cloud_analysis.py — Analiza jakości pokrycia chmur punktów metodą Cloud-to-Mesh.

Użycie:
    python3 analysis/cloud_analysis.py <chmura1.csv> <chmura2.csv> [--labels A B]
                                       [--mesh path/to/mesh.obj]
                                       [--samples N] [--radius R [R ...]]

Algorytm:
  1. Wczytaj chmury punktów (CSV: world_x, world_y, world_z)
  2. Wczytaj mesh referencyjny (OBJ) i transformuj do przestrzeni świata Isaac Sim
  3. Próbkuj mesh równomiernie po powierzchni (N punktów)
  4. Dla każdej próbki znajdź odległość do najbliższego punktu w każdej chmurze (KDTree)
  5. Wykresy: histogram porównawczy, CDF, pokrycie @ różnych promieniach, tabela statystyk

Zależności:
    pip install trimesh scipy numpy matplotlib
"""

import argparse
import pathlib
import sys
from datetime import datetime

import numpy as np


MESH_DEFAULT = pathlib.Path(__file__).parent.parent / \
    "src/xm540_bringup/meshes/big_lake_simp.obj"

# Transformacja OBJ → świat Isaac Sim (musi zgadzać się z isaac_sim.py)
# world_x =  obj_x
# world_y = -obj_z
# world_z =  obj_y - 3.0   (LAKE_TRANSLATE_Z = -3.0, LAKE_SCALE = 1.0)
def obj_to_world(pts_obj: np.ndarray) -> np.ndarray:
    """pts_obj: (N,3) kolumny [x, y, z] w przestrzeni OBJ."""
    wx =  pts_obj[:, 0]
    wy = -pts_obj[:, 2]
    wz =  pts_obj[:, 1] - 3.0
    return np.column_stack([wx, wy, wz])


def load_cloud(path: pathlib.Path) -> np.ndarray:
    """Wczytuje CSV ze skanem. Zwraca (N,3) float64."""
    with open(path) as f:
        header = f.readline().strip().split(",")
    try:
        cols = (header.index("wx"), header.index("wy"), header.index("wz"))
    except ValueError:
        cols = (header.index("world_x"), header.index("world_y"), header.index("world_z"))
    data = np.loadtxt(path, delimiter=",", skiprows=1, usecols=cols)
    print(f"  {path.name}: {len(data):,} punktów")
    return data.astype(np.float64)


def sample_mesh(mesh_path: pathlib.Path, n_samples: int) -> np.ndarray:
    """Próbkuje mesh równomiernie po powierzchni. Zwraca (N,3) w przestrzeni świata."""
    try:
        import trimesh
    except ImportError:
        print("BŁĄD: brak trimesh. Zainstaluj: pip install trimesh", file=sys.stderr)
        sys.exit(1)

    print(f"Wczytywanie mesha: {mesh_path}")
    mesh = trimesh.load(str(mesh_path), force="mesh")
    print(f"  Trójkąty: {len(mesh.faces):,}  Próbki: {n_samples:,}")

    pts_obj, _ = trimesh.sample.sample_surface(mesh, n_samples)
    pts_world  = obj_to_world(pts_obj)

    # Ogranicz do obszaru powyżej dna i poniżej powierzchni wody (world_z ∈ [-50, 0])
    mask = (pts_world[:, 2] >= -50.0) & (pts_world[:, 2] <= 0.1)
    pts_world = pts_world[mask]
    print(f"  Próbek po filtrowaniu (z ∈ [-50, 0.1]): {len(pts_world):,}")
    return pts_world


def c2m_distances(cloud: np.ndarray, mesh_samples: np.ndarray) -> np.ndarray:
    """Dla każdej próbki mesha zwraca odległość do najbliższego punktu chmury."""
    from scipy.spatial import KDTree
    tree = KDTree(cloud)
    dists, _ = tree.query(mesh_samples, workers=-1)
    return dists


def _build_rows(distances, labels, radii):
    rows = [
        ("Średnia [m]",    lambda d: f"{np.mean(d):.4f}"),
        ("Mediana [m]",    lambda d: f"{np.median(d):.4f}"),
        ("P75 [m]",        lambda d: f"{np.percentile(d, 75):.4f}"),
        ("P90 [m]",        lambda d: f"{np.percentile(d, 90):.4f}"),
        ("P95 [m]",        lambda d: f"{np.percentile(d, 95):.4f}"),
        ("Std [m]",        lambda d: f"{np.std(d):.4f}"),
    ]
    for r_val in radii:
        rows.append((f"Pokrycie @ {r_val}m [%]",
                     lambda d, r=r_val: f"{np.mean(d <= r)*100:.1f}"))
    return rows


def print_stats(distances: list[np.ndarray], labels: list[str],
                radii: list[float]) -> None:
    header = f"{'Metryka':<30}" + "".join(f"{l:>14}" for l in labels)
    print("\n" + header)
    print("-" * len(header))
    for name, fn in _build_rows(distances, labels, radii):
        print(f"{name:<30}" + "".join(f"{fn(d):>14}" for d in distances))


def plot_results(distances: list[np.ndarray], labels: list[str],
                 radii: list[float], out_dir: pathlib.Path) -> None:
    import matplotlib.pyplot as plt

    colors = ["steelblue", "tomato", "seagreen", "orange"]
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    # --- Histogram ---
    ax = axes[0]
    clip = np.percentile(np.concatenate(distances), 98)
    bins = np.linspace(0, clip, 80)
    for d, label, color in zip(distances, labels, colors):
        ax.hist(np.clip(d, 0, clip), bins=bins, alpha=0.6,
                label=f"{label}  μ={np.mean(d):.3f}m  med={np.median(d):.3f}m",
                color=color, density=True)
    ax.set_xlabel("Odległość do najbliższego pomiaru [m]")
    ax.set_ylabel("Gęstość")
    ax.set_title("Histogram odległości C2M")
    ax.legend(fontsize=8)

    # --- CDF ---
    ax = axes[1]
    x_max = np.percentile(np.concatenate(distances), 99)
    xs = np.linspace(0, x_max, 500)
    for d, label, color in zip(distances, labels, colors):
        cdf = np.mean(d[:, None] <= xs, axis=0)
        ax.plot(xs, cdf * 100, label=label, color=color, linewidth=2)
    ax.set_xlabel("Promień r [m]")
    ax.set_ylabel("% powierzchni pokrytej")
    ax.set_title("CDF pokrycia")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- Pokrycie @ konkretnych promieniach ---
    ax = axes[2]
    x = np.arange(len(radii))
    width = 0.8 / len(distances)
    for i, (d, label, color) in enumerate(zip(distances, labels, colors)):
        coverage = [np.mean(d <= r) * 100 for r in radii]
        ax.bar(x + i * width, coverage, width, label=label, color=color, alpha=0.8)
    ax.set_xticks(x + width * (len(distances) - 1) / 2)
    ax.set_xticklabels([f"{r}m" for r in radii])
    ax.set_ylabel("% powierzchni pokrytej")
    ax.set_title("Pokrycie @ promieniu r")
    ax.legend()
    ax.set_ylim(0, 105)
    ax.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    fig.savefig(out_dir / "plots.png", dpi=150, bbox_inches="tight")
    plt.show()


def plot_table(distances: list[np.ndarray], labels: list[str],
               radii: list[float], out_dir: pathlib.Path) -> None:
    import matplotlib.pyplot as plt

    rows = _build_rows(distances, labels, radii)
    row_labels = [name for name, _ in rows]
    cell_data  = [[fn(d) for d in distances] for _, fn in rows]

    fig_h = max(3.0, 0.45 * len(row_labels) + 1.0)
    fig_w = max(5.0, 2.5 * len(labels) + 2.5)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))
    ax.axis("off")

    colors_header = ["#d0e4f7"] * len(labels)
    tbl = ax.table(
        cellText=cell_data,
        rowLabels=row_labels,
        colLabels=labels,
        loc="center",
        cellLoc="center",
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(10)
    tbl.scale(1.0, 1.6)

    for (r, c), cell in tbl.get_celld().items():
        if r == 0:
            cell.set_facecolor("#4a90d9")
            cell.set_text_props(color="white", fontweight="bold")
        elif c == -1:
            cell.set_facecolor("#eef2f7")
        elif r % 2 == 0:
            cell.set_facecolor("#f7f9fc")

    ax.set_title("Statystyki C2M", fontsize=12, fontweight="bold", pad=12)
    plt.tight_layout()
    fig.savefig(out_dir / "table.png", dpi=150, bbox_inches="tight")
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Analiza C2M chmur punktów")
    parser.add_argument("clouds", nargs="+", metavar="CSV",
                        help="Pliki CSV ze skanami (min. 1, max. 4)")
    parser.add_argument("--labels", nargs="+", metavar="LABEL",
                        help="Etykiety dla każdej chmury (domyślnie: nazwy plików)")
    parser.add_argument("--mesh", type=pathlib.Path, default=MESH_DEFAULT,
                        help=f"Mesh referencyjny OBJ (domyślnie: {MESH_DEFAULT})")
    parser.add_argument("--samples", type=int, default=50_000,
                        help="Liczba próbek na meshu (domyślnie: 50000)")
    parser.add_argument("--radius", type=float, nargs="+",
                        default=[0.1, 0.25, 0.5, 1.0, 3.0, 5.0, 10.0],
                        help="Progi pokrycia [m] (domyślnie: 0.1 0.25 0.5 1.0)")
    args = parser.parse_args()

    cloud_paths = [pathlib.Path(p) for p in args.clouds]
    labels = args.labels or [p.stem for p in cloud_paths]

    if len(cloud_paths) != len(labels):
        parser.error("Liczba --labels musi zgadzać się z liczbą plików CSV")

    ts      = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = pathlib.Path(__file__).parent / f"results_{ts}"
    out_dir.mkdir()
    print(f"Wyniki zapisywane do: {out_dir}")

    with open(out_dir / "sources.txt", "w") as f:
        for label, path in zip(labels, cloud_paths):
            f.write(f"{label}: {path.resolve()}\n")
        f.write(f"mesh: {args.mesh.resolve()}\n")
        f.write(f"samples: {args.samples}\n")
        f.write(f"radii: {args.radius}\n")

    print("Wczytywanie chmur punktów:")
    clouds = [load_cloud(p) for p in cloud_paths]

    mesh_samples = sample_mesh(args.mesh, args.samples)

    print("\nObliczanie odległości C2M...")
    distances = []
    for cloud, label in zip(clouds, labels):
        d = c2m_distances(cloud, mesh_samples)
        distances.append(d)
        print(f"  {label}: gotowe")

    print_stats(distances, labels, args.radius)
    plot_results(distances, labels, args.radius, out_dir)
    plot_table(distances, labels, args.radius, out_dir)

    print(f"\nZapisano: {out_dir}/plots.png, table.png, sources.txt")


if __name__ == "__main__":
    main()
