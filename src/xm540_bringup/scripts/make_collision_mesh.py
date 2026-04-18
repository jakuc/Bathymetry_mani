#!/usr/bin/env python3
"""
Konwertuje big_lake_simp.obj (12M trójkątów, punkty z RGB) do regularnej
siatki wysokościowej (height grid) odpowiedniej dla PhysX collision cooking.

Wyjście: big_lake_collision.obj  — czysta siatka ~GRID×GRID trójkątów,
w układzie identycznym jak wejście (transformacja isaac_sim.py bez zmian).

Użycie:
    python3 make_collision_mesh.py [--grid N]  # domyślnie N=512
"""

import argparse
import math
import pathlib
import sys
import time

import numpy as np

SCRIPT_DIR = pathlib.Path(__file__).parent.resolve()
MESHES_DIR = SCRIPT_DIR.parent / "meshes"
INPUT_OBJ  = MESHES_DIR / "big_lake_simp.obj"
OUTPUT_OBJ = MESHES_DIR / "big_lake_collision.obj"


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--grid", type=int, default=512,
                   help="Rozdzielczość siatki N×N (domyślnie 512)")
    return p.parse_args()


def load_vertices_streaming(path: pathlib.Path) -> np.ndarray:
    """Ładuje tylko wierzchołki z OBJ (linie 'v x y z [r g b]') jako numpy array."""
    xs, ys, zs = [], [], []
    t0 = time.monotonic()
    total_bytes = path.stat().st_size
    read_bytes = 0
    last_print = 0.0

    with open(path, "r") as f:
        for line in f:
            read_bytes += len(line)
            now = time.monotonic()
            if now - last_print > 2.0:
                pct = 100.0 * read_bytes / total_bytes
                print(f"  Ładowanie: {pct:.1f}%  ({len(xs):,} wierzchołków) "
                      f"[{now - t0:.0f}s]", end="\r", flush=True)
                last_print = now

            if not line.startswith("v "):
                continue
            parts = line.split()
            try:
                xs.append(float(parts[1]))
                ys.append(float(parts[2]))
                zs.append(float(parts[3]))
            except (IndexError, ValueError):
                continue

    print()
    return np.column_stack([xs, ys, zs]).astype(np.float32)


def build_height_grid(verts: np.ndarray, grid: int) -> tuple:
    """
    Buduje siatkę N×N punktów w przestrzeni XZ oryginalnego OBJ.
    Dla każdej komórki bierze MINIMUM Y (= najgłębszy punkt – dno jeziora).

    Zwraca (x_lin, z_lin, H) gdzie H[i,j] = min Y w komórce (i,j).
    """
    x = verts[:, 0]
    y = verts[:, 1]   # Y = głębokość (po rotX=90° stanie się Z world)
    z = verts[:, 2]

    x_min, x_max = x.min(), x.max()
    z_min, z_max = z.min(), z.max()

    pad_x = (x_max - x_min) * 0.001
    pad_z = (z_max - z_min) * 0.001
    x_min -= pad_x; x_max += pad_x
    z_min -= pad_z; z_max += pad_z

    print(f"  Zakres X: [{x_min:.4f}, {x_max:.4f}]")
    print(f"  Zakres Y: [{y.min():.4f}, {y.max():.4f}]  (głębokość)")
    print(f"  Zakres Z: [{z_min:.4f}, {z_max:.4f}]")

    x_lin = np.linspace(x_min, x_max, grid)
    z_lin = np.linspace(z_min, z_max, grid)

    # Inicjalizuj H jako +inf, potem weź minimum
    H = np.full((grid, grid), np.inf, dtype=np.float32)

    # Bin indices
    ix = np.clip(((x - x_min) / (x_max - x_min) * (grid - 1)).astype(np.int32), 0, grid - 1)
    iz = np.clip(((z - z_min) / (z_max - z_min) * (grid - 1)).astype(np.int32), 0, grid - 1)

    print(f"  Wypełnianie siatki {grid}×{grid}...")
    # minimum po binach — pętla wolna, ale numpy min-accumulate nie ma wbudowanej
    # Dla dużych N×N robimy to wektorowo chunk-by-chunk
    np.minimum.at(H, (ix, iz), y)

    # Interpoluj brakujące komórki (inf) z sąsiadów
    inf_mask = np.isinf(H)
    n_inf = inf_mask.sum()
    if n_inf > 0:
        print(f"  Interpolacja {n_inf} pustych komórek...")
        # Prosta dyfuzja: zastąp inf medianą sąsiadów (iteracyjnie)
        from scipy.ndimage import generic_filter
        H_filled = H.copy()
        H_filled[inf_mask] = np.nan
        # Iteracyjne wypełnianie (max 20 przebiegów)
        try:
            from scipy.ndimage import median_filter
            for _ in range(20):
                if not np.isnan(H_filled).any():
                    break
                tmp = median_filter(np.where(np.isnan(H_filled), 0, H_filled), size=3)
                H_filled = np.where(np.isnan(H_filled), tmp, H_filled)
        except ImportError:
            # Bez scipy – zastąp inf globalnym minimum
            H_filled[np.isnan(H_filled)] = y.min()
        H = H_filled

    return x_lin, z_lin, H


def export_obj(x_lin: np.ndarray, z_lin: np.ndarray, H: np.ndarray,
               out_path: pathlib.Path) -> None:
    """Eksportuje height grid jako OBJ (układ XYZ identyczny z wejściem)."""
    grid = len(x_lin)
    n_verts = grid * grid
    n_faces = 2 * (grid - 1) * (grid - 1)
    print(f"  Eksport: {n_verts:,} wierzchołków, {n_faces:,} trójkątów → {out_path}")

    with open(out_path, "w") as f:
        f.write(f"# Height-grid collision mesh ({grid}x{grid})\n")
        f.write(f"# Generated from big_lake_simp.obj\n")
        for i in range(grid):
            for j in range(grid):
                f.write(f"v {x_lin[i]:.6f} {float(H[i, j]):.6f} {z_lin[j]:.6f}\n")

        for i in range(grid - 1):
            for j in range(grid - 1):
                v0 = i * grid + j + 1        # 1-indexed
                v1 = i * grid + j + 2
                v2 = (i + 1) * grid + j + 2
                v3 = (i + 1) * grid + j + 1
                f.write(f"f {v0} {v1} {v2}\n")
                f.write(f"f {v0} {v2} {v3}\n")

    size_mb = out_path.stat().st_size / 1_048_576
    print(f"  Zapisano: {out_path} ({size_mb:.1f} MB)")


def main():
    args = parse_args()
    grid = args.grid
    t_start = time.monotonic()

    print(f"=== make_collision_mesh.py  grid={grid}×{grid} ===")
    print(f"Wejście: {INPUT_OBJ}  ({INPUT_OBJ.stat().st_size / 1_048_576:.0f} MB)")

    print("1. Ładowanie wierzchołków...")
    verts = load_vertices_streaming(INPUT_OBJ)
    print(f"   {len(verts):,} wierzchołków załadowanych")

    print("2. Budowanie height grid...")
    x_lin, z_lin, H = build_height_grid(verts, grid)

    del verts  # zwolnij RAM

    print("3. Eksport OBJ...")
    export_obj(x_lin, z_lin, H, OUTPUT_OBJ)

    elapsed = time.monotonic() - t_start
    print(f"Gotowe w {elapsed:.1f}s")
    print(f"\nAktualizuj LAKE_OBJ_PATH w isaac_sim.py → big_lake_collision.obj")


if __name__ == "__main__":
    main()
