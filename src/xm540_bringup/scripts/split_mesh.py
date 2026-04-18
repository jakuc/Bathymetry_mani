#!/usr/bin/env python3
"""
Dzieli plik OBJ na N×N kafelków przestrzennych (tile'ów).
Każdy kafelek to osobny OBJ z własną indeksacją wierzchołków — gotowy
do załadowania przez PhysX collision cooking bez problemów z rozmiarem.

Użycie:
    python3 split_mesh.py [--input plik.obj] [--tiles N]

Domyślnie: --input big_lake_simp.obj --tiles 4 → 16 plików

Wyjście (nazwa bazowana na pliku wejściowym):
    <stem>_tile_0_0.obj
    <stem>_tile_0_1.obj
    ...
    <stem>_tile_N-1_N-1.obj

─── Jak działa ──────────────────────────────────────────────────────────────

Przejście 1/2 — wczytywanie wierzchołków:
  Czyta plik linia po linii, wyciąga tylko linie "v x y z [r g b]".
  Ignoruje UV (vt) i normalne (vn) — nie są potrzebne do kolizji.
  Wynik: tablica numpy (N_verts × 3) float32 w RAM.
  Potencjalny problem: jeśli plik ma niestandardowy format wierzchołków
  (np. dodatkowe pola przed x/y/z), współrzędne będą błędne — sprawdź
  pierwsze linie "v" ręcznie: head -5 plik.obj

Przejście 2/2 — podział face'ów:
  Czyta linie "f v0[/vt/vn] v1[/vt/vn] v2[/vt/vn]".
  Obsługuje formaty: "f 1 2 3", "f 1/1 2/2 3/3", "f 1/1/1 2/2/2 3/3/3".
  Liczy centroid każdego trójkąta (średnia XZ trzech wierzchołków) i
  przypisuje go do kafelka na siatce N×N rozpiętej na zakresie XZ mesha.
  Potencjalny problem: face'y z wierzchołkami w różnych kafelkach trafiają
  do kafelka swojego centroidu — krawędzie między kafelkami mogą mieć
  szczeliny (brak duplikowania wierzchołków brzegowych). Dla kolizji
  fizycznej to zazwyczaj bez znaczenia.
  Potencjalny problem: linie "f" z więcej niż 3 wierzchołkami (quady,
  n-gony) są pomijane — skrypt obsługuje tylko trójkąty. Jeśli face'ów
  jest znacznie mniej niż oczekiwano, plik może mieć quady.

Zapis kafelków:
  Dla każdego kafelka zbiera unikalne wierzchołki użyte przez jego face'y
  i zapisuje OBJ z lokalną indeksacją (od 1). UV i normalne są pomijane —
  wyjście to czysty mesh geometryczny bez materiałów.
  Kafelki bez żadnych face'ów są pomijane (nie powstaje pusty plik).
"""

import argparse
import pathlib
import time
import numpy as np

SCRIPT_DIR = pathlib.Path(__file__).parent.resolve()
MESHES_DIR = SCRIPT_DIR.parent / "meshes"
INPUT_OBJ  = MESHES_DIR / "big_lake_simp.obj"


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--tiles", type=int, default=4,
                   help="Liczba kafelków na oś (domyślnie 4 → 4×4=16 plików)")
    p.add_argument("--input", type=pathlib.Path, default=INPUT_OBJ,
                   help=f"Ścieżka do pliku OBJ (domyślnie {INPUT_OBJ})")
    return p.parse_args()


def pass1_vertices(path: pathlib.Path):
    """Pierwsze przejście: wczytaj wszystkie wierzchołki strumieniowo."""
    print("Przejście 1/2: wczytywanie wierzchołków...")
    xs, ys, zs = [], [], []
    total = path.stat().st_size
    read  = 0
    t0    = time.monotonic()
    last  = 0.0

    with open(path, "r") as f:
        for line in f:
            read += len(line)
            now = time.monotonic()
            if now - last > 2.0:
                print(f"  {100*read/total:.1f}%  ({len(xs):,} wierzchołków) [{now-t0:.0f}s]",
                      end="\r", flush=True)
                last = now
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
    verts = np.column_stack([xs, ys, zs]).astype(np.float32)
    print(f"  Załadowano {len(verts):,} wierzchołków")
    return verts


def pass2_split(path: pathlib.Path, verts: np.ndarray, tiles: int, meshes_dir: pathlib.Path):
    """Drugie przejście: przypisz face'y do kafelków i zapisz osobne OBJ."""
    x = verts[:, 0]
    z = verts[:, 2]
    x_min, x_max = x.min(), x.max()
    z_min, z_max = z.min(), z.max()

    print(f"\nZakres X: [{x_min:.3f}, {x_max:.3f}]")
    print(f"Zakres Z: [{z_min:.3f}, {z_max:.3f}]")
    print(f"Podział: {tiles}×{tiles} = {tiles*tiles} kafelków\n")

    # Dla każdego kafelka: lista trójkątów (i0, i1, i2) — indeksy globalne
    tile_faces = [[[] for _ in range(tiles)] for _ in range(tiles)]

    total = path.stat().st_size
    read  = 0
    t0    = time.monotonic()
    last  = 0.0
    n_faces = 0

    print("Przejście 2/2: przypisywanie face'ów do kafelków...")
    with open(path, "r") as f:
        for line in f:
            read += len(line)
            now = time.monotonic()
            if now - last > 2.0:
                print(f"  {100*read/total:.1f}%  ({n_faces:,} face'ów) [{now-t0:.0f}s]",
                      end="\r", flush=True)
                last = now

            if not line.startswith("f "):
                continue

            parts = line.split()
            try:
                # Format: v/vt/vn lub v/vt lub v — bierzemy tylko indeks wierzchołka
                i0 = int(parts[1].split("/")[0]) - 1
                i1 = int(parts[2].split("/")[0]) - 1
                i2 = int(parts[3].split("/")[0]) - 1
            except (IndexError, ValueError):
                continue

            # Centroid face'a — wyznacza kafelek
            cx = (verts[i0, 0] + verts[i1, 0] + verts[i2, 0]) / 3.0
            cz = (verts[i0, 2] + verts[i1, 2] + verts[i2, 2]) / 3.0

            tx = int((cx - x_min) / (x_max - x_min) * tiles)
            tz = int((cz - z_min) / (z_max - z_min) * tiles)
            tx = min(tx, tiles - 1)
            tz = min(tz, tiles - 1)

            # Bounding box kafelka z małym marginesem — trójkąty graniczne
            # wchodzą do obu sąsiednich kafelków, zdegenerowane face'y
            # z odległymi wierzchołkami są odrzucane
            margin = (x_max - x_min) / tiles * 0.01
            tile_x0 = x_min + tx       * (x_max - x_min) / tiles
            tile_x1 = x_min + (tx + 1) * (x_max - x_min) / tiles
            tile_z0 = z_min + tz       * (z_max - z_min) / tiles
            tile_z1 = z_min + (tz + 1) * (z_max - z_min) / tiles

            for vi in (i0, i1, i2):
                vx, vz = verts[vi, 0], verts[vi, 2]
                if not (tile_x0 - margin <= vx <= tile_x1 + margin and
                        tile_z0 - margin <= vz <= tile_z1 + margin):
                    break
            else:
                tile_faces[tx][tz].append((i0, i1, i2))
                n_faces += 1

    print(f"\n  Łącznie {n_faces:,} face'ów przypisanych")

    # Zapisz każdy kafelek jako osobny OBJ
    print("\nZapis kafelków...")
    written = 0
    for tx in range(tiles):
        for tz in range(tiles):
            faces = tile_faces[tx][tz]
            if not faces:
                continue

            stem = pathlib.Path(path).stem
            out_path = meshes_dir / f"{stem}_tile_{tx}_{tz}.obj"

            # Zbierz unikalne indeksy wierzchołków użytych w tym kafelku
            used = sorted(set(i for tri in faces for i in tri))
            remap = {old: new for new, old in enumerate(used)}

            with open(out_path, "w") as f:
                f.write(f"# big_lake tile {tx},{tz}  ({len(faces):,} trójkątów)\n")
                for vi in used:
                    v = verts[vi]
                    f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
                for i0, i1, i2 in faces:
                    f.write(f"f {remap[i0]+1} {remap[i1]+1} {remap[i2]+1}\n")

            size_mb = out_path.stat().st_size / 1_048_576
            print(f"  [{tx},{tz}] {len(faces):,} trójkątów → {out_path.name} ({size_mb:.1f} MB)")
            written += 1

    print(f"\nZapisano {written} kafelków do {meshes_dir}")


def main():
    args = parse_args()
    tiles = args.tiles
    t_start = time.monotonic()

    input_obj = args.input.resolve()
    out_dir = input_obj.parent / f"{input_obj.stem}_tiles"
    out_dir.mkdir(exist_ok=True)

    print(f"=== split_mesh.py  tiles={tiles}×{tiles} ===")
    print(f"Wejście: {input_obj}  ({input_obj.stat().st_size / 1_048_576:.0f} MB)")
    print(f"Wyjście: {out_dir}\n")

    verts = pass1_vertices(input_obj)
    pass2_split(input_obj, verts, tiles, out_dir)

    elapsed = time.monotonic() - t_start
    print(f"\nGotowe w {elapsed:.1f}s")
    print(f"\nNastępny krok: zaktualizuj add_lake() w isaac_sim.py")
    print(f"  → załaduj wszystkie big_lake_tile_*.obj zamiast big_lake_simp.obj")


if __name__ == "__main__":
    main()
