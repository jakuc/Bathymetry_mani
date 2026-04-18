#!/usr/bin/env python3
"""
Czyści kafelki OBJ przez pymeshlab — usuwa oderwane fragmenty i zdegenerowane face'y.

Użycie:
    python3 clean_tiles.py [--pattern "big_lake_simp_tile_*.obj"]

Wyjście: pliki nadpisane w miejscu (ta sama nazwa).
"""

import argparse
import pathlib
import time

SCRIPT_DIR = pathlib.Path(__file__).parent.resolve()
MESHES_DIR = SCRIPT_DIR.parent / "meshes"


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--pattern", default="*_tile_*.obj",
                   help="Glob pattern kafelków w katalogu meshes (domyślnie *_tile_*.obj)")
    return p.parse_args()


def clean(ms, path: pathlib.Path) -> dict:
    ms.load_new_mesh(str(path))

    before = ms.current_mesh().face_number()

    ms.meshing_remove_duplicate_faces()
    ms.meshing_remove_duplicate_vertices()
    ms.meshing_remove_null_faces()
    ms.meshing_remove_connected_component_by_face_number(mincomponentsize=100)
    after = ms.current_mesh().face_number()

    if after == 0:
        return {"before": before, "after": 0, "removed": before, "skipped": True}

    ms.meshing_close_holes(maxholesize=200)
    after = ms.current_mesh().face_number()

    ms.save_current_mesh(str(path))
    ms.clear()

    return {"before": before, "after": after, "removed": before - after, "skipped": False}


def main():
    args = parse_args()
    tiles = sorted(MESHES_DIR.glob(args.pattern))

    if not tiles:
        print(f"Brak plików pasujących do wzorca '{args.pattern}' w {MESHES_DIR}")
        return

    import pymeshlab

    print(f"Czyszczenie {len(tiles)} kafelków...\n")
    t_start = time.monotonic()

    for path in tiles:
        ms = pymeshlab.MeshSet()
        t0 = time.monotonic()
        stats = clean(ms, path)
        elapsed = time.monotonic() - t0
        if stats.get("skipped"):
            print(f"  {path.name}: pusty po czyszczeniu — pominięto zapis")
        else:
            print(f"  {path.name}: {stats['before']:,} → {stats['after']:,} face'ów "
                  f"(usunięto {stats['removed']:,}) [{elapsed:.1f}s]")

    print(f"\nGotowe w {time.monotonic() - t_start:.1f}s")


if __name__ == "__main__":
    main()
