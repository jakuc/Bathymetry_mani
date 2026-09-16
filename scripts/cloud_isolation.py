#!/usr/bin/env python3
"""
cloud_isolation.py - ile w chmurze jest SAMOTNYCH punktów i czy odcina je próg SQ.

PYTANIE, NA KTÓRE ODPOWIADA (user, 2026-09-17): skoki o 30 cm między sąsiednimi
pomiarami są normalne (rury i profile pod sufitem). Źle jest, gdy w środku
pomieszczenia pojawia się punkt daleko od czegokolwiek innego - taki punkt nie
opisuje żadnej powierzchni, więc dalmierz skłamał.

MIARA. Dla każdego punktu odległość do NAJBLIŻSZEGO innego punktu w 3D,
podzielona przez OCZEKIWANY odstęp siatki na tej odległości:

    izolacja = nn_3d / (d * krok_kątowy)

Dzielenie jest konieczne, żeby miara działała w każdej scenie, a nie tylko
w pokoju: przy kroku 3 st sąsiednie punkty ściany 3 m dalej leżą o ~16 cm od
siebie, ale na 10 m już o 52 cm. Liczba w metrach uznałaby każdy daleki, zupełnie
poprawny punkt za samotnika. Izolacja ~1 to zwykły sąsiad na siatce; kilka
i więcej to punkt, do którego żaden inny nie przylega.

Bez kryterium odległości od głowicy - świadomie (decyzja usera).

Użycie:
  python3 scripts/cloud_isolation.py polsfera_ciagla_*.csv [--step-deg 3]
  python3 scripts/cloud_isolation.py a.csv b.csv          # porównanie dwóch skanów
"""

import argparse
import csv
import math
import sys

import numpy as np


def load(path):
    xyz, d, sq, el = [], [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                x, y, z, dist = (float(row[k]) for k in ("x", "y", "z", "d"))
            except (KeyError, ValueError):
                continue
            s = row.get("signal_quality", "")
            xyz.append((x, y, z))
            d.append(dist)
            sq.append(float(s) if s not in ("", "None") else math.nan)
            # Elewacja z kolumny kolektora: OSTATNIE /joint_states przy odbiorze,
            # nie interpolowane - do podziału na obszary wystarczy, do geometrii nie.
            e = row.get("j_xm540_joint", "")
            el.append(math.degrees(float(e)) if e not in ("", "None") else math.nan)
    return np.array(xyz), np.array(d), np.array(sq), np.array(el)


def nearest_neighbour(xyz, chunk=1024):
    """Odległość do najbliższego INNEGO punktu; brute force w porcjach, bo
    kilka tysięcy punktów to kilka milionów par - bez potrzeby scipy."""
    n = len(xyz)
    out = np.empty(n)
    for i in range(0, n, chunk):
        a = xyz[i:i + chunk]
        dist = np.sqrt(((a[:, None, :] - xyz[None, :, :]) ** 2).sum(axis=2))
        for j in range(len(a)):
            dist[j, i + j] = np.inf           # sam do siebie
        out[i:i + chunk] = dist.min(axis=1)
    return out


def report(path, step_deg, thresholds, iso_levels):
    xyz, d, sq, el = load(path)
    n = len(d)
    if n < 10:
        print(f"{path}: za mało punktów ({n})")
        return
    step = math.radians(step_deg)
    print(f"\n=== {path}")
    print(f"{n} punktów, bez SQ: {int(np.isnan(sq).sum())}")

    def isolation(mask):
        nn = nearest_neighbour(xyz[mask])
        return nn / np.maximum(d[mask] * step, 1e-3)

    iso_all = isolation(np.ones(n, dtype=bool))
    head = "  próg SQ   zostaje  " + "  ".join(f"izol>{lv:<4g}" for lv in iso_levels)
    print(head)

    def row(label, mask, iso):
        kept = int(mask.sum())
        cells = "  ".join(f"{int((iso > lv).sum()):5d} ({100 * (iso > lv).mean():4.1f}%)" for lv in iso_levels)
        print(f"  {label:>8}  {kept:6d}   {cells}")

    row("brak", np.ones(n, dtype=bool), iso_all)
    for thr in thresholds:
        mask = np.nan_to_num(sq, nan=np.inf) <= thr
        if mask.sum() < 10:
            continue
        # IZOLACJĘ LICZYMY OD NOWA po odcięciu: usunięcie słabych punktów
        # potrafi osierocić dobre, które sąsiadowały tylko z nimi. Liczenie
        # na starej izolacji ukryłoby ten koszt filtra.
        row(f"<= {thr:g}", mask, isolation(mask))

    # KRYTERIUM ŁĄCZONE: odrzucamy tylko punkt SŁABY i jednocześnie SAMOTNY.
    # Sam próg SQ działa jak ukryte cięcie odległości (sygnał słabnie z dystansem:
    # w pokoju 2026-09-17 odciął 100% punktów > 5 m i 24% ściany na 3-5 m),
    # a sama izolacja wycięłaby rzadkie, ale dobre punkty (pojedyncza rura).
    print("  kryterium łączone (SQ > próg I izolacja > poziom) - ile punktów odrzuca:")
    for thr in (1000, 1500, 2000):
        weak = np.nan_to_num(sq, nan=np.inf) > thr
        cells = "  ".join(f"izol>{lv:g}: {int((weak & (iso_all > lv)).sum()):3d}" for lv in iso_levels)
        print(f"    SQ > {thr:4d}: {cells}")

    # OBSZARY po elewacji: |el| < 30 to okolica zenitu, czyli sufit (świetlówki).
    regions = [("sufit |el|<30", np.abs(el) < 30), ("skos 30-60", (np.abs(el) >= 30) & (np.abs(el) < 60)),
               ("ściany |el|>=60", np.abs(el) >= 60)]
    print("  obszary:       punkty  NaN-free SQ med   SQ p90   izol>3   izol>5")
    for name, m in regions:
        if m.sum() < 5:
            continue
        s = sq[m]
        print(f"    {name:15s} {int(m.sum()):5d}   {np.nanmedian(s):8.0f}  {np.nanpercentile(s, 90):7.0f}"
              f"   {int((iso_all[m] > 3).sum()):5d}   {int((iso_all[m] > 5).sum()):5d}")

    lone = iso_all > max(iso_levels)
    if lone.any():
        print(f"  SQ samotników (izol > {max(iso_levels):g}): mediana {np.nanmedian(sq[lone]):.0f}; "
              f"SQ reszty: mediana {np.nanmedian(sq[~lone]):.0f}")
        far = d[lone]
        print(f"  odległości samotników: mediana {np.median(far):.2f} m, "
              f"min {far.min():.2f}, max {far.max():.2f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--step-deg", type=float, default=3.0,
                    help="krok kątowy siatki [st]: odstęp wierszy / punktów w wierszu")
    ap.add_argument("--sq", type=float, nargs="*", default=[3000, 2000, 1500, 1000, 700, 500])
    ap.add_argument("--iso", type=float, nargs="*", default=[3, 5, 10])
    ap.add_argument("--write", action="store_true",
                    help="zapisz obok <plik>_filtr.csv z kolumnami isolation i keep "
                         "(kryterium łączone --drop-sq / --drop-iso); oryginał nietknięty")
    ap.add_argument("--drop-sq", type=float, default=1500.0)
    ap.add_argument("--drop-iso", type=float, default=3.0)
    args = ap.parse_args()
    for path in args.csv:
        report(path, args.step_deg, args.sq, args.iso)
        if args.write:
            write_filtered(path, args.step_deg, args.drop_sq, args.drop_iso)
    return 0


def load_rows(path):
    """Numer wiersza skanu dla każdego punktu - z azymutu jointa (w wierszu stały)."""
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                float(row["x"])
            except (KeyError, ValueError):
                continue
            a = row.get("j_xm540_joint_z", "")
            rows.append(math.degrees(float(a)) if a not in ("", "None") else math.nan)
    return np.array(rows)


def cross_row_isolation(xyz, d, row_az, step_deg, chunk=1024):
    """Izolacja liczona TYLKO względem punktów z INNYCH wierszy skanu.

    Dlaczego nie zwykły najbliższy sąsiad: śmieci modułu przychodzą SERIAMI.
    Dwa kolejne strzały w jednym wierszu potrafią zwrócić prawie tę samą
    fałszywą wartość (34,74 i 34,77 m, 2026-09-17), więc każdy ma sąsiada 3 cm
    dalej i zwykła izolacja wynosi ~0 - filtr ich nie widział.
    Prawdziwą powierzchnię widzi też SĄSIEDNI przejazd azymutu (3 st obok);
    serii śmieci z jednego wiersza nie potwierdza żaden inny.
    """
    step = math.radians(step_deg)
    n = len(xyz)
    out = np.empty(n)
    rid = np.round(np.nan_to_num(row_az, nan=1e6) / step_deg).astype(np.int64)
    for i in range(0, n, chunk):
        a = xyz[i:i + chunk]
        dist = np.sqrt(((a[:, None, :] - xyz[None, :, :]) ** 2).sum(axis=2))
        same_row = rid[i:i + chunk, None] == rid[None, :]
        dist[same_row] = np.inf
        out[i:i + chunk] = dist.min(axis=1)
    return out / np.maximum(d * step, 1e-3)


def write_filtered(path, step_deg, drop_sq, drop_iso):
    """Kryterium łączone: wyrzuć punkt SŁABY (SQ > drop_sq) I NIEPOTWIERDZONY
    przez inne wiersze (izolacja międzywierszowa > drop_iso).
    Bez kryterium odległości - decyzja usera."""
    xyz, d, sq, _ = load(path)
    row_az = load_rows(path)
    iso = cross_row_isolation(xyz, d, row_az, step_deg)
    drop = (np.nan_to_num(sq, nan=np.inf) > drop_sq) & (iso > drop_iso)
    out = path[:-4] + "_filtr.csv" if path.endswith(".csv") else path + "_filtr.csv"
    with open(path) as fin, open(out, "w", newline="") as fout:
        rd = csv.DictReader(fin)
        rows = [r for r in rd if r.get("x") not in (None, "")]
        w = csv.DictWriter(fout, fieldnames=list(rd.fieldnames) + ["isolation_cross_row", "keep"])
        w.writeheader()
        for r, i, dr in zip(rows, iso, drop):
            r["isolation_cross_row"] = f"{i:.2f}"
            r["keep"] = int(not dr)
            w.writerow(r)
    kept = d[~drop]
    strong_lone = int(((np.nan_to_num(sq, nan=np.inf) <= drop_sq) & (iso > drop_iso)).sum())
    print(f"  zapisano {out}: odrzucone {int(drop.sum())}, zostaje {int((~drop).sum())}; "
          f"max odległość zostających {kept.max():.2f} m, zostających > 5 m: {int((kept > 5).sum())}; "
          f"niepotwierdzonych, ale z MOCNYM sygnałem (zostawione): {strong_lone}")


if __name__ == "__main__":
    sys.exit(main())
