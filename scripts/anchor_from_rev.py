#!/usr/bin/env python3
"""
anchor_from_rev.py - analiza testu rewersyjnego: kotwica czasu i luz osi.

Wejście: CSV z anchor_rev_test (t, d, el_deg, omega_meas, speed_cmd, dir...).

MODEL. Ten sam wiersz przejechany w obie strony daje dwa profile r(theta).
Jeśli czas pomiaru jest przesunięty o `dt` względem stempla, każdy punkt ląduje
o `omega*dt` w kierunku jazdy, więc profile rozjeżdżają się o

    Delta(omega) = 2*dt*omega + b

Nachylenie niesie CZAS, wyraz wolny LUZ mechaniczny (ten sam w obie strony, bo
zależy tylko od znaku prędkości). Jedna prędkość ich nie rozdziela - stąd kilka.

JAK MIERZYMY Delta. Nie parujemy pojedynczych pomiarów (nie ma ich jak sparować,
bo padają w innych miejscach), tylko dopasowujemy CAŁE PROFILE: szukamy
przesunięcia, przy którym profil "w górę" najlepiej pokrywa się z profilem
"w dół", w sensie sumy kwadratów różnic na wspólnej siatce kątów. Kształt sceny
jest więc mierzony, a nie zakładany - ale tylko tak dobrze, jak pozwala gęstość
próbkowania: interpolacja liniowa między punktami TO JEST założenie o prostej.

CZEGO TEN TEST NIE MIERZY. Wszystko, co jest WSPÓLNE dla obu kierunków, znosi
się i jest niewidoczne: skala kąta, zero osi, przesunięcie osi optycznej.
To jest test SPÓJNOŚCI, nie dokładności bezwzględnej.

Użycie:
  python3 scripts/anchor_from_rev.py ~/scans/anchor_rev_20260916_0142.csv
"""

import argparse
import csv
import math
import sys
from collections import defaultdict


def read_rows(path, cruise_frac):
    """Wczytuje pomiary, ZOSTAWIAJĄC TYLKO JAZDĘ ZE STAŁĄ PRĘDKOŚCIĄ.

    Dwa powody, oba psują model po cichu:
      * model Delta = 2*dt*omega zakłada omega RÓWNE ZADANEJ - na rozbiegu
        i hamowaniu jest mniejsze, więc te punkty zaniżałyby Delta;
      * etykieta kierunku pochodzi z chwili PRZYJŚCIA pomiaru, a nie z chwili
        jego wykonania. Pomiar zrobiony na postoju przy nawrocie potrafi przyjść
        już po starcie przejazdu w drugą stronę i dostać zły kierunek.

    PRĘDKOŚĆ LICZYMY TUTAJ, z sąsiednich pomiarów tego samego przejazdu
    (różnica centralna, rozstaw ~250 ms), a NIE bierzemy kolumny omega_meas.
    Ta ostatnia pochodzi z dwóch próbek /joint_states odległych o 20 ms, a jeden
    tick enkodera (0,088 st) daje w 20 ms skok 4,4 st/s - czyli omega_meas
    przyjmuje tylko wartości 0 / 4,4 / 8,8 / 13,2 ... Filtr "co najmniej 80%
    zadanej" zostawiał wtedy wyłącznie wyższy próg i RAPORTOWAŁ go jako prędkość:
    przy 5,5 st/s z firmware'u wychodziło 8,77, identycznie w dwóch zupełnie
    różnych przebiegach (2026-09-17). Różnica centralna ma kwant ~0,35 st/s,
    a kąt jest już interpolowany, więc nie jest skokowy.
    """
    raw = []
    with open(path) as f:
        for row in csv.DictReader(f):
            raw.append({
                "t": float(row["t"]),
                "d": float(row["d"]) if row["d"] else None,
                "el": float(row["el_deg"]),
                "speed": abs(float(row["speed_cmd"])),
                "dir": int(row["dir"]),
                "pass_id": int(row["pass_id"]),
            })

    groups = defaultdict(list)
    for r in raw:
        groups[(r["speed"], r["dir"], r["pass_id"])].append(r)

    out = []
    dropped = nan = 0
    for key, rows in groups.items():
        rows.sort(key=lambda r: r["t"])
        # Kolejne przejazdy tej samej pary (prędkość, kierunek, numer) nie
        # istnieją, ale na wszelki wypadek tniemy grupę na przerwach > 0,5 s,
        # żeby różnica centralna nie przeskoczyła przez nawrót.
        for i, r in enumerate(rows):
            if r["d"] is None:
                nan += 1
                continue
            if i == 0 or i == len(rows) - 1:
                dropped += 1
                continue
            a, b = rows[i - 1], rows[i + 1]
            if b["t"] - a["t"] > 0.5 or b["t"] <= a["t"]:
                dropped += 1
                continue
            omega = (b["el"] - a["el"]) / (b["t"] - a["t"])
            r = dict(r, omega=omega)
            out.append(r)

    # Filtr jazdy ze stałą prędkością - względem MEDIANY prędkości danej grupy
    # (prędkość, kierunek), a nie zadanej: firmware kwantuje profil co
    # 1,374 st/s, więc "6 st/s" jedzie naprawdę 5,5.
    med = {}
    by_sd = defaultdict(list)
    for r in out:
        if r["dir"] * r["omega"] > 0:
            by_sd[(r["speed"], r["dir"])].append(abs(r["omega"]))
    for k, v in by_sd.items():
        v.sort()
        med[k] = v[len(v) // 2]
    kept = []
    for r in out:
        m = med.get((r["speed"], r["dir"]))
        if m is None or r["dir"] * r["omega"] < cruise_frac * m:
            dropped += 1
            continue
        kept.append(r)
    print(f"Odrzucone poza jazdą ze stałą prędkością (rozbieg, hamowanie, nawrót, brzegi przejazdu): "
          f"{dropped}; bez odległości (NaN): {nan}")
    return kept


def resample(points, grid):
    """Profil r(theta) na wspólnej siatce, interpolacja liniowa. None tam, gdzie
    siatka wychodzi poza zakres danych - takich węzłów nie porównujemy."""
    pts = sorted(points)
    if len(pts) < 2:
        return [None] * len(grid)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    out = []
    j = 0
    for g in grid:
        if g < xs[0] or g > xs[-1]:
            out.append(None)
            continue
        while j + 1 < len(xs) - 1 and xs[j + 1] < g:
            j += 1
        k = j
        while k + 1 < len(xs) and xs[k + 1] < g:
            k += 1
        if xs[k + 1] == xs[k]:
            out.append(ys[k])
            continue
        f = (g - xs[k]) / (xs[k + 1] - xs[k])
        out.append(ys[k] + f * (ys[k + 1] - ys[k]))
    return out


def best_shift(up, down, lo, hi, step_deg, max_shift):
    """Rozjazd profilów Delta, dodatni gdy pomiar jest PÓŹNIEJSZY niż stempel.

    ZNAK JEST TU CAŁĄ TREŚCIĄ, bo decyduje, czy kotwicę dodać, czy odjąć.
    Wewnątrz przesuwamy profil "w dół" o `shift` i szukamy najlepszego pokrycia
    z profilem "w górę". Przesunięcie punktów o `shift` przesuwa samą FUNKCJĘ
    w przeciwną stronę, więc zwracamy `-shift`. Sprawdzone na danych
    syntetycznych o znanym dt: bez tej zmiany znaku wychodziło lustrzane odbicie
    prawdy, czyli poprawka pogarszałaby chmurę dwukrotnie zamiast ją naprawić."""
    grid = [lo + i * step_deg for i in range(int((hi - lo) / step_deg) + 1)]
    a = resample(up, grid)
    best = (None, None, 0)
    shift = -max_shift
    while shift <= max_shift + 1e-9:
        b = resample([(x + shift, y) for x, y in down], grid)
        diffs = sorted(abs(va - vb) for va, vb in zip(a, b) if va is not None and vb is not None)
        n = len(diffs)
        if n >= 10:
            # MEDIANA |różnic|, nie rms. Zmierzone 2026-09-17: 4 odczyty-śmieci
            # (~30 m na ścianie 2,6 m) na 1534 pomiary podnosiły rms do 4,2 m,
            # bo kwadrat błędu 28 m zjada całą resztę - i minimum wypadało
            # tam, gdzie akurat mniej śmieci trafiło w siatkę, a nie tam, gdzie
            # profile się pokrywają.
            cost = diffs[n // 2]
            if best[0] is None or cost < best[0]:
                best = (cost, -shift, n)
        shift += step_deg
    return best


def fit_line(xs, ys):
    n = len(xs)
    sx, sy = sum(xs), sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    den = n * sxx - sx * sx
    if abs(den) < 1e-12:
        return None, None
    a = (n * sxy - sx * sy) / den
    return a, (sy - a * sx) / n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--step", type=float, default=0.25, help="krok siatki kątów [st]")
    ap.add_argument("--max-shift", type=float, default=10.0, help="zakres szukania [st]")
    ap.add_argument("--cruise", type=float, default=0.8,
                    help="minimalny ułamek zadanej prędkości, żeby punkt wszedł do analizy")
    args = ap.parse_args()

    rows = read_rows(args.csv, args.cruise)
    if not rows:
        print("Brak pomiarów z odległością w pliku.")
        return 1

    by_speed = defaultdict(lambda: {1: [], -1: []})
    omegas = defaultdict(list)
    for r in rows:
        by_speed[r["speed"]][r["dir"]].append((r["el"], r["d"], r["pass_id"]))
        omegas[r["speed"]].append(abs(r["omega"]))

    def delta_for(up, down):
        if len(up) < 5 or len(down) < 5:
            return None, None, 0
        lo = max(min(x for x, _ in up), min(x for x, _ in down))
        hi = min(max(x for x, _ in up), max(x for x, _ in down))
        cost, shift, n = best_shift(up, down, lo, hi, args.step, args.max_shift)
        return cost, shift, n

    print(f"{len(rows)} pomiarów, prędkości zadane: {sorted(by_speed)}\n")
    print("Delta z CAŁOŚCI oraz z dwóch ROZŁĄCZNYCH połówek przejazdów (parzyste/nieparzyste).")
    print("Tylko rozjazd połówek jest uczciwą miarą niepewności - bootstrap na tych samych")
    print("punktach ją zaniża, bo nie widzi błędu interpolacji między próbkami.\n")
    print(f"{'zadane':>7} {'zmierz.':>8} {'pkt +':>6} {'pkt -':>6} {'Delta':>7} {'med|r|':>8}"
          f" {'Delta A':>8} {'Delta B':>8} {'rozjazd':>8}")
    xs, ys = [], []
    for speed in sorted(by_speed):
        up_all = [(x, d) for x, d, _ in by_speed[speed][1]]
        dn_all = [(x, d) for x, d, _ in by_speed[speed][-1]]
        cost, shift, n = delta_for(up_all, dn_all)
        if shift is None:
            print(f"{speed:7.1f}   za mało punktów")
            continue
        halves = []
        for parity in (0, 1):
            up = [(x, d) for x, d, k in by_speed[speed][1] if k % 2 == parity]
            dn = [(x, d) for x, d, k in by_speed[speed][-1] if k % 2 == parity]
            halves.append(delta_for(up, dn)[1])
        omega = sorted(omegas[speed])[len(omegas[speed]) // 2]
        ha = "   -" if halves[0] is None else f"{halves[0]:8.2f}"
        hb = "   -" if halves[1] is None else f"{halves[1]:8.2f}"
        spread = "   -" if None in halves else f"{abs(halves[0] - halves[1]):8.2f}"
        print(f"{speed:7.1f} {omega:8.2f} {len(up_all):6d} {len(dn_all):6d} {shift:7.2f}"
              f" {cost * 1000:6.1f}mm {ha} {hb} {spread}")
        # Do prostej bierzemy prędkość ZMIERZONĄ: firmware kwantuje profil co
        # 1,374 st/s, a przy strumieniowaniu pozycji realna prędkość potrafiła
        # odbiegać od zadanej o 45% (6 -> 8,8 st/s).
        xs.append(omega)
        ys.append(shift)

    if len(xs) < 2:
        print("\nZa mało prędkości, żeby rozdzielić czas od luzu.")
        return 1

    slope, intercept = fit_line(xs, ys)
    dt_ms = slope / 2.0 * 1000.0
    print(f"\nDopasowanie Delta = 2*dt*omega + b (omega ZMIERZONE):")
    print(f"  nachylenie {slope:+.5f} st/(st/s)  ->  dt = {dt_ms:+.1f} ms")
    print(f"  wyraz wolny b = {intercept:+.3f} st  ->  luz ~ {abs(intercept):.2f} st")
    print(f"\nDO WPISANIA: m703a_anchor_offset_ms:={dt_ms:+.0f}")
    print("Luz kompensuje się osobno, znakiem prędkości osi - nie kotwicą.")

    # Kontrola liniowości: co model przewiduje dla każdej zmierzonej prędkości.
    print("\nKontrola (im bliżej zera, tym lepiej trzyma się model liniowy):")
    for x, y in zip(xs, ys):
        print(f"  {x:5.1f} st/s: zmierzone {y:+.2f} st, model {slope * x + intercept:+.2f} st, "
              f"różnica {y - (slope * x + intercept):+.2f} st")
    return 0


if __name__ == "__main__":
    sys.exit(main())
