#!/usr/bin/env python3
"""Zwija każdy postój sweepa do jednego punktu - jedna kreska, jeden pomiar.

PROBLEM: kolektor zapisuje KAŻDĄ ramkę, która padła w okno bramki, a bramka stoi
otwarta przez cały dwell (0,5 s = ok. 9 ramek). Kierunek jest w tym czasie
idealnie stabilny, ale odległość skacze o kilka zliczeń ADC, więc jeden kierunek
daje w chmurze nie punkt, tylko krótką KRESKĘ wycelowaną w czujnik. Zmierzone na
skanie z 2026-08-21 (606 postojów): rozrzut poprzeczny mediana 0,0 mm / 90c
2,9 mm, rozrzut wzdłuż promienia mediana 7,2 cm / 90c 14,8 cm / max 31 cm.
Źródło to kwantyzacja ADC wzmocniona stromością krzywej Sharpa - przy 3 m jedno
zliczenie to ponad 10 cm.

ROZWIĄZANIE: mediana z postoju. Dwa powody, dla których to mediana, a nie średnia:

  1. Jest ODPORNA na pojedynczy odlot (skos, krawędź, chwilowe zgubienie echa),
     a takich ramek w postoju bywa 1-2 na 9.
  2. Jest PRZEZROCZYSTA NA KALIBRACJĘ. Krzywa ADC->metry jest ściśle malejąca,
     a przekształcenie monotoniczne zachowuje medianę - mediana metrów to
     dokładnie ta sama próbka co mediana ADC. Skrypt nie musi więc znać ani
     powtarzać kalibracji, a wynik jest ten sam, co przy zwijaniu w dziedzinie
     ADC. Dla ŚREDNIEJ to nie zachodzi (krzywa jest mocno nieliniowa) - to jest
     powód, żeby nie brać średniej nawet przy czystych danych.

Dlatego wynikowy wiersz to PRAWDZIWA, niezmodyfikowana ramka z postoju - ta
o medianowym ADC. Nic nie jest przeliczane, więc x/y/z, TF i kąty jointów
pozostają wzajemnie spójne. Dopisane są tylko kolumny opisujące zwinięte okno.

CZEGO SKRYPT NIE ROBI: nie dotyka surowego CSV. Zwijanie jest decyzją
analityczną, a nie poprawką danych - surowy plik zostaje jedynym źródłem prawdy
i można z niego zwinąć inaczej albo wcale.

Wejście:  CSV z cloud_collector_node (kolumny stamp, d, x, y, z; adc opcjonalne).
Wyjście:  <plik>_dwell.csv + <plik>_dwell.ply (albo ścieżki z -o).

    ./scripts/collapse_dwells.py ~/scans/laser_sweep_20260821_092919.csv
"""
import argparse
import csv
import os
import statistics


# Przerwa w stemplach, która oddziela dwa postoje. Dwell to 0,5 s, a przejazd
# między punktami siatki trwa ponad 1,5 s (settle 1,0 s + sam dojazd), więc
# 0,3 s leży z zapasem między odstępem ramek (ok. 0,05 s) a przerwą na przejazd.
# Grupowanie po kątach jointów byłoby GORSZE: odczyt enkodera skacze o pojedyncze
# zliczenia w obrębie jednego postoju i rozbija go na kilka grup.
DOMYSLNA_PRZERWA = 0.3


def wczytaj(sciezka):
    with open(sciezka) as f:
        czytnik = csv.DictReader(f)
        pola = list(czytnik.fieldnames)
        wiersze = list(czytnik)
    wiersze.sort(key=lambda w: float(w["stamp"]))
    return wiersze, pola


def podziel_na_postoje(wiersze, przerwa):
    postoje, biezacy = [], [wiersze[0]]
    for poprzedni, nastepny in zip(wiersze, wiersze[1:]):
        if float(nastepny["stamp"]) - float(poprzedni["stamp"]) > przerwa:
            postoje.append(biezacy)
            biezacy = []
        biezacy.append(nastepny)
    postoje.append(biezacy)
    return postoje


def zwin(postoj):
    """Zwraca ramkę o medianowym d, wzbogaconą o opis okna."""
    # Przy parzystej liczbie ramek bierzemy DOLNĄ ze środkowych, a nie średnią
    # z dwóch - chodzi o to, żeby wynik był istniejącym pomiarem, a nie liczbą
    # policzoną obok danych.
    posortowane = sorted(postoj, key=lambda w: float(w["d"]))
    przedstawiciel = dict(posortowane[(len(posortowane) - 1) // 2])

    d = [float(w["d"]) for w in postoj]
    adc = [float(w["adc"]) for w in postoj if w.get("adc")]

    przedstawiciel["n_ramek"] = len(postoj)
    przedstawiciel["d_min"] = f"{min(d):.6f}"
    przedstawiciel["d_max"] = f"{max(d):.6f}"
    przedstawiciel["d_rozrzut"] = f"{max(d) - min(d):.6f}"
    # Odchylenie od mediany, nie od średniej - spójnie z tym, co zwracamy.
    mediana = statistics.median(d)
    przedstawiciel["d_mad"] = f"{statistics.median([abs(x - mediana) for x in d]):.6f}"
    przedstawiciel["adc_rozrzut"] = f"{max(adc) - min(adc):.1f}" if adc else ""
    przedstawiciel["stamp_start"] = postoj[0]["stamp"]
    przedstawiciel["stamp_koniec"] = postoj[-1]["stamp"]
    return przedstawiciel


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv_wejscie")
    p.add_argument("-o", "--wyjscie", default="",
                   help="Baza nazw plików wyjściowych; puste = <wejście>_dwell")
    p.add_argument("--przerwa", type=float, default=DOMYSLNA_PRZERWA,
                   help=f"Przerwa w stemplach dzieląca postoje [s], domyślnie {DOMYSLNA_PRZERWA}")
    p.add_argument("--min-ramek", type=int, default=1,
                   help="Pomiń postoje krótsze niż tyle ramek (domyślnie 1 = bierz wszystkie)")
    args = p.parse_args()

    wiersze, pola = wczytaj(args.csv_wejscie)
    if not wiersze:
        raise SystemExit("Pusty plik wejściowy.")

    postoje = podziel_na_postoje(wiersze, args.przerwa)
    pominiete = [p_ for p_ in postoje if len(p_) < args.min_ramek]
    postoje = [p_ for p_ in postoje if len(p_) >= args.min_ramek]
    if not postoje:
        raise SystemExit("Wszystkie postoje odpadły przez --min-ramek.")

    punkty = [zwin(p_) for p_ in postoje]

    baza = args.wyjscie or os.path.splitext(args.csv_wejscie)[0] + "_dwell"
    dodatkowe = ["n_ramek", "d_min", "d_max", "d_rozrzut", "d_mad",
                 "adc_rozrzut", "stamp_start", "stamp_koniec"]

    with open(f"{baza}.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=pola + dodatkowe)
        w.writeheader()
        w.writerows(punkty)

    with open(f"{baza}.ply", "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(punkty)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("end_header\n")
        for pkt in punkty:
            f.write(f"{float(pkt['x']):.6f} {float(pkt['y']):.6f} {float(pkt['z']):.6f}\n")

    n = [p_["n_ramek"] for p_ in punkty]
    rozrzuty = sorted(float(p_["d_rozrzut"]) for p_ in punkty)
    mad = sorted(float(p_["d_mad"]) for p_ in punkty)
    c90 = lambda v: v[int(0.9 * len(v))]  # noqa: E731
    print(f"{len(wiersze)} ramek -> {len(punkty)} punktów "
          f"({len(wiersze) / len(punkty):.1f} ramki na postój)")
    if pominiete:
        print(f"pominięte postoje krótsze niż {args.min_ramek} ramek: {len(pominiete)}")
    print(f"ramek na postój: min {min(n)}, mediana {statistics.median(n):.0f}, max {max(n)}")
    print(f"rozrzut d w postoju [m]: mediana {statistics.median(rozrzuty):.3f}, "
          f"90c {c90(rozrzuty):.3f}, max {rozrzuty[-1]:.3f}")
    print(f"MAD wokół mediany [m]:   mediana {statistics.median(mad):.3f}, "
          f"90c {c90(mad):.3f}   <- tyle zostaje po zwinięciu")
    print(f"zapisano: {baza}.csv oraz {baza}.ply")


if __name__ == "__main__":
    main()
