#!/usr/bin/env python3
"""Kalibracja krzywej dalmierza Sharp z samego skanu - bez taśmy mierniczej.

POMYSŁ: płaska ściana MUSI wyjść płaska. Szukamy takich stałych modelu
`L_cm = A / (V - B)`, przy których płaszczyzny w chmurze są najbardziej płaskie.
To znacznie mocniejsze ograniczenie niż kilka punktów z taśmy, bo opiera się na
tysiącach pomiarów rozłożonych po całym zakresie odległości.

DLACZEGO TO DZIAŁA NA KRZYWIZNĘ: błąd stały wzdłuż promienia wypycha każdy punkt
na zewnątrz wzdłuż jego własnego kierunku. Ściana leży w odległości d/cos(theta),
więc taki błąd zamienia płaszczyznę w powierzchnię wybrzuszoną - tym bardziej,
im dalej od prostopadłej. To jest obserwowany "banan".

CZEGO NIE DA SIĘ TAK WYZNACZYĆ: zmiana A to niemal czyste skalowanie, a
przeskalowana płaszczyzna dalej jest płaszczyzną. Płaskość ogranicza więc B
(kształt krzywej), ale prawie nie ogranicza A (skalę). Skalę trzeba przybić
JEDNYM pomiarem znanej odległości - skrypt to raportuje, zamiast udawać, że
wyznaczył oba parametry.

Wejście: CSV z cloud_collector_node (kolumny d, laser_t*, laser_q*).
"""
import argparse
import csv
import math

import numpy as np

A0, B0 = 134.44, 1.1556          # stałe z karty katalogowej - punkt wyjścia


def wczytaj(sciezka):
    d, t, q = [], [], []
    with open(sciezka) as f:
        for row in csv.DictReader(f):
            d.append(float(row["d"]))
            t.append([float(row["laser_tx"]), float(row["laser_ty"]), float(row["laser_tz"])])
            q.append([float(row["laser_qx"]), float(row["laser_qy"]),
                      float(row["laser_qz"]), float(row["laser_qw"])])
    return np.array(d), np.array(t), np.array(q)


def osie_wiazki(q):
    """Kierunek +X ramki dalmierza (oś wiązki) w ukladzie docelowym."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)], axis=1)


def napiecie(d_m):
    """Odwrócenie modelu: z zapisanej odleglosci odzyskujemy surowy pomiar."""
    return A0 / (100.0 * d_m) + B0


def osie_wszystkie(q):
    """Pelna macierz obrotu ramki dalmierza - kolumny to osie X, Y, Z."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    ex = np.stack([1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)], axis=1)
    ey = np.stack([2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)], axis=1)
    ez = np.stack([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)], axis=1)
    return ex, ey, ez


def punkty(V, A, B, t, u, offset=None, osie=None):
    """Chmura dla kandydujacych stalych. NaN tam, gdzie model sie nie stosuje.

    `offset` to przesuniecie POCZATKU WIAZKI wzgledem ramki dalmierza, wyrazone
    w tej ramce. Model domyslnie zaklada zero, czyli ze promien wychodzi dokladnie
    z osi obrotu glowicy. Jesli czujnik siedzi na wsporniku, jego rzeczywisty
    poczatek zatacza przy obrocie okrag, a nieruchomy model zamienia plaszczyzne
    w luk o amplitudzie rzedu tego ramienia.
    """
    mian = V - B
    with np.errstate(divide="ignore", invalid="ignore"):
        d = np.where(mian > 1e-6, A / mian / 100.0, np.nan)
    P = t + u * d[:, None]
    if offset is not None:
        ex, ey, ez = osie
        P = P + ex * offset[0] + ey * offset[1] + ez * offset[2]
    return P, d


def dopasuj_plaszczyzne(P):
    """Zwraca (normalna, punkt, reszty prostopadle)."""
    c = P.mean(axis=0)
    _, _, Vt = np.linalg.svd(P - c, full_matrices=False)
    n = Vt[-1]
    return n, c, (P - c) @ n


def ransac_plaszczyzny(P, ile, prog, min_pkt, iteracje=3000, rng=None):
    """Największe płaszczyzny w chmurze. Próg celowo luźny - szukamy ŚCIANY,
    która na tym etapie jest jeszcze wygięta, więc ciasny próg rozbiłby ją
    na kawałki i optymalizacja nie miałaby czego prostować."""
    rng = rng or np.random.default_rng(0)
    zostalo = np.arange(len(P))
    grupy = []
    for _ in range(ile):
        if len(zostalo) < min_pkt:
            break
        Q = P[zostalo]
        best_cnt, best_mask = 0, None
        for _ in range(iteracje):
            idx = rng.choice(len(Q), 3, replace=False)
            a, b, c = Q[idx]
            n = np.cross(b - a, c - a)
            nn = np.linalg.norm(n)
            if nn < 1e-9:
                continue
            n = n / nn
            odl = np.abs((Q - a) @ n)
            mask = odl < prog
            cnt = int(mask.sum())
            if cnt > best_cnt:
                best_cnt, best_mask = cnt, mask
        if best_mask is None or best_cnt < min_pkt:
            break
        grupy.append(zostalo[best_mask])
        zostalo = zostalo[~best_mask]
    return grupy


def rms_plaskosci(V, A, B, t, u, grupy):
    """RMS reszt od płaszczyzn, ZNORMALIZOWANY średnią odległością.

    Normalizacja nie jest kosmetyką. Zmiana A to niemal czyste skalowanie chmury,
    więc surowy RMS w milimetrach maleje wraz z A po prostu dlatego, że wszystko
    się kurczy - i optymalizacja zbiegłaby do A -> 0. Dzieląc przez średnią
    odległość mierzymy KSZTAŁT, a nie rozmiar, i degeneracja skali przestaje
    udawać poprawę.
    """
    P, d = punkty(V, A, B, t, u)
    suma, n, dsum = 0.0, 0, 0.0
    for g in grupy:
        ok = np.isfinite(P[g]).all(axis=1)
        Q, dq = P[g][ok], d[g][ok]
        if len(Q) < 10:
            continue
        _, _, r = dopasuj_plaszczyzne(Q)
        suma += float((r ** 2).sum())
        dsum += float(dq.sum())
        n += len(Q)
    if not n:
        return float("inf")
    return math.sqrt(suma / n) / (dsum / n)


def profil_reszt(V, A, B, t, u, g, koszyki=8):
    """Średnia reszta ZE ZNAKIEM w funkcji odległości - bezpośredni obraz banana.

    Sam szum jest symetryczny, więc uśredniony w koszyku znika. To, co zostaje,
    jest błędem systematycznym: jeśli rośnie monotonicznie z odległością,
    płaszczyzna jest wyginana przez model czujnika, a nie przez szum.
    """
    P, d = punkty(V, A, B, t, u)
    ok = np.isfinite(P[g]).all(axis=1)
    Q, dq = P[g][ok], d[g][ok]
    _, _, r = dopasuj_plaszczyzne(Q)
    kr = np.linspace(dq.min(), dq.max(), koszyki + 1)
    out = []
    for i in range(koszyki):
        m = (dq >= kr[i]) & (dq <= kr[i + 1] if i == koszyki - 1 else dq < kr[i + 1])
        if m.sum() >= 15:
            out.append((0.5 * (kr[i] + kr[i + 1]), int(m.sum()),
                        float(r[m].mean()), float(r[m].std())))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--plaszczyzn", type=int, default=5)
    ap.add_argument("--prog", type=float, default=0.10, help="próg RANSAC [m]")
    ap.add_argument("--min-pkt", type=int, default=250)
    args = ap.parse_args()

    d, t, q = wczytaj(args.csv)
    u = osie_wiazki(q)
    V = napiecie(d)
    print("wczytano %d punktów, zakres odleglosci %.2f-%.2f m\n" % (len(d), d.min(), d.max()))

    P0 = t + u * d[:, None]
    grupy = ransac_plaszczyzny(P0, args.plaszczyzn, args.prog, args.min_pkt)
    if not grupy:
        print("nie znalazłem żadnej płaszczyzny - poluzuj --prog albo --min-pkt")
        return
    print("znalezione płaszczyzny (na chmurze wyjściowej):")
    for i, g in enumerate(grupy):
        _, c, r = dopasuj_plaszczyzne(P0[g])
        print("  %d: %5d pkt, RMS od płaszczyzny %6.1f mm, srodek (%.2f %.2f %.2f)"
              % (i, len(g), r.std() * 1000, *c))

    najwieksza = max(grupy, key=len)
    print("\n--- czy to na pewno banan? reszta ZE ZNAKIEM vs odleglosc"
          " (najwieksza plaszczyzna, %d pkt) ---" % len(najwieksza))
    print("%10s %8s %12s %10s" % ("d [m]", "n", "srednia [mm]", "sigma [mm]"))
    for dd, n, sr, sd in profil_reszt(V, A0, B0, t, u, najwieksza):
        print("%10.2f %8d %12.1f %10.1f" % (dd, n, sr * 1000, sd * 1000))
    print("Szum jest symetryczny i w sredniej znika. Monotoniczny trend sredniej")
    print("= blad systematyczny modelu, czyli wyginanie plaszczyzny.")

    baza = rms_plaskosci(V, A0, B0, t, u, grupy)
    print("\nZnormalizowany RMS dla stalych z karty (A=%.2f B=%.4f): %.4f  (%.1f mm/m)"
          % (A0, B0, baza, baza * 1000))

    print("\n--- dopasowanie B (A = %.2f) ---" % A0)
    najlepsze = (baza, B0)
    for B in np.arange(0.40, 1.70, 0.005):
        r = rms_plaskosci(V, A0, float(B), t, u, grupy)
        if r < najlepsze[0]:
            najlepsze = (r, float(B))
    print("najlepsze B = %.3f -> %.1f mm/m  (bylo %.1f mm/m przy B=%.4f)"
          % (najlepsze[1], najlepsze[0] * 1000, baza * 1000, B0))

    print("\n--- wrazliwosc na A (przy B = %.3f), miara bezwymiarowa ---" % najlepsze[1])
    print("%10s %14s" % ("A", "RMS [mm/m]"))
    for A in (0.7 * A0, 0.85 * A0, A0, 1.15 * A0, 1.3 * A0):
        print("%10.2f %14.1f" % (A, rms_plaskosci(V, A, najlepsze[1], t, u, grupy) * 1000))

    print("\n--- wspolne dopasowanie A i B ---")
    best = (najlepsze[0], A0, najlepsze[1])
    for A in np.arange(0.6 * A0, 1.45 * A0, 0.02 * A0):
        for B in np.arange(0.40, 1.70, 0.01):
            r = rms_plaskosci(V, float(A), float(B), t, u, grupy)
            if r < best[0]:
                best = (r, float(A), float(B))
    print("A = %.2f, B = %.3f -> %.1f mm/m (poprawa z %.1f mm/m)"
          % (best[1], best[2], best[0] * 1000, baza * 1000))

    # ---------------------------------------------------------------- geometria
    # Kalibracja czujnika zawiodla, wiec sprawdzamy druga hipoteze: offset montazu.
    from scipy.optimize import minimize

    osie = osie_wszystkie(q)

    def koszt(par):
        off = par[:3]
        B = par[3] if len(par) > 3 else B0
        P, d = punkty(V, A0, B, t, u, off, osie)
        suma, n, dsum = 0.0, 0, 0.0
        for g in grupy:
            ok = np.isfinite(P[g]).all(axis=1)
            Q, dq = P[g][ok], d[g][ok]
            if len(Q) < 10:
                continue
            _, _, r = dopasuj_plaszczyzne(Q)
            suma += float((r ** 2).sum()); dsum += float(dq.sum()); n += len(Q)
        return math.sqrt(suma / n) / (dsum / n) if n else 1e9

    print("\n--- hipoteza 2: offset montazu dalmierza (laser_xyz) ---")
    print("start (offset 0,0,0):            %.1f mm/m" % (koszt(np.zeros(3)) * 1000))
    res = minimize(koszt, np.zeros(3), method="Nelder-Mead",
                   options={"xatol": 1e-4, "fatol": 1e-7, "maxiter": 600})
    print("najlepszy offset: x=%+.4f y=%+.4f z=%+.4f m  ->  %.1f mm/m"
          % (res.x[0], res.x[1], res.x[2], res.fun * 1000))
    print("dlugosc ramienia: %.1f mm" % (np.linalg.norm(res.x) * 1000))

    print("\n--- offset + B razem ---")
    res2 = minimize(koszt, np.r_[res.x, B0], method="Nelder-Mead",
                    options={"xatol": 1e-4, "fatol": 1e-7, "maxiter": 1200})
    print("offset x=%+.4f y=%+.4f z=%+.4f m, B=%.4f  ->  %.1f mm/m"
          % (res2.x[0], res2.x[1], res2.x[2], res2.x[3], res2.fun * 1000))

    print("\n--- profil reszt PO korekcie geometrii ---")
    P_ok, _ = punkty(V, A0, res2.x[3], t, u, res2.x[:3], osie)
    ok = np.isfinite(P_ok[najwieksza]).all(axis=1)
    Q = P_ok[najwieksza][ok]
    _, dcal = punkty(V, A0, res2.x[3], t, u, res2.x[:3], osie)
    dq = dcal[najwieksza][ok]
    _, _, r = dopasuj_plaszczyzne(Q)
    kr = np.linspace(dq.min(), dq.max(), 9)
    print("%10s %8s %12s %10s" % ("d [m]", "n", "srednia [mm]", "sigma [mm]"))
    for i in range(8):
        m = (dq >= kr[i]) & (dq <= kr[i + 1] if i == 7 else dq < kr[i + 1])
        if m.sum() >= 15:
            print("%10.2f %8d %12.1f %10.1f"
                  % (0.5 * (kr[i] + kr[i + 1]), int(m.sum()),
                     r[m].mean() * 1000, r[m].std() * 1000))


if __name__ == "__main__":
    main()
