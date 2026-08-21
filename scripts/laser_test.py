#!/usr/bin/env python3
"""laser_test.py - szybka weryfikacja dalmierza Sharp GP2Y0A710K0F na Arduino Nano.

Czyta ramki `LSR,<mediana>,<min>,<max>,<n>` z firmware'u firmware/laser_nano
i przelicza surowy ADC na metry. Kalibracja jest TUTAJ, nie w firmware - patrz
komentarz w laser_nano.ino.

  ./scripts/laser_test.py                     # podgląd na żywo, port autowykryty
  ./scripts/laser_test.py --port /dev/ttyUSB1
  ./scripts/laser_test.py --calib 1.50        # zbierz punkt kalibracyjny @1,50 m
  ./scripts/laser_test.py --fit calib.csv     # dopasuj stałe A/B (model przestarzały)
  ./scripts/laser_test.py --table calib.csv   # wypisz tabelę kalibracyjną do URDF-a
  ./scripts/laser_test.py --log pomiar.csv    # dopisuj odczyty do CSV

MODEL CZUJNIKA
Karta katalogowa obiecuje, że napięcie jest liniowe względem ODWROTNOŚCI
odległości:  V = A / L_cm + B.  Dla naszego egzemplarza to NIEPRAWDA - zmierzone
2026-08-20 na 7 punktach 1,05-3,69 m. Nachylenie dV/d(1/L) liczone między
sąsiednimi punktami spada monotonicznie 167,8 -> 147,9 -> 140,2 -> 102,1 ->
91,0 -> 50,2, a w modelu hiperbolicznym byłoby stałe. W walidacji leave-one-out
dopasowanie A/B myli się o 26 cm.

Dlatego domyślnym przeliczeniem jest TABELA punktów kalibracyjnych z interpolacją
PCHIP (monotoniczna sześcienna Fritscha-Carlsona) - dokładnie to samo, co robi
wtyczka LaserSensor. Podaj --calib-csv <plik>, żeby podgląd liczył z tabeli.
Tryb --fit zostaje do celów porównawczych; --table wypisuje gotowy wiersz do
wklejenia w laser.xacro.

ZASIĘG
Powyżej ~3 m czujnikowi kończy się rozdzielczość: jedno zliczenie ADC to 0,3 cm
przy 1 m, ale 6 cm przy 2,93 m i ponad 10 cm przy 3,2 m. 90% całego zakresu ADC
zużywa się na pierwsze 1,5 m. Odczyty zza tej granicy raportujemy jako
NIEJEDNOZNACZNE - wyglądają tak samo pewnie jak każde inne i nie niosą odległości.

PUŁAPKA ZAKRESU
Poniżej ~100 cm krzywa się zawija: 40 cm daje to samo napięcie co ~150 cm.
Czujnik nie ma jak zasygnalizować "za blisko", więc taki odczyt to cichy błąd
o metr. Wszystko powyżej napięcia dla 100 cm raportujemy jako NIEJEDNOZNACZNE,
a nie jako odległość.
"""

import argparse
import csv
import glob
import math
import os
import statistics
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("Brak pyserial. Zainstaluj: pip install pyserial  (albo apt install python3-serial)")

# V = A / L_cm + B  (patrz docstring)
DEFAULT_A = 134.44
DEFAULT_B = 1.1556

VCC = 5.0          # referencja ADC Nano = jego Vcc
ADC_MAX = 1023.0   # 10-bitowy przetwornik

RANGE_MIN_CM = 100.0   # karta katalogowa GP2Y0A710K0F
RANGE_MAX_CM = 300.0   # NIE 550 z karty: powyżej 3 m kończy się rozdzielczość

TOO_CLOSE = "za blisko"
TOO_FAR = "za daleko"


def adc_to_volts(adc):
    return adc * VCC / ADC_MAX


def volts_to_cm(volts, a, b):
    """Zwraca (odległość_cm, status). Odległość None, gdy odczyt jest bezużyteczny."""
    denom = volts - b
    if denom <= 0:
        # Napięcie poniżej asymptoty modelu - w praktyce brak echa albo cel dalej
        # niż zasięg czujnika.
        return None, TOO_FAR
    cm = a / denom
    if cm < RANGE_MIN_CM:
        # Strefa zawinięcia krzywej: ta sama wartość odpowiada dwóm odległościom.
        return None, TOO_CLOSE
    if cm > RANGE_MAX_CM:
        return None, TOO_FAR
    return cm, "ok"


def read_calib_csv(path):
    """Zwraca [(adc, L_cm)] posortowane rosnąco po ADC."""
    with open(path, newline="") as f:
        rows = [r for r in csv.DictReader(f)]
    nodes = sorted((float(r["adc_mean"]), float(r["distance_m"]) * 100.0) for r in rows)
    if len(nodes) < 2:
        sys.exit(f"{path}: potrzeba co najmniej 2 punktów, jest {len(nodes)}.")
    for (a1, l1), (a2, l2) in zip(nodes, nodes[1:]):
        if a2 == a1:
            sys.exit(f"{path}: dwa punkty przy tym samym ADC ({a1}).")
        if l2 >= l1:
            sys.exit(f"{path}: niemonotonicznie - ADC {a1} -> {l1} cm, ADC {a2} -> {l2} cm. "
                     "Rosnące ADC musi znaczyć malejącą odległość; sprawdź pomiar.")
    return nodes


def pchip_slopes(xs, ys):
    """Nachylenia Fritscha-Carlsona. Ten sam algorytm co w laser_sensor.cpp
    i co scipy.interpolate.PchipInterpolator - porównane numerycznie."""
    n = len(xs)
    h = [xs[i + 1] - xs[i] for i in range(n - 1)]
    delta = [(ys[i + 1] - ys[i]) / h[i] for i in range(n - 1)]
    slope = [0.0] * n

    for i in range(1, n - 1):
        if delta[i - 1] * delta[i] <= 0.0:
            slope[i] = 0.0          # ekstremum lokalne: płasko, żeby nie przestrzelić
        else:
            w1 = 2.0 * h[i] + h[i - 1]
            w2 = h[i] + 2.0 * h[i - 1]
            slope[i] = (w1 + w2) / (w1 / delta[i - 1] + w2 / delta[i])

    def edge(h0, h1, d0, d1):
        s = ((2.0 * h0 + h1) * d0 - h0 * d1) / (h0 + h1)
        if s * d0 <= 0.0:
            return 0.0
        if d0 * d1 <= 0.0 and abs(s) > abs(3.0 * d0):
            return 3.0 * d0
        return s

    if n == 2:
        return [delta[0], delta[0]]
    slope[0] = edge(h[0], h[1], delta[0], delta[1])
    slope[-1] = edge(h[-1], h[-2], delta[-1], delta[-2])
    return slope


def make_table_model(nodes):
    """Zwraca funkcję adc -> (odległość_cm, status), interpolującą 1/L przez PCHIP.

    Interpolujemy 1/L, a nie L: tam zależność jest prawie liniowa, więc
    interpolator wnosi drobną korektę zamiast odtwarzać cały kształt krzywej.
    Poza skrajnymi węzłami zwracamy status zamiast liczby - PCHIP przedłużyłby
    brzegowy wielomian sześcienny i potrafi uciec o metry.
    """
    xs = [a for a, _ in nodes]
    ys = [1.0 / l for _, l in nodes]
    slope = pchip_slopes(xs, ys)

    def model(adc):
        if adc < xs[0]:
            return None, TOO_FAR
        if adc > xs[-1]:
            return None, TOO_CLOSE
        i = 0
        while i < len(xs) - 2 and adc > xs[i + 1]:
            i += 1
        h = xs[i + 1] - xs[i]
        t = (adc - xs[i]) / h
        t2, t3 = t * t, t * t * t
        inv = (ys[i] * (2 * t3 - 3 * t2 + 1) + h * slope[i] * (t3 - 2 * t2 + t) +
               ys[i + 1] * (-2 * t3 + 3 * t2) + h * slope[i + 1] * (t3 - t2))
        if inv <= 0.0:
            return None, TOO_FAR
        return 1.0 / inv, "ok"

    return model


def cmd_table(args):
    """Wypisuje tabelę kalibracyjną w formacie, który przyjmuje laser.xacro."""
    nodes = read_calib_csv(args.table)
    model = make_table_model(nodes)
    spec = " ".join(f"{a:.2f}:{l / 100.0:.2f}" for a, l in nodes)

    print(f"Węzłów: {len(nodes)}   zakres ADC {nodes[0][0]:.2f}-{nodes[-1][0]:.2f}"
          f"  =  {nodes[-1][1] / 100.0:.2f}-{nodes[0][1] / 100.0:.2f} m\n")
    print("laser_calib_table:=\"" + spec + "\"\n")

    print("Rozdzielczość wzdłuż zakresu (ile centymetrów waży jedno zliczenie ADC):")
    for adc in range(int(math.ceil(nodes[0][0])), int(nodes[-1][0]) + 1, 10):
        lo, _ = model(adc - 0.5)
        hi, _ = model(adc + 0.5)
        if lo is not None and hi is not None:
            here, _ = model(adc)
            print(f"  ADC {adc:4d}: {here / 100.0:5.2f} m,  1 tick = {abs(lo - hi):5.2f} cm")

    print("\nWalidacja leave-one-out (dopasowanie bez punktu, sprawdzenie na nim):")
    worst = 0.0
    for i in range(len(nodes)):
        rest = nodes[:i] + nodes[i + 1:]
        if len(rest) < 2:
            continue
        got, status = make_table_model(rest)(nodes[i][0])
        edge = " (brzegowy: to ekstrapolacja)" if i in (0, len(nodes) - 1) else ""
        if got is None:
            print(f"  {nodes[i][1] / 100.0:5.2f} m -> {status}{edge}")
            continue
        err = got - nodes[i][1]
        worst = max(worst, abs(err))
        print(f"  {nodes[i][1] / 100.0:5.2f} m -> {got / 100.0:5.2f} m  ({err:+6.1f} cm){edge}")
    print(f"\nNajgorszy błąd LOO: {worst:.1f} cm")
    print("To jedyna uczciwa miara jakości - przez własne punkty tabela przechodzi\n"
          "dokładnie z definicji, więc zerowy błąd na nich nic nie znaczy.")


def find_port(explicit=None, timeout=4.0):
    """Zwraca pierwszy port, z którego faktycznie lecą ramki LSR."""
    if explicit:
        return explicit
    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if not candidates:
        sys.exit("Nie widzę żadnego /dev/ttyUSB* ani /dev/ttyACM*. Podepnij Nano albo podaj --port.")
    for port in candidates:
        try:
            with serial.Serial(port, 115200, timeout=0.5) as ser:
                # Otwarcie portu resetuje Nano (DTR) - musi zdążyć wstać.
                time.sleep(2.0)
                ser.reset_input_buffer()
                deadline = time.time() + timeout
                while time.time() < deadline:
                    line = ser.readline().decode("ascii", errors="replace").strip()
                    if line.startswith("LSR,") or line.startswith("#LSR"):
                        return port
        except (OSError, serial.SerialException) as exc:
            print(f"  {port}: {exc}", file=sys.stderr)
    sys.exit(f"Żaden z portów {candidates} nie nadaje ramek LSR. Czy Nano ma wgrany laser_nano.ino?")


def open_stream(port):
    ser = serial.Serial(port, 115200, timeout=2.0)
    time.sleep(2.0)          # reset po otwarciu portu
    ser.reset_input_buffer()
    return ser


def frames(ser):
    """Generator (mediana, min, max) z kolejnych poprawnych ramek."""
    while True:
        raw = ser.readline().decode("ascii", errors="replace").strip()
        if not raw or raw.startswith("#"):
            continue
        parts = raw.split(",")
        if len(parts) != 5 or parts[0] != "LSR":
            continue
        try:
            yield int(parts[1]), int(parts[2]), int(parts[3])
        except ValueError:
            continue


def cmd_live(args):
    port = find_port(args.port)

    if args.calib_csv:
        nodes = read_calib_csv(args.calib_csv)
        table = make_table_model(nodes)
        def to_cm(_volts, adc):
            return table(adc)
        print(f"Port: {port}   tabela PCHIP z {args.calib_csv}: {len(nodes)} węzłów, "
              f"{nodes[-1][1] / 100.0:.2f}-{nodes[0][1] / 100.0:.2f} m")
    else:
        def to_cm(volts, _adc):
            return volts_to_cm(volts, args.a, args.b)
        print(f"Port: {port}   model: V = {args.a:.3f}/L_cm + {args.b:.4f}")
        print("UWAGA: model hiperboliczny jest dla tego egzemplarza zły "
              "(myli się o ~26 cm). Podaj --calib-csv, żeby liczyć z tabeli.")
    print(f"{'ADC':>6} {'V':>7} {'odległość':>12} {'szum ADC':>10}")

    log_file = log_writer = None
    if args.log:
        new = not os.path.exists(args.log)
        log_file = open(args.log, "a", newline="")
        log_writer = csv.writer(log_file)
        if new:
            log_writer.writerow(["timestamp", "adc", "adc_min", "adc_max", "volts", "distance_m", "status"])

    try:
        with open_stream(port) as ser:
            for median, lo, hi in frames(ser):
                volts = adc_to_volts(median)
                cm, status = to_cm(volts, median)
                shown = f"{cm / 100.0:9.3f} m" if cm is not None else f"{status:>12}"
                print(f"{median:6d} {volts:7.3f} {shown} {hi - lo:10d}")
                if log_writer:
                    log_writer.writerow([
                        f"{time.time():.3f}", median, lo, hi, f"{volts:.4f}",
                        "" if cm is None else f"{cm / 100.0:.4f}", status,
                    ])
                    log_file.flush()
    except KeyboardInterrupt:
        print("\nKoniec.")
    finally:
        if log_file:
            log_file.close()


def cmd_calib(args):
    """Zbiera jeden punkt kalibracyjny przy ZNANEJ odległości i dopisuje go do CSV."""
    port = find_port(args.port)
    target_m = args.calib
    print(f"Port: {port}")
    print(f"Ustaw cel dokładnie na {target_m:.3f} m i nie ruszaj. Zbieram {args.n} ramek...")

    values = []
    with open_stream(port) as ser:
        for median, _lo, _hi in frames(ser):
            values.append(median)
            print(f"  {len(values):3d}/{args.n}  ADC={median}", end="\r", flush=True)
            if len(values) >= args.n:
                break

    mean = statistics.fmean(values)
    stdev = statistics.stdev(values) if len(values) > 1 else 0.0
    volts = adc_to_volts(mean)
    print(f"\nADC = {mean:.1f} ± {stdev:.1f}   ->  {volts:.4f} V")

    new = not os.path.exists(args.out)
    with open(args.out, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["distance_m", "adc_mean", "adc_stdev", "volts", "n"])
        w.writerow([f"{target_m:.4f}", f"{mean:.2f}", f"{stdev:.2f}", f"{volts:.4f}", len(values)])
    print(f"Dopisano do {args.out}. Zbierz co najmniej 4-5 punktów w zakresie 1-5 m, potem: --fit {args.out}")


def cmd_fit(args):
    """Dopasowuje V = A/L_cm + B metodą najmniejszych kwadratów (regresja liniowa po 1/L)."""
    with open(args.fit, newline="") as f:
        rows = [r for r in csv.DictReader(f)]
    if len(rows) < 2:
        sys.exit(f"{args.fit}: potrzeba co najmniej 2 punktów, jest {len(rows)}.")

    xs = [1.0 / (float(r["distance_m"]) * 100.0) for r in rows]   # 1/L_cm
    ys = [float(r["volts"]) for r in rows]

    n = len(xs)
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        sys.exit("Wszystkie punkty z tej samej odległości - nie ma czego dopasować.")
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    b = my - a * mx

    resid = [y - (a * x + b) for x, y in zip(xs, ys)]
    rms_v = math.sqrt(sum(r * r for r in resid) / n)

    print(f"Punktów: {n}")
    print(f"  A = {a:.4f}")
    print(f"  B = {b:.4f}")
    print(f"  RMS reszt: {rms_v * 1000:.1f} mV")
    print("\nBłąd odtworzenia odległości w punktach kalibracyjnych:")
    worst = 0.0
    for r in rows:
        d_true = float(r["distance_m"])
        cm, status = volts_to_cm(float(r["volts"]), a, b)
        if cm is None:
            print(f"  {d_true:5.2f} m -> {status}")
            continue
        err = cm / 100.0 - d_true
        worst = max(worst, abs(err))
        print(f"  {d_true:5.2f} m -> {cm / 100.0:5.2f} m   ({err * 100:+.1f} cm)")
    print(f"\nNajgorszy błąd: {worst * 100:.1f} cm")
    print(f"\nUżycie: ./scripts/laser_test.py --a {a:.4f} --b {b:.4f}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="Port szeregowy Nano (domyślnie: autowykrywanie po ramkach LSR)")
    p.add_argument("--a", type=float, default=DEFAULT_A, help=f"Stała A modelu V=A/L_cm+B (domyślnie {DEFAULT_A})")
    p.add_argument("--b", type=float, default=DEFAULT_B, help=f"Stała B modelu V=A/L_cm+B (domyślnie {DEFAULT_B})")
    p.add_argument("--log", help="Dopisuj odczyty do pliku CSV")
    p.add_argument("--calib", type=float, metavar="ODLEGŁOŚĆ_M",
                   help="Zbierz punkt kalibracyjny przy podanej znanej odległości")
    p.add_argument("--n", type=int, default=50, help="Ile ramek uśrednić w trybie --calib (domyślnie 50)")
    p.add_argument("--out", default="calib.csv", help="Plik punktów kalibracyjnych dla --calib (domyślnie calib.csv)")
    p.add_argument("--fit", metavar="PLIK_CSV",
                   help="Dopasuj stałe A i B (model hiperboliczny - dla tego egzemplarza zły, "
                        "zostaje do porównań)")
    p.add_argument("--table", metavar="PLIK_CSV",
                   help="Wypisz tabelę kalibracyjną do wklejenia w laser.xacro, z walidacją LOO")
    p.add_argument("--calib-csv", metavar="PLIK_CSV",
                   help="Licz podgląd na żywo z tabeli punktów kalibracyjnych (PCHIP), "
                        "tak jak robi to wtyczka LaserSensor")
    args = p.parse_args()

    if args.table:
        cmd_table(args)
    elif args.fit:
        cmd_fit(args)
    elif args.calib is not None:
        cmd_calib(args)
    else:
        cmd_live(args)


if __name__ == "__main__":
    main()
