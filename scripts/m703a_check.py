#!/usr/bin/env python3
"""Kontrola dalmierza JRT M703A przez mostek Nano (firmware/jrt_bridge) - tryb
ciągły ZE STEMPLAMI CZASU z Nano.

Co mierzy i po co:
  * tempo i odstępy pomiarów WEDŁUG ZEGARA NANO - to jest prawdziwy rytm
    modułu, bez jittera USB;
  * rozrzut odległości i SQ - czy tor mierzy poprawnie;
  * czas_hosta - stempel_Nano: dolna obwiednia daje przesunięcie i DRYF zegara
    Nano, a rozrzut nad nią to jitter USB/systemu, który stempel wycina.
    To jest liczba, dla której ten tryb w ogóle istnieje;
  * t_last - t_first: czas nadawania linii; ~0,52 ms/bajt przy 19200, inna
    wartość = poszarpana linia.

Uwaga: otwarcie portu resetuje Nano (DTR), stąd pauza na baner.

  python3 scripts/m703a_check.py                # F, 30 s
  python3 scripts/m703a_check.py --mode D -s 20
"""
import argparse
import re
import statistics as st
import time
from collections import Counter

import numpy as np
import serial

LINE_RE = re.compile(r"^\$(\d+),(\d+),(.*)$")
RANGE_RE = re.compile(r"([\d.]+)m,(\d+)")
WRAP = 1 << 32


def read_for(ser, seconds):
    """Zbiera linie razem z czasem hosta przyjścia końca linii."""
    out, buf = [], b""
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        now = time.monotonic()
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            out.append((now, line.rstrip(b"\r").decode(errors="replace")))
    return out


def ctrl(ser, cmd, arg=None):
    ser.write(b"\x1b\x1b" + cmd + (bytes([arg]) if arg is not None else b""))


def lower_envelope_fit(x, y, iters=6):
    """Prosta pod chmurą punktów: dopasowanie iterowane do najniższych reszt.
    Opóźnienie USB jest zawsze >= minimum, więc tylko dolna obwiednia niesie
    relację zegarów - średnia byłaby przesunięta o średni jitter."""
    mask = np.ones_like(x, dtype=bool)
    for _ in range(iters):
        a, b = np.polyfit(x[mask], y[mask], 1)
        r = y - (a * x + b)
        mask = r <= np.percentile(r, 20)
    a, b = np.polyfit(x[mask], y[mask], 1)
    r = y - (a * x + b)
    return a, b - r.min(), r - r.min()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--mode", default="F", choices=["F", "D", "M"])
    ap.add_argument("-s", "--seconds", type=float, default=30.0)
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.002)
    time.sleep(2.5)
    print("baner:", ser.read(500).decode(errors="replace").strip())

    ctrl(ser, b"T", 1)
    ser.write(b"S")
    for _, l in read_for(ser, 0.6):
        print("  ", l)

    ctrl(ser, b"N", 0)                 # nCTRL nisko = ciągły
    time.sleep(0.1)
    ser.reset_input_buffer()
    ser.write(args.mode.encode())
    lines = read_for(ser, args.seconds)
    ctrl(ser, b"N", 1)
    time.sleep(0.4)
    ser.write(b"S")
    tail = read_for(ser, 0.6)
    ser.close()

    rows = []
    for host_t, l in lines:
        m = LINE_RE.match(l)
        if m:
            rows.append((host_t, int(m.group(1)), int(m.group(2)), m.group(3)))
    # Pierwsza linia po komendzie jest ROZCIĄGNIĘTA: moduł od razu odsyła literę
    # komendy, a resztę dopiero po pierwszym pomiarze (zmierzone: ~300 ms).
    # Jej t_first to chwila przyjęcia komendy, nie pomiaru - do statystyk się nie nadaje.
    rows = rows[1:]
    if len(rows) < 5:
        print("za malo linii ze stemplem:", lines[:10])
        return

    # Rozwinięcie uint32 mikrosekund.
    first, last, epoch, prev = [], [], 0, rows[0][1]
    for _, tf, tl, _ in rows:
        if tf < prev - WRAP // 2:
            epoch += WRAP
        prev = tf
        first.append(tf + epoch)
        last.append(tl + epoch if tl >= tf else tl + epoch + WRAP)
    first = np.array(first, dtype=np.int64)
    last = np.array(last, dtype=np.int64)
    host = np.array([r[0] for r in rows])

    ok = [(float(m.group(1)), int(m.group(2))) for r in rows for m in [RANGE_RE.search(r[3])] if m]
    errs = Counter(r[3] for r in rows if "Er" in r[3])
    span = (first[-1] - first[0]) / 1e6
    print(f"\n{args.mode}: {len(rows)} linii w {span:.1f} s -> {(len(rows) - 1) / span:.2f} Hz,"
          f" poprawnych {len(ok)}, bledy {dict(errs)}")
    if ok:
        r = [o[0] for o in ok]
        q = [o[1] for o in ok]
        print(f"odleglosc: srednia {st.mean(r):.4f} m, std {st.pstdev(r) * 1e3:.2f} mm,"
              f" p-p {(max(r) - min(r)) * 1e3:.0f} mm, SQ {min(q)}-{max(q)}")

    dt = np.diff(first) / 1e3
    print("odstepy wg zegara Nano [ms]:", sorted(Counter(np.round(dt, 0).astype(int)).items()))
    dur = (last - first) / 1e3
    print(f"nadawanie linii t_last-t_first: {dur.min():.2f}-{dur.max():.2f} ms"
          f" (linia {len(rows[1][3]) + 2} B; od 1. do ostatniego bajtu ~{(len(rows[1][3]) + 1) * 0.521:.2f} ms)")

    # Relacja zegarów: host (s) vs t_last Nano (s).
    x = (last - last[0]) / 1e6
    y = host - host[0]
    a, _, resid = lower_envelope_fit(x, y)
    resid_ms = resid * 1e3
    print(f"dryf zegara Nano wzgledem PC: {(a - 1) * 1e6:+.0f} ppm"
          f" ({(a - 1) * 1e3:+.3f} ms na sekunde)")
    print(f"jitter przyjscia na PC ponad obwiednia: mediana {np.median(resid_ms):.2f} ms,"
          f" p95 {np.percentile(resid_ms, 95):.2f} ms, max {resid_ms.max():.2f} ms")
    print("  ^ to wycina stempel z Nano")
    print("po serii:", [l for _, l in tail])


if __name__ == "__main__":
    main()
