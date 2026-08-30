#!/usr/bin/env python3
"""
jrt_probe.py - identyfikacja i pierwszy strzal modulu JRT (naklejka LDB1 A191).

Po co: naklejka nie mowi, ktory to model. Modul mowi - ma komendy na wersje
sprzetu, wersje firmware i numer seryjny. Ten skrypt je odpytuje, a jak nie
dostanie odpowiedzi na nominalnym 19200, przelatuje pozostale baudy zamiast
kazac zgadywac.

  ./jrt_probe.py                  # autodetekcja portu, skan baudow
  ./jrt_probe.py --port /dev/ttyUSB0 --baud 19200
  ./jrt_probe.py --measure        # dodatkowo pojedynczy pomiar

Protokol: ramka AA <payload...> <suma>, gdzie suma = (suma bajtow po AA) & 0xFF.
"""

import argparse
import glob
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("Brak pyserial. Zainstaluj: pip install pyserial")

# Baudy w kolejnosci prawdopodobienstwa: 19200 to nominal dla linii M703A/M88/U81.
BAUDS = [19200, 115200, 9600, 38400, 57600]

WAKE = bytes([0x55])          # wybudzenie / auto-baud
STOP = bytes([0x58])          # stop pomiaru ciaglego


def frame(payload: bytes) -> bytes:
    """Domyka ramke suma kontrolna - liczona z bajtow PO naglowku 0xAA."""
    return bytes([0xAA]) + payload + bytes([sum(payload) & 0xFF])


QUERIES = [
    ("status modulu",   frame(bytes([0x80, 0x00, 0x00]))),
    ("napiecie zasil.", frame(bytes([0x80, 0x00, 0x06]))),
    ("wersja HW",       frame(bytes([0x80, 0x00, 0x0A]))),
    ("wersja SW",       frame(bytes([0x80, 0x00, 0x0C]))),
    ("numer seryjny",   frame(bytes([0x80, 0x00, 0x0E]))),
]

MEASURE = frame(bytes([0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x01]))
READ_RESULT = frame(bytes([0x80, 0x00, 0x22]))


def hexs(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


def exchange(ser, cmd: bytes, wait: float = 0.35) -> bytes:
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()
    time.sleep(wait)
    return ser.read(ser.in_waiting or 1)


def probe(port: str, baud: float, measure: bool) -> bool:
    """Zwraca True, jesli modul odpowiedzial czymkolwiek sensownym."""
    with serial.Serial(port, baud, timeout=0.4) as ser:
        time.sleep(0.15)
        ser.write(WAKE)          # auto-baud: modul dostraja sie do tego bajtu
        ser.flush()
        time.sleep(0.25)
        ser.write(STOP)          # gdyby zostal w trybie ciaglym po poprzednim tescie
        ser.flush()
        time.sleep(0.2)
        ser.reset_input_buffer()

        answered = False
        for label, cmd in QUERIES:
            resp = exchange(ser, cmd)
            mark = "  " if resp else "--"
            print(f"  {mark} {label:16s} -> {hexs(cmd):28s} | {hexs(resp) or '(cisza)'}")
            if resp:
                answered = True

        if measure and answered:
            print("  -- pomiar --")
            resp = exchange(ser, MEASURE, wait=1.2)
            print(f"     strzal            -> {hexs(MEASURE)} | {hexs(resp) or '(cisza)'}")
            if len(resp) >= 10:
                # Dystans siedzi w bajtach 8-9 ramki odpowiedzi, big-endian, w mm.
                mm = (resp[7] << 8) | resp[8]
                print(f"     dystans ~ {mm} mm  ({mm / 1000:.3f} m)")
            resp = exchange(ser, READ_RESULT)
            print(f"     odczyt wyniku     -> {hexs(READ_RESULT)} | {hexs(resp) or '(cisza)'}")

        return answered


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="np. /dev/ttyUSB0 (domyslnie: pierwszy znaleziony)")
    ap.add_argument("--baud", type=int, help="wymus jeden baud zamiast skanu")
    ap.add_argument("--measure", action="store_true", help="dodaj pojedynczy pomiar")
    args = ap.parse_args()

    port = args.port
    if not port:
        found = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        if not found:
            return print("Nie widze /dev/ttyUSB* ani /dev/ttyACM*. Podepnij FT232RL "
                         "albo podaj --port.", file=sys.stderr) or 1
        port = found[0]
        print(f"Port: {port} (z {len(found)} znalezionych: {', '.join(found)})")

    bauds = [args.baud] if args.baud else BAUDS
    for baud in bauds:
        print(f"\n=== {baud} bps 8N1 ===")
        try:
            if probe(port, baud, args.measure):
                print(f"\nODPOWIEDZIAL na {baud} bps. Numer seryjny z linii wyzej -> "
                      f"to nim JRT ustali model.")
                return 0
        except serial.SerialException as exc:
            return print(f"Port {port}: {exc}", file=sys.stderr) or 1

    print("\nCisza na wszystkich baudach. Kolejnosc sprawdzania:\n"
          "  1. zworka FT232RL na 3,3 V (nie 5 V)\n"
          "  2. PWREN podciagniety do 3,3 V - bez tego modul jest WYLACZONY\n"
          "  3. pull-upy 10k ida do 3,3 V, nie do GND\n"
          "  4. TXD modulu -> RXD konwertera (nie TXD do TXD)\n"
          "  5. napiecie na VCC modulu pod obciazeniem - FT232RL daje ~50 mA")
    return 2


if __name__ == "__main__":
    sys.exit(main())
