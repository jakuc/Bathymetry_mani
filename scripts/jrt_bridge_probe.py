#!/usr/bin/env python3
"""Odpytanie dalmierza JRT przez mostek na Nano (jrt_bridge.ino)."""
import serial, sys, time

BAUDS = [19200, 9600, 38400, 57600, 4800, 2400, 14400, 115200]
ESC = b"\x1b\x1b"

def frame(p): return bytes([0xAA]) + p + bytes([sum(p) & 0xFF])

QUERIES = [
    ("status",        frame(bytes([0x80, 0x00, 0x00]))),
    ("napiecie",      frame(bytes([0x80, 0x00, 0x06]))),
    ("wersja HW",     frame(bytes([0x80, 0x00, 0x0A]))),
    ("wersja SW",     frame(bytes([0x80, 0x00, 0x0C]))),
    ("NUMER SERYJNY", frame(bytes([0x80, 0x00, 0x0E]))),
]
MEASURE = frame(bytes([0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x01]))

def hx(b): return " ".join(f"{x:02X}" for x in b) or "(cisza)"

def drain(s, t=0.3):
    time.sleep(t)
    return s.read(s.in_waiting or 1) if s.in_waiting else b""

s = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.5)
time.sleep(2.2)                                  # reset Nano + rozruch mostka
banner = s.read(s.in_waiting or 1)
print("banner mostka:", banner.decode(errors="replace").strip() or "(brak)")
print()

found = False
for idx, baud in enumerate(BAUDS):
    s.reset_input_buffer()
    s.write(ESC + b"\x4b" + bytes([idx])); s.flush()
    ack = drain(s, 0.4)
    print(f"=== modul @ {baud} bps ===  ({ack.decode(errors='replace').strip()})")

    s.write(ESC + b"\x52"); s.flush()            # czysty reset modulu
    drain(s, 0.6)

    s.reset_input_buffer()
    s.write(b"\x55"); s.flush(); time.sleep(0.25)  # wybudzenie / auto-baud
    s.reset_input_buffer()

    for label, cmd in QUERIES:
        s.write(cmd); s.flush()
        r = drain(s, 0.45)
        mark = "<<<" if r else "   "
        print(f"  {mark} {label:14s} {hx(cmd):26s} | {hx(r)}")
        if r:
            found = True
    if found:
        print("\n  -- POMIAR --")
        s.write(MEASURE); s.flush()
        r = drain(s, 1.5)
        print(f"      strzal {hx(MEASURE)} | {hx(r)}")
        break
    print()

s.close()
print()
print(">>>", "ODPOWIEDZIAL" if found else "cisza na wszystkich baudach modulu")
