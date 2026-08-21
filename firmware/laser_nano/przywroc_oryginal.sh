#!/usr/bin/env bash
# przywroc_oryginal.sh - wgranie z powrotem szkica, który był na Arduino Nano
# przed podmianą na laser_nano.ino. Uruchamiane NA PŁYTCE bathset.
#
#   scp -r firmware/laser_nano ubuntu@10.42.0.73:~/
#   ssh ubuntu@10.42.0.73 '~/laser_nano/przywroc_oryginal.sh'
#
# Wgrywamy flash_app.hex (0x0000-0x0F57), a NIE flash_backup.hex. Pełny zrzut
# obejmuje obszar bootloadera pod 0x7E00, a Optiboot nie potrafi nadpisać sam
# siebie - zapis takiego pliku przechodzi, ale wywala się na weryfikacji.
# Bootloader i tak zostaje nietknięty, więc obraz samej aplikacji wystarcza.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${PORT:-/dev/ttyUSB0}"
MCU="${MCU:-atmega328p}"
BAUD="${BAUD:-115200}"
IMG="$HERE/backup_oryginalny/flash_app.hex"

[ -f "$IMG" ] || { echo "BLAD: brak $IMG" >&2; exit 1; }

echo "== wgrywanie $IMG na $PORT =="
avrdude -c arduino -p "$MCU" -P "$PORT" -b "$BAUD" -U flash:w:"$IMG":i

echo "== weryfikacja =="
avrdude -c arduino -p "$MCU" -P "$PORT" -b "$BAUD" -U flash:v:"$IMG":i

# Dowód na to, że wróciło działanie, a nie tylko bajty: oryginalny szkic nadaje
# gołe liczby ADC na 115200, ~2 Hz.
echo "== nasłuch 6 s (spodziewane: gołe liczby, ~2 Hz) =="
python3 - "$PORT" <<'PY'
import serial, sys, time
with serial.Serial(sys.argv[1], 115200, timeout=1) as s:
    time.sleep(2.2)
    s.reset_input_buffer()
    t0 = time.time()
    lines = []
    while time.time() - t0 < 6:
        ln = s.readline().decode("ascii", errors="replace").strip()
        if ln:
            lines.append(ln)
    print(f"linii: {len(lines)}  ({len(lines)/6.0:.1f} Hz)")
    print("probka:", lines[:8])
PY
