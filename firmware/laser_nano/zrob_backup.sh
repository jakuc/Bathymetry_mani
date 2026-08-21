#!/usr/bin/env bash
# Zrzut zawartości Arduino Nano do plików odtwarzalnych przez avrdude.
# Uruchamiane NA PŁYTCE (Nano wisi na jej /dev/ttyUSB0).
set -uo pipefail

PORT="${PORT:-/dev/ttyUSB0}"
MCU="${MCU:-atmega328p}"
OUT="${OUT:-$HOME/nano_backup}"
mkdir -p "$OUT"

# Klony Nano z CH340 chodzą albo na starym bootloaderze (57600), albo na nowym
# (115200). Zamiast zgadywać - próbujemy obu i zapamiętujemy ten, który odpowie.
# Uwaga: wynik avrdude idzie do PLIKU, nie do potoku z grepem. Przy `grep -q`
# grep kończy się po pierwszym trafieniu, avrdude dostaje SIGPIPE i zwraca 141,
# a `set -o pipefail` propaguje właśnie ten kod - udana detekcja wyglądałaby
# wtedy na nieudaną.
BAUD=""
for b in 115200 57600; do
  echo "== proba bootloadera @ ${b} =="
  avrdude -c arduino -p "$MCU" -P "$PORT" -b "$b" -v > "$OUT/probe_${b}.log" 2>&1
  if grep -q "Device signature" "$OUT/probe_${b}.log"; then
    BAUD="$b"
    echo "== bootloader odpowiada na ${b} =="
    break
  fi
done

if [ -z "$BAUD" ]; then
  echo "BLAD: bootloader nie odpowiada ani na 115200, ani na 57600." >&2
  exit 1
fi

echo "$BAUD" > "$OUT/bootloader_baud.txt"

echo "== sygnatura ukladu =="
avrdude -c arduino -p "$MCU" -P "$PORT" -b "$BAUD" -v 2>&1 | grep -iE "Device signature|hardware version|firmware version" | tee "$OUT/device_info.txt"

# Flash w dwóch postaciach: .hex do wgrania z powrotem, .bin do porównań bajt po bajcie.
echo "== odczyt flash -> hex =="
avrdude -c arduino -p "$MCU" -P "$PORT" -b "$BAUD" -U flash:r:"$OUT/flash_backup.hex":i 2>&1 | tail -6
echo "== odczyt flash -> bin =="
avrdude -c arduino -p "$MCU" -P "$PORT" -b "$BAUD" -U flash:r:"$OUT/flash_backup.bin":r 2>&1 | tail -6
# EEPROM-u CELOWO nie zrzucamy. Optiboot 4.4 nie implementuje dostępu do
# EEPROM-u i na żądanie odczytu zwraca zawartość flasha - bez błędu, a avrdude
# melduje sukces. Powstaje plik, który wygląda wiarygodnie i jest kopią
# pierwszego kilobajta flasha (sprawdzone 2026-08-12). Taki backup jest gorszy
# niż jego brak, bo się na nim polega.
# Realny zrzut EEPROM-u wymaga programatora ISP. Nie jest potrzebny do
# odtworzenia stanu: wgranie szkica przez bootloader EEPROM-u nie rusza.

# Drugi, niezależny odczyt flash. Jeśli oba zrzuty są identyczne, transmisja
# przez bootloader nie przekłamała - to jedyny tani dowód, że backup jest
# wierny. Bez tego "mam plik" nie znaczy jeszcze "mam kopię".
echo "== kontrolny drugi odczyt flash =="
avrdude -c arduino -p "$MCU" -P "$PORT" -b "$BAUD" -U flash:r:"$OUT/flash_verify.bin":r 2>&1 | tail -6

echo "== porownanie dwoch odczytow =="
if cmp -s "$OUT/flash_backup.bin" "$OUT/flash_verify.bin"; then
  echo "ZGODNE - oba odczyty identyczne"
  rm -f "$OUT/flash_verify.bin"
else
  echo "ROZNE! zrzut niewiarygodny - NIE nadpisuj flasha" >&2
fi

echo "== obraz do przywracania + zajetosc flasha =="
# 0xFF = komórka nigdy nie zapisana. Licząc bajty różne od 0xFF widzimy, ile
# realnie zajmuje szkic + bootloader - zrzut samych 0xFF oznaczałby pustą kość.
#
# Z pełnego zrzutu wycinamy obszar aplikacji (poniżej 0x7E00) do osobnego
# flash_app.hex. To jego, a nie pełnego zrzutu, używa przywracanie: pełny plik
# obejmuje bootloader, którego Optiboot nie potrafi nadpisać sam sobą - zapis
# przechodzi, ale weryfikacja się wywala.
python3 - "$OUT/flash_backup.bin" "$OUT/flash_app.hex" <<'PY'
import sys

BOOT_START = 0x7E00   # Optiboot 512 B na końcu 32 KB flasha

data = open(sys.argv[1], "rb").read()
used = sum(1 for b in data if b != 0xFF)
print(f"rozmiar zrzutu: {len(data)} B")
print(f"bajty != 0xFF : {used} B  ({100.0 * used / len(data):.1f}%)")

app = data[:BOOT_START]
end = len(app)
while end > 0 and app[end - 1] == 0xFF:   # nie zapisujemy pustych stron
    end -= 1
app = app[:end]

lines = []
for off in range(0, len(app), 16):
    chunk = app[off:off + 16]
    rec = [len(chunk), (off >> 8) & 0xFF, off & 0xFF, 0x00] + list(chunk)
    lines.append(":" + "".join("%02X" % b for b in rec) + "%02X" % (((~sum(rec)) + 1) & 0xFF))
lines.append(":00000001FF")
open(sys.argv[2], "w").write("\n".join(lines) + "\n")
print(f"aplikacja: 0x0000 - 0x{end - 1:04X} ({end} B) -> flash_app.hex")
PY

# Ostateczny dowód: obraz przywracania porównany z żywą kością. Dopiero to
# odróżnia "mam plik" od "mam kopię, z której da się wrócić".
echo "== weryfikacja obrazu przywracania wobec ukladu =="
avrdude -c arduino -p "$MCU" -P "$PORT" -b "$BAUD" -U flash:v:"$OUT/flash_app.hex":i 2>&1 | grep -iE "verified|mismatch|error" || true

echo "== pliki =="
ls -l "$OUT"
sha256sum "$OUT"/* 2>/dev/null
