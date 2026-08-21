# Backup oryginalnej zawartości Arduino Nano (2026-08-12)

Zrzut szkica, który był wgrany na Nano dalmierza Sharp GP2Y0A710K0F **zanim**
podmieniliśmy go na `../laser_nano.ino`. Zrobiony po to, żeby dało się wrócić
do stanu sprzed zmiany — patrz `../przywroc_oryginal.sh`.

## Układ

| | |
|---|---|
| MCU | ATmega328P (sygnatura `0x1e950f`) |
| Bootloader | Optiboot, HW 3 / FW 4.4, 512 B pod `0x7E00` |
| Prędkość bootloadera | **115200** (nie 57600 — mimo że to klon z CH340) |
| Port na płytce | `/dev/ttyUSB0` |

Rozkład flasha (32768 B, zajęte 4342 B = 13,3%):

```
0x0000 - 0x0F57   3928 B   szkic
0x7E00 - 0x7FFF    512 B   Optiboot
```

## Pliki

| plik | do czego |
|---|---|
| `flash_app.hex` | **obraz do przywracania** — sama aplikacja, `0x0000-0x0F57` |
| `flash_backup.hex` | pełny zrzut 32 KB w Intel HEX (referencja, nie do wgrywania) |
| `flash_backup.bin` | ten sam zrzut surowo, do porównań bajt po bajcie |
| `device_info.txt` | sygnatura i wersje bootloadera prosto z avrdude |
| `bootloader_baud.txt` | wykryta prędkość bootloadera |

Do przywracania służy `flash_app.hex`, a nie pełny `flash_backup.hex`: ten drugi
obejmuje obszar bootloadera, którego Optiboot nie potrafi nadpisać sam sobą —
zapis przechodzi, ale weryfikacja się wywala. Bootloader przy wgrywaniu szkica
i tak zostaje nietknięty, więc obraz samej aplikacji w zupełności wystarcza.

## Czym ten backup jest sprawdzony

Sam fakt istnienia pliku nie znaczy, że kopia jest wierna — dlatego dwa dowody:

1. **Dwa niezależne odczyty flasha** wyszły bajt w bajt identyczne (`cmp`), więc
   transmisja przez bootloader niczego nie przekłamała.
2. **`avrdude -U flash:v:flash_app.hex:i`** — 3928 bajtów zweryfikowanych
   zgodnie z zawartością żywej kości. Obraz przywracania jest porównany ze
   sprzętem, nie tylko zapisany na dysk.

## Czego tu NIE ma

**EEPROM-u.** Optiboot 4.4 nie implementuje dostępu do EEPROM-u i na żądanie
odczytu zwraca zawartość flasha — bez żadnego błędu. `avrdude` melduje wtedy
sukces, a plik wygląda wiarygodnie: 1024 bajty, „zapisanych" 1010. W praktyce
był to bajt w bajt pierwszy kilobajt flasha (zweryfikowane porównaniem), więc
plik został skasowany, bo backup, który kłamie, jest gorszy niż jego brak.

Odtwarzalności to nie psuje: wgranie szkica przez bootloader **nie rusza
EEPROM-u**, więc cokolwiek tam siedzi, przeżyje podmianę firmware'u. Gdyby
kiedyś naprawdę trzeba było zrzucić EEPROM, wymaga to programatora ISP —
przez ten bootloader się nie da.

**Źródła.** To zrzut binarny; `.ino` się z niego nie odtworzy. Do przywrócenia
działania wystarcza, do czytania kodu nie.

## Jak rozpoznać, że oryginał wrócił

Oryginalny szkic nadaje na 115200 **gołe liczby ADC, ~2 Hz** (bez żadnego
prefiksu). Nasz `laser_nano.ino` nadaje ramki `LSR,<mediana>,<min>,<max>,<n>`
z częstotliwością 20 Hz — pomylić się nie da. `przywroc_oryginal.sh` sprawdza
to sam po wgraniu.
