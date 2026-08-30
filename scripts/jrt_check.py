#!/usr/bin/env python3
"""
jrt_check.py - kontrola dalmierza JRT po stronie hosta, przez mostek na Nano.

Po co osobny skrypt obok jrt_probe.py: tamten SZUKAŁ modułu (skan baudów, ślepe
strzały). Ten zakłada, że wiadomo już co i jak - baud 38400, PWREN na D4 - i
odpowiada na jedno pytanie: czy tor host -> Nano -> konwerter -> moduł działa i
czy pomiar jest powtarzalny. Dlatego kończy serią strzałów i rozrzutem, a nie
pojedynczą liczbą: jeden odczyt nie odróżnia dobrego montażu od przypadku.

  ./jrt_check.py                  # pełna kontrola, 10 pomiarów
  ./jrt_check.py -n 30            # dłuższa seria
  ./jrt_check.py --baud 19200     # inny baud modułu (musi być w BAUDS mostka)

Ramki do modułu idą przez ESC ESC 'W' <len> - mostek zbiera je w RAM i nadaje
bez przerw. Bez tego SoftwareSerial wstawia luki między bajtami, a moduł gubi
ramkę i milczy; to była jedna z trzech przyczyn ciszy z 2026-08-29.
"""

import argparse
import statistics
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("Brak pyserial. Zainstaluj: pip install pyserial")

BRIDGE_BAUDS = [19200, 9600, 38400, 57600, 4800, 2400, 14400, 115200]
ESC = b"\x1b\x1b"

WAKE = bytes([0x55])
STOP = bytes([0x58])

STATUS = {
    0x0000: "brak bledu", 0x0001: "za niskie zasilanie (<2,2 V)",
    0x0002: "blad wewnetrzny (do zignorowania)", 0x0003: "za zimno (<-20 C)",
    0x0004: "za goraco (>+40 C)", 0x0005: "cel poza zasiegiem",
    0x0006: "wynik nieprawidlowy", 0x0007: "za silne swiatlo tla",
    0x0008: "sygnal lasera za slaby", 0x0009: "sygnal lasera za silny",
    0x000A: "usterka sprzetowa 1", 0x000B: "usterka sprzetowa 2",
    0x000C: "usterka sprzetowa 3", 0x000D: "usterka sprzetowa 4",
    0x000E: "usterka sprzetowa 5", 0x000F: "sygnal lasera niestabilny",
    0x0010: "usterka sprzetowa 6", 0x0011: "usterka sprzetowa 7",
    0x0081: "ramka nieprawidlowa",
}


def frame(payload: bytes) -> bytes:
    """Suma kontrolna liczona z bajtow PO naglowku 0xAA, przepelnienie ignorowane."""
    return bytes([0xAA]) + payload + bytes([sum(payload) & 0xFF])


READ_STATUS = frame(bytes([0x80, 0x00, 0x00]))
READ_VOLT   = frame(bytes([0x80, 0x00, 0x06]))
READ_HW     = frame(bytes([0x80, 0x00, 0x0A]))
READ_SW     = frame(bytes([0x80, 0x00, 0x0C]))
READ_SN     = frame(bytes([0x80, 0x00, 0x0E]))
MEAS_SLOW   = frame(bytes([0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x01]))
MEAS_FAST   = frame(bytes([0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x02]))
LASER_OFF   = frame(bytes([0x00, 0x01, 0xBE, 0x00, 0x01, 0x00, 0x00]))


def hx(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b) if b else "(cisza)"


def payload_of(resp: bytes, reg: int):
    """Wyluskuje payload ramki odpowiedzi dla danego rejestru; None gdy nie ma."""
    for i in range(len(resp) - 5):
        if resp[i] != 0xAA:
            continue
        if (resp[i + 2] << 8 | resp[i + 3]) != reg:
            continue
        words = resp[i + 4] << 8 | resp[i + 5]
        end = i + 6 + 2 * words
        if end <= len(resp):
            return resp[i + 6:end]
    return None


def err_of(resp: bytes):
    """Ramka bledu zaczyna sie 0xEE zamiast 0xAA."""
    for i in range(len(resp) - 7):
        if resp[i] == 0xEE:
            return resp[i + 6] << 8 | resp[i + 7]
    return None


class Bridge:
    def __init__(self, port, mod_baud):
        self.s = serial.Serial(port, 115200, timeout=0.5)
        time.sleep(2.2)                    # DTR zresetowal Nano - czekamy na mostek
        self.banner = self.s.read(self.s.in_waiting or 1)
        self.set_baud(mod_baud)

    def cmd(self, body, wait=0.4):
        self.s.reset_input_buffer()
        self.s.write(ESC + body)
        self.s.flush()
        time.sleep(wait)
        return self.s.read(self.s.in_waiting or 1)

    def set_baud(self, baud):
        if baud not in BRIDGE_BAUDS:
            sys.exit(f"Baud {baud} nie jest w tabeli mostka: {BRIDGE_BAUDS}")
        return self.cmd(b"\x4b" + bytes([BRIDGE_BAUDS.index(baud)]))

    def power(self, on):
        return self.cmd(b"\x50" + bytes([1 if on else 0]), wait=0.6)

    def send(self, data, reg=None, timeout=3.0):
        """Ramka do modulu przez bufor mostka - nadana bez przerw miedzy bajtami.

        Czeka DO KOMPLETNEJ RAMKI, nie przez staly czas. Pomiar slow potrafi
        trwac ponad sekunde i bywa, ze modul odzywa sie dopiero po sztywnym
        oknie - wtedy odpowiedz ginela przy czyszczeniu bufora pod nastepna
        komende i wygladalo to na losowe gubienie co drugiego strzalu.
        """
        self.s.reset_input_buffer()
        self.s.write(ESC + b"\x57" + bytes([len(data)]) + data)
        self.s.flush()
        deadline = time.monotonic() + timeout
        buf = b""
        while time.monotonic() < deadline:
            buf += self.s.read(self.s.in_waiting or 1)
            if reg is not None and payload_of(buf, reg) is not None:
                break
            if err_of(buf) is not None:
                break
            time.sleep(0.02)
        return buf

    def raw(self, data, wait=0.3):
        self.s.reset_input_buffer()
        self.s.write(data)
        self.s.flush()
        time.sleep(wait)
        return self.s.read(self.s.in_waiting or 1)

    def close(self):
        self.s.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=38400, help="baud modulu (domyslnie 38400)")
    ap.add_argument("-n", "--count", type=int, default=10, help="ile pomiarow w serii")
    ap.add_argument("--fast", action="store_true",
                    help="tryb fast zamiast slow: ~1,9 Hz zamiast ~0,3 Hz, ta sama "
                         "powtarzalnosc na twardym celu")
    args = ap.parse_args()

    b = Bridge(args.port, args.baud)
    print(f"mostek : {b.banner.decode(errors='replace').strip() or '(brak bannera)'}")
    print(f"port   : {args.port} @115200, modul @{args.baud}")

    print(f"PWREN  : {b.power(True).decode(errors='replace').strip() or '(brak potwierdzenia)'}")

    addr = b.raw(WAKE)                     # auto-baud: modul odsyla swoj adres
    print(f"auto-baud (0x55) -> {hx(addr)}"
          + (f"   adres modulu 0x{addr[-1]:02X}" if addr else "   BRAK ODPOWIEDZI"))
    b.raw(STOP)                            # gdyby zostal w trybie ciaglym

    if not addr:
        print("\n>>> modul nie odpowiada na auto-baud. Kolejnosc sprawdzania:\n"
              "    1. PWREN faktycznie wysoki na pinie modulu (zmierz, nie zakladaj)\n"
              "    2. VCC modulu 2,5-3,3 V pod obciazeniem\n"
              "    3. TXD modulu -> D3, RXD modulu <- D2 (nie odwrotnie)\n"
              "    4. konwerter poziomow zasilany z obu stron (5 V i 3,3 V)\n"
              "    5. inny baud: --baud 19200")
        b.close()
        return 2

    ident = [
        ("napiecie", READ_VOLT,   0x0006),
        ("HW      ", READ_HW,     0x000A),
        ("SW      ", READ_SW,     0x000C),
        ("nr ser. ", READ_SN,     0x000E),
    ]
    print()
    for label, cmd, reg in ident:
        resp = b.send(cmd, reg, timeout=1.5)
        pay = payload_of(resp, reg)
        extra = ""
        if pay and reg == 0x0000:
            code = int.from_bytes(pay, "big")
            extra = f"   -> {STATUS.get(code, 'kod nieznany')}"
        elif pay and reg == 0x0006:
            # Rejestr jest w BCD, nie w miliwoltach: 0x3179 = 3,179 V. Ustalone
            # 2026-08-29 przez porownanie z miernikiem, ktory pokazal 3,18 V.
            bcd = "".join(f"{x:02X}" for x in pay)
            extra = (f"   -> {int(bcd[0])},{bcd[1:]} V" if bcd.isdigit()
                     else "   -> (nie BCD, surowo)")
        print(f"  {label} {hx(pay) if pay else hx(resp)}{extra}")

    meas = MEAS_FAST if args.fast else MEAS_SLOW
    print(f"\n-- seria {args.count} pomiarow (1-shot {'fast' if args.fast else 'slow'}) --")
    t_start = time.monotonic()
    good, dists = 0, []
    for i in range(args.count):
        resp = b.send(meas, 0x0022, timeout=4.0)
        pay = payload_of(resp, 0x0022)
        if pay and len(pay) >= 6:
            mm = int.from_bytes(pay[0:4], "big")
            sq = int.from_bytes(pay[4:6], "big")
            dists.append(mm)
            good += 1
            print(f"  {i+1:3d}. {mm:6d} mm  ({mm/1000:7.3f} m)   SQ={sq}")
        else:
            code = err_of(resp)
            why = STATUS.get(code, f"kod 0x{code:04X}") if code is not None else hx(resp)
            print(f"  {i+1:3d}. BLAD: {why}")

    elapsed = time.monotonic() - t_start

    # Status czytamy PO serii: tuz po surowym 0x55/0x58 modul melduje 0x0081
    # ("ramka nieprawidlowa"), bo te bajty nie sa ramkami - i wyglada to na
    # usterke, ktora usterka nie jest. Po pomiarze kod dotyczy juz pomiaru.
    st = payload_of(b.send(READ_STATUS, 0x0000, timeout=1.5), 0x0000)
    if st is not None:
        code = int.from_bytes(st, "big")
        print(f"\nstatus po serii: {hx(st)} -> {STATUS.get(code, 'kod nieznany')}")

    b.send(LASER_OFF, 0x01BE, timeout=1.0)                      # nie zostawiamy swiecacej diody
    b.close()

    print(f"udane: {good}/{args.count} w {elapsed:.1f} s ({args.count/elapsed:.2f} Hz)")
    if len(dists) >= 2:
        sd = statistics.pstdev(dists)
        print(f"srednia {statistics.mean(dists):.1f} mm, "
              f"rozrzut {min(dists)}..{max(dists)} mm (p-p {max(dists)-min(dists)}), "
              f"odch. std {sd:.2f} mm")
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
