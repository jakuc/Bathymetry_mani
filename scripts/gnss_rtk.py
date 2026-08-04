#!/usr/bin/env python3
"""Konfiguracja RTK (klient NTRIP) w odbiorniku Septentrio mosaic-H.

Korekcje pobiera SAM ODBIORNIK swoim wbudowanym klientem NTRIP - nie ma po
stronie ROS-a żadnego procesu pośredniczącego. Konsekwencje:
  * ustawienia siedzą w pamięci nieulotnej odbiornika (zapisujemy je do profilu
    Boot), więc konfiguruje się je RAZ, a nie przy każdym uruchomieniu stacku;
  * login i hasło do serwisu RTK nigdy nie trafiają do repozytorium ani do URDF;
  * kod ROS-a się nie zmienia - fix po prostu wchodzi w tryb 4 (RTK fixed),
    a GnssSensor tylko obserwuje status połączenia (zdanie $PSSN,SNC).

Odbiornik potrzebuje internetu, który dostaje od hosta przez ethernet-over-USB:
najpierw `sudo ./scripts/gnss_internet_share.sh up`, potem dopiero `configure`.

Domyślnie rozmawiamy przez /dev/gnss_aux (drugi port CDC odbiornika), bo
/dev/gnss trzyma wtyczka GnssSensor - dzięki temu można konfigurować RTK
przy DZIAŁAJĄCYM stacku ROS, bez zatrzymywania pomiaru.

Poświadczenia bierzemy ze zmiennych środowiskowych albo z pliku
~/.config/mani_ros/rtk.env (format KEY=VALUE, jedna para na linię):

    RTK_CASTER=system.asgeupos.pl
    RTK_PORT=2101
    RTK_MOUNTPOINT=RTN_VRS_3_1
    RTK_USER=login
    RTK_PASSWORD=haslo

Użycie:
    ./scripts/gnss_rtk.py test-caster         - sprawdź konto RTK Z TEGO KOMPUTERA
                                                (nie dotyka odbiornika - oddziela
                                                 "złe hasło" od "odbiornik bez sieci")
    ./scripts/gnss_rtk.py status              - co odbiornik ma ustawione teraz
    ./scripts/gnss_rtk.py sourcetable         - lista mountpointów z castera
    ./scripts/gnss_rtk.py configure           - włącz internet + skonfiguruj NTRIP
    ./scripts/gnss_rtk.py monitor [sekundy]   - podgląd statusu NTRIP i jakości fixa
    ./scripts/gnss_rtk.py internet-on|internet-off
    ./scripts/gnss_rtk.py off                 - rozłącz klienta NTRIP
"""

import argparse
import os
import re
import select
import sys
import termios
import time

DEFAULT_PORT_DEVICE = "/dev/gnss_aux"
DEFAULT_CASTER = "system.asgeupos.pl"
DEFAULT_CASTER_PORT = 2101
CONFIG_FILE = os.path.expanduser("~/.config/mani_ros/rtk.env")

# Deskryptor połączenia NTRIP w odbiorniku (NTR1..NTR4). Musi się zgadzać z
# parametrem ntrip_connection wtyczki GnssSensor.
NTRIP_CONNECTION = "NTR1"
# Strumień NMEA używany TYLKO przez ten skrypt do podglądu. Strumienie 1 i 2 są
# zajęte przez GnssSensor (pozycja i status) - pula Stream1..Stream10 jest
# wspólna dla całego odbiornika, więc powtórzenie numeru rozwaliłoby pomiar.
DIAG_STREAM = "Stream3"
# Nazwa portu tak, jak widzi go odbiornik. Numer wynika z tego, na którym
# porcie CDC siedzimy - inaczej włączylibyśmy strumień diagnostyczny na porcie,
# z którego nie czytamy (klasyczna pułapka: cisza mimo poprawnej komendy).
RECEIVER_PORT_BY_DEVICE = {"gnss": "USB1", "gnss_aux": "USB2"}

# Odbiornik kończy odpowiedź promptem w rodzaju "USB2>" BEZ znaku nowej linii.
PROMPT_RE = re.compile(rb"[A-Z]{2,4}\d*>\s*$")

NTRIP_STATUS = {
    0: "połączenie wyłączone",
    1: "inicjalizacja",
    2: "DZIAŁA - korekcje płyną",
    3: "błąd",
    4: "ponawianie połączenia",
    5: "wyłączone (duplikat innego połączenia)",
}

NTRIP_ERROR = {
    0: "brak błędu",
    1: "błąd inicjalizacji (nie pobrano source table)",
    2: "błąd autoryzacji - zły login/hasło",
    3: "błąd połączenia - odbiornik nie ma internetu?",
    4: "mountpoint nie istnieje",
    5: "mountpoint niedostępny",
    6: "caster czeka na GGA (odbiornik jeszcze bez pozycji?)",
    7: "wysyłanie GGA wyłączone, a mountpoint go wymaga",
    8: "nie rozwiązano nazwy hosta - brak DNS",
    9: "poza obszarem obsługi serwisu",
    10: "błąd konfiguracji TLS",
    11: "błąd handshake TLS",
    12: "błąd odcisku certyfikatu TLS",
    13: "nieznany czas - nie da się zweryfikować certyfikatu TLS",
    254: "nieznany błąd",
}

# Typy RTCM3, które faktycznie płyną z ASG-EUPOS (zweryfikowane na żywo).
# MSM5 (10x5) to nowoczesny, wielokonstelacyjny format - mosaic-H go rozumie.
RTCM_TYPES = {
    1005: "pozycja stacji ARP", 1006: "pozycja stacji ARP + wysokość",
    1007: "opis anteny", 1013: "parametry systemu", 1030: "GPS residuals",
    1031: "GLONASS residuals", 1032: "pozycja stacji fizycznej",
    1033: "opis anteny i odbiornika", 1230: "GLONASS bias kodowy",
    1074: "MSM4 GPS", 1084: "MSM4 GLONASS", 1094: "MSM4 Galileo", 1124: "MSM4 BeiDou",
    1075: "MSM5 GPS", 1085: "MSM5 GLONASS", 1095: "MSM5 Galileo", 1125: "MSM5 BeiDou",
    1077: "MSM7 GPS", 1087: "MSM7 GLONASS", 1097: "MSM7 Galileo", 1127: "MSM7 BeiDou",
    1303: "Galileo residuals", 1304: "BeiDou residuals", 4094: "proprietary Trimble",
}

FIX_QUALITY = {
    0: "brak fixa",
    1: "single-point (bez korekcji)",
    2: "DGPS/SBAS",
    4: "RTK FIXED",
    5: "RTK float",
    6: "dead reckoning",
}


class Receiver:
    """Sesja komend na porcie CDC odbiornika."""

    def __init__(self, device, receiver_port=None):
        self.device = device
        self.receiver_port = receiver_port or RECEIVER_PORT_BY_DEVICE.get(
            os.path.basename(os.path.realpath(device)),
            RECEIVER_PORT_BY_DEVICE.get(os.path.basename(device), "USB2"))
        try:
            self.fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as exc:
            sys.exit(
                f"Nie można otworzyć {device}: {exc}\n"
                "Czy odbiornik jest podpięty i czy reguły udev są zainstalowane "
                "(./docker/run_real.sh install-udev)?"
            )
        self._make_raw()
        self._drain()

    def _make_raw(self):
        attrs = termios.tcgetattr(self.fd)
        # CDC-ACM ignoruje prędkość, ale tryb kanoniczny i echo już nie -
        # bez raw dostalibyśmy poskładane linie i własne komendy z powrotem.
        attrs[0] = 0  # iflag
        attrs[1] = 0  # oflag
        attrs[3] = 0  # lflag: bez ICANON, ECHO, ISIG
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def _drain(self):
        """Wyrzuca to, co zalegało w buforze (np. resztki strumienia NMEA)."""
        while select.select([self.fd], [], [], 0.05)[0]:
            if not os.read(self.fd, 4096):
                break

    def command(self, text, timeout=5.0):
        """Wysyła komendę i zwraca odpowiedź odbiornika (bez promptu)."""
        os.write(self.fd, (text + "\r\n").encode())
        buf = b""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if select.select([self.fd], [], [], 0.1)[0]:
                chunk = os.read(self.fd, 4096)
                if chunk:
                    buf += chunk
                    if PROMPT_RE.search(buf):
                        break
        reply = PROMPT_RE.sub(b"", buf).decode("ascii", "replace").strip()
        if not reply:
            raise TimeoutError(f"Brak odpowiedzi na komendę: {text}")
        return reply

    def read_sentences(self, seconds, callback):
        """Czyta zdania NMEA przez zadany czas, woła callback dla każdego."""
        buf = b""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if select.select([self.fd], [], [], 0.2)[0]:
                buf += os.read(self.fd, 4096)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("ascii", "replace").strip()
                # Prompt cudzej sesji potrafi się skleić z początkiem zdania.
                if "$" in text:
                    callback(text[text.rindex("$"):])

    def close(self):
        os.close(self.fd)


def load_config():
    """Ustawienia: plik rtk.env pod spodem, zmienne środowiskowe na wierzchu."""
    values = {}
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, _, value = line.partition("=")
                values[key.strip()] = value.strip().strip('"').strip("'")
    values.update({k: v for k, v in os.environ.items() if k.startswith("RTK_")})
    return values


def require_credentials(config):
    caster = config.get("RTK_CASTER", DEFAULT_CASTER)
    port = config.get("RTK_PORT", str(DEFAULT_CASTER_PORT))
    mountpoint = config.get("RTK_MOUNTPOINT")
    user = config.get("RTK_USER")
    password = config.get("RTK_PASSWORD")

    missing = [
        name
        for name, value in (
            ("RTK_MOUNTPOINT", mountpoint),
            ("RTK_USER", user),
            ("RTK_PASSWORD", password),
        )
        if not value
    ]
    if missing:
        sys.exit(
            "Brak ustawień: " + ", ".join(missing) + "\n"
            f"Uzupełnij {CONFIG_FILE} albo wyeksportuj zmienne środowiskowe.\n"
            "Mountpointy dostępne dla Twojego konta pokaże: ./scripts/gnss_rtk.py sourcetable"
        )

    # Komendy Septentrio są rozdzielane przecinkami, a wartości ze spacjami
    # wymagają cudzysłowów - lepiej odrzucić od razu niż wysłać śmieci.
    for name, value in (("RTK_USER", user), ("RTK_PASSWORD", password),
                        ("RTK_MOUNTPOINT", mountpoint)):
        if "," in value or '"' in value or " " in value:
            sys.exit(f"{name} zawiera przecinek, spację lub cudzysłów - odbiornik tego nie przyjmie.")
    if len(password) > 20:
        # Reference guide: pole ma 40 znaków, ale użytkownikowi przysługuje połowa.
        sys.exit("RTK_PASSWORD dłuższe niż 20 znaków - odbiornik obetnie je po cichu.")

    return caster, port, mountpoint, user, password


def mask(text):
    return text[:2] + "*" * max(0, len(text) - 2)


def parse_snc(sentence, connection_index=1):
    """$PSSN,SNC,[rev,TOW,WNc,[CDIndex,Status,Error,Info]] -> (status, error)."""
    flat = sentence.replace("[", "").replace("]", "").split("*")[0]
    fields = flat.split(",")
    if len(fields) < 6 or fields[1] != "SNC":
        return None
    for i in range(5, len(fields) - 3, 4):
        try:
            if int(fields[i]) != connection_index:
                continue
            return int(fields[i + 1]), int(fields[i + 2])
        except ValueError:
            continue
    return None


def parse_gga(sentence):
    """-> (jakość fixa, liczba satelitów, wiek korekcji, ID stacji)."""
    fields = sentence.split("*")[0].split(",")
    if len(fields) < 15 or not fields[0].endswith("GGA"):
        return None

    def number(text, cast=float):
        try:
            return cast(text)
        except ValueError:
            return None

    return (number(fields[6], int), number(fields[7], int),
            number(fields[13]), fields[14] or None)


def cmd_status(receiver, _config, _args):
    print(f"Odbiornik na {receiver.device}:\n")
    for label, command in (
        ("Dostęp do internetu przez USB", "getUSBInternetAccess"),
        ("Ustawienia NTRIP", f"getNtripSettings, {NTRIP_CONNECTION}"),
    ):
        print(f"  {label}:")
        for line in receiver.command(command).splitlines():
            print(f"    {line.strip()}")
        print()

    print("  Bieżący stan połączenia (jedno zdanie SNC):")
    receiver.command(f"setNMEAOutput, {DIAG_STREAM}, {receiver.receiver_port}, SNC, sec1")
    shown = []

    def show(sentence):
        parsed = parse_snc(sentence)
        if parsed and not shown:
            status, error = parsed
            shown.append(True)
            print(f"    status: {NTRIP_STATUS.get(status, status)}")
            print(f"    błąd:   {NTRIP_ERROR.get(error, error)}")

    receiver.read_sentences(3.0, show)
    receiver.command(f"setNMEAOutput, {DIAG_STREAM}, none")
    if not shown:
        print("    (brak zdania SNC - klient NTRIP prawdopodobnie nigdy nie był konfigurowany)")


def cmd_sourcetable(receiver, config, _args):
    caster = config.get("RTK_CASTER", DEFAULT_CASTER)
    port = config.get("RTK_PORT", str(DEFAULT_CASTER_PORT))
    print(f"Pobieram source table z {caster}:{port} (to sam odbiornik pyta, więc "
          f"udany wynik dowodzi, że MA internet)...\n")
    try:
        reply = receiver.command(f"lstNTRIPSourceTable, {caster}, {port}", timeout=30.0)
    except TimeoutError:
        sys.exit(
            "Odbiornik nie odpowiedział w 30 s. Najczęstsza przyczyna: nie ma internetu.\n"
            "Sprawdź: sudo ./scripts/gnss_internet_share.sh status oraz "
            "./scripts/gnss_rtk.py status"
        )

    streams = [line for line in reply.splitlines() if line.startswith("STR;")]
    if not streams:
        print(reply)
        return
    print(f"{'mountpoint':<24} {'format':<18} {'sieć':<10} opis")
    print("-" * 78)
    for line in streams:
        parts = line.split(";")
        while len(parts) < 9:
            parts.append("")
        print(f"{parts[1]:<24} {parts[3]:<18} {parts[7]:<10} {parts[2]}")
    print(f"\nRazem {len(streams)} strumieni. Do batymetrii bierz VRS w RTCM 3.x "
          f"(wymaga wysyłania GGA - odbiornik robi to sam w trybie 'auto').")


def nmea_gga(lat, lon):
    """Minimalne GGA do zgłoszenia pozycji casterowi VRS."""
    def deg_min(value, width):
        degrees = int(abs(value))
        return f"{degrees:0{width}d}{(abs(value) - degrees) * 60:07.4f}"

    stamp = time.strftime("%H%M%S", time.gmtime())
    body = (f"GPGGA,{stamp}.00,{deg_min(lat, 2)},{'N' if lat >= 0 else 'S'},"
            f"{deg_min(lon, 3)},{'E' if lon >= 0 else 'W'},1,10,1.0,120.0,M,38.0,M,,")
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return f"${body}*{checksum:02X}\r\n".encode()


def cmd_test_caster(_receiver, config, args):
    """Łączy się z casterem Z HOSTA i sprawdza, czy korekcje płyną.

    Sens: gdy na odbiorniku nie wchodzi RTK, to albo konto/mountpoint, albo
    sieć odbiornika. Ten test rozstrzyga, która połowa jest winna, zanim
    zaczniemy grzebać w konfiguracji sprzętu.
    """
    import base64
    import socket

    caster, port, mountpoint, user, password = require_credentials(config)
    print(f"Łączę się z {caster}:{port}/{mountpoint} jako {user} (hasło {mask(password)})...\n")

    auth = base64.b64encode(f"{user}:{password}".encode()).decode()
    try:
        sock = socket.create_connection((caster, int(port)), timeout=15)
    except OSError as exc:
        sys.exit(f"Nie ma połączenia z casterem: {exc}")
    sock.settimeout(20)
    sock.sendall(
        f"GET /{mountpoint} HTTP/1.0\r\nUser-Agent: NTRIP mani_ros/1.0\r\n"
        f"Authorization: Basic {auth}\r\nAccept: */*\r\nConnection: close\r\n\r\n".encode())

    header = sock.recv(512)
    first = header.split(b"\r\n")[0].decode("latin-1")
    print(f"  odpowiedź castera: {first}")
    if b"200" not in header:
        sock.close()
        if b"401" in header:
            sys.exit("  Odrzucone - zły login lub hasło (RTK_USER / RTK_PASSWORD).")
        sys.exit("  Caster nie oddał strumienia - sprawdź nazwę mountpointu (sourcetable).")

    # VRS liczy korekcje dla pozycji zgłoszonej przez klienta - bez GGA
    # połączenie stoi otwarte, ale nic nie przychodzi.
    sock.sendall(nmea_gga(args.latitude, args.longitude))
    print(f"  wysłałem GGA dla {args.latitude:.4f}°N {args.longitude:.4f}°E, "
          f"słucham {args.seconds:.0f} s...")

    data = b""
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            chunk = sock.recv(4096)
            if not chunk:
                break
            data += chunk
    except socket.timeout:
        pass
    sock.close()

    types = {}
    index = 0
    while index < len(data) - 6:
        if data[index] != 0xD3:
            index += 1
            continue
        length = ((data[index + 1] & 0x03) << 8) | data[index + 2]
        if index + 6 + length > len(data):
            break
        message = (data[index + 3] << 4) | (data[index + 4] >> 4)
        types[message] = types.get(message, 0) + 1
        index += 6 + length

    print(f"\n  odebrano {len(data)} B, typy wiadomości RTCM3:")
    for message in sorted(types):
        label = RTCM_TYPES.get(message, "")
        print(f"    {message:<6} x{types[message]:<4} {label}")

    if len(data) > 1000:
        print("\nKonto i mountpoint działają. Jeśli odbiornik nadal nie wchodzi w RTK,"
              "\nproblem jest po stronie jego dostępu do sieci"
              " (gnss_internet_share.sh status).")
    else:
        print("\nPołączenie stanęło, ale korekcje nie płyną - czy mountpoint na pewno"
              "\njest typu VRS i czy zgłoszona pozycja mieści się w obszarze serwisu?")


def cmd_internet_on(receiver, _config, _args):
    print(receiver.command("setUSBInternetAccess, on"))
    print("\nOdbiornik jest teraz klientem DHCP na łączu USB - pod 192.168.3.1 "
          "już NIE odpowiada.\nJego nowy adres: sudo ./scripts/gnss_internet_share.sh status")


def cmd_internet_off(receiver, _config, _args):
    print(receiver.command("setUSBInternetAccess, off"))


def cmd_off(receiver, _config, _args):
    print(receiver.command(f"setNtripSettings, {NTRIP_CONNECTION}, off"))
    print(receiver.command("exeCopyConfigFile, Current, Boot"))


def cmd_configure(receiver, config, args):
    caster, port, mountpoint, user, password = require_credentials(config)

    print(f"Konfiguruję {NTRIP_CONNECTION}: {user}@{caster}:{port}/{mountpoint} "
          f"(hasło {mask(password)})\n")

    print("1/4 Włączam dostęp do internetu przez USB...")
    print("   ", receiver.command("setUSBInternetAccess, on").replace("\n", "\n    "))

    # SendGGA=auto: odbiornik sam wysyła GGA, jeśli caster tego wymaga - a VRS
    # wymaga, bo korekcje są generowane dla bieżącej pozycji odbiornika.
    # Hojny timeout: po tej komendzie odbiornik od razu zestawia połączenie z
    # casterem i potrafi milczeć kilkanaście sekund. Krótki timeout dawał
    # wyjątek MIMO że komenda przechodziła - i przerywał zapis do Boot.
    print("2/4 Ustawiam klienta NTRIP...")
    try:
        reply = receiver.command(
            f"setNtripSettings, {NTRIP_CONNECTION}, Client, {caster}, {port}, "
            f"{user}, {password}, {mountpoint}, v2, auto",
            timeout=20.0,
        )
        print("   ", reply.replace(password, mask(password)).replace("\n", "\n    "))
    except TimeoutError:
        print("    (odbiornik nie potwierdził w 20 s - sprawdzam, czy mimo to przyjął)")
        confirmation = receiver.command(f"getNtripSettings, {NTRIP_CONNECTION}", timeout=20.0)
        if mountpoint not in confirmation:
            sys.exit(f"    Nie przyjął ustawień:\n    {confirmation}")
        print("    ", confirmation.replace("\n", "\n    "))

    print("3/4 Zapisuję konfigurację do profilu Boot (przetrwa restart)...")
    print("   ", receiver.command(
        "exeCopyConfigFile, Current, Boot", timeout=20.0).replace("\n", "\n    "))

    print(f"4/4 Sprawdzam połączenie przez {args.seconds} s...\n")
    monitor(receiver, args.seconds)


def cmd_monitor(receiver, _config, args):
    monitor(receiver, args.seconds)


def monitor(receiver, seconds):
    """Podgląd statusu NTRIP i jakości fixa na porcie diagnostycznym."""
    receiver.command(f"setNMEAOutput, {DIAG_STREAM}, {receiver.receiver_port}, GGA+SNC, sec1")
    state = {"ntrip": None, "fix": None}

    def show(sentence):
        parsed = parse_snc(sentence)
        if parsed:
            if parsed != state["ntrip"]:
                state["ntrip"] = parsed
                status, error = parsed
                suffix = "" if error == 0 else f"  [{NTRIP_ERROR.get(error, error)}]"
                print(f"  NTRIP: {NTRIP_STATUS.get(status, status)}{suffix}")
            return

        gga = parse_gga(sentence)
        if gga:
            quality, satellites, age, station = gga
            if quality != state["fix"]:
                state["fix"] = quality
                print(f"  FIX:   {FIX_QUALITY.get(quality, quality)}"
                      f"  ({satellites} sat)")
            if quality in (2, 4, 5) and age is not None:
                print(f"         wiek korekcji {age:.1f} s, stacja {station}")

    try:
        receiver.read_sentences(seconds, show)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.command(f"setNMEAOutput, {DIAG_STREAM}, none")

    print()
    if state["fix"] == 4:
        print("RTK fixed - pozycja centymetrowa. Gotowe.")
    elif state["ntrip"] and state["ntrip"][0] == 2:
        print("Korekcje płyną, ale fix jeszcze nie doszedł do 4. Daj odbiornikowi "
              "kilkadziesiąt sekund pod otwartym niebem.")
    elif state["ntrip"] and state["ntrip"][1] == 6 and not state["fix"]:
        # Nie awaria: VRS liczy korekcje dla pozycji zgłoszonej przez GGA, więc
        # bez własnego fixa odbiornik nie ma czego wysłać. Rozwiąże się samo
        # na dworze - i tak wygląda poprawnie skonfigurowany stack w budynku.
        print("Konfiguracja jest poprawna. Odbiornik nie ma jeszcze fixa (0 sat),\n"
              "więc nie ma czego zgłosić casterowi VRS - korekcje ruszą same,\n"
              "gdy złapie pozycję pod otwartym niebem.")
    elif state["ntrip"]:
        print("Klient NTRIP nie zestawił połączenia - patrz komunikat wyżej.")
    else:
        print("Brak zdań SNC - sprawdź, czy klient NTRIP w ogóle jest skonfigurowany "
              "(./scripts/gnss_rtk.py status).")


COMMANDS = {
    "test-caster": cmd_test_caster,
    "status": cmd_status,
    "sourcetable": cmd_sourcetable,
    "configure": cmd_configure,
    "monitor": cmd_monitor,
    "internet-on": cmd_internet_on,
    "internet-off": cmd_internet_off,
    "off": cmd_off,
}

COMMANDS_WITHOUT_RECEIVER = {"test-caster"}


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=sorted(COMMANDS))
    parser.add_argument(
        "seconds", nargs="?", type=float, default=30.0,
        help="czas podglądu dla 'monitor'/'configure' (domyślnie 30 s)")
    parser.add_argument(
        "--device", default=DEFAULT_PORT_DEVICE,
        help=f"port CDC odbiornika (domyślnie {DEFAULT_PORT_DEVICE})")
    parser.add_argument(
        "--receiver-port",
        help="nazwa portu po stronie odbiornika (USB1/USB2); domyślnie z nazwy urządzenia")
    parser.add_argument(
        "--latitude", type=float, default=51.076,
        help="szerokość zgłaszana casterowi VRS w 'test-caster' (domyślnie Wrocław)")
    parser.add_argument(
        "--longitude", type=float, default=16.966,
        help="długość zgłaszana casterowi VRS w 'test-caster'")
    args = parser.parse_args()

    # test-caster rozmawia tylko z casterem - ma działać także wtedy, gdy
    # odbiornik jest odpięty albo port trzyma stack ROS.
    if args.command in COMMANDS_WITHOUT_RECEIVER:
        COMMANDS[args.command](None, load_config(), args)
        return

    receiver = Receiver(args.device, args.receiver_port)
    try:
        COMMANDS[args.command](receiver, load_config(), args)
    finally:
        receiver.close()


if __name__ == "__main__":
    main()
