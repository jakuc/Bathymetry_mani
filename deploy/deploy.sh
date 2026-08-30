#!/usr/bin/env bash
# deploy.sh - wypchnięcie stacka na płytkę bathset (Raspberry Pi 3B). Ze STACJI.
#
# Model: płytka ma ROS natywnie i sama kompiluje workspace. Stacja daje jej
# internet przez NAT i wysyła źródła - nic tu nie jest cross-kompilowane i nie
# ma żadnego Dockera. Poprzednie podejście (obraz arm64 pod qemu + komplet
# offline .deb) zostało porzucone: brało się z założenia, że płytka nie ma
# internetu, a NAT to założenie unieważnia.
#
#   ./deploy/deploy.sh              # net + rsync + build na płytce
#   ./deploy/deploy.sh --provision  # świeża płytka: najpierw install.sh, potem build
#   ./deploy/deploy.sh --net        # tylko podnieś NAT i znajdź płytkę
#   ./deploy/deploy.sh --status     # co jest na płytce
#   ./deploy/deploy.sh --run        # odpal stack (argumenty launcha w LAUNCH_ARGS)
#   ./deploy/deploy.sh --sweep      # odpal sweep sferyczny dalmierzem (SWEEP_ARGS)
#
# Zmienne:
#   BATHSET_HOST   - user@adres; domyślnie autowykrywanie po MAC w sieci NAT
#   NM_CON         - profil NetworkManagera z NAT-em (domyślnie bathset-eth)
#   LAUNCH_ARGS    - argumenty do real_hardware.launch.py przy --run
#   SWEEP_ARGS     - argumenty do sweep_laser.launch.py przy --sweep
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$HERE")"

NM_CON="${NM_CON:-bathset-eth}"
RPI_USER="${RPI_USER:-ubuntu}"
REMOTE_WS="${REMOTE_WS:-bathset_ws}"
RPI_OUI="b8:27:eb"          # pula MAC Raspberry Pi Foundation
# Podsieć NAT-u NIE jest stała, mimo że przez długi czas wychodziła na 10.42.0.
# NetworkManager w trybie "shared" nadaje 10.42.X.0/24 i zwiększa X, gdy inny
# aktywny profil shared już trzyma poprzednią pulę - wystarczy druga karta
# z własnym udostępnianiem (u nas wojtek-eth na adapterze USB), żeby bathset-eth
# wylądował na 10.42.1.0/24. Dlatego pulę czytamy z interfejsu, a nie zgadujemy;
# wartość poniżej to tylko awaryjny fallback.
NAT_SUBNET_FALLBACK="10.42.0"
# Domyślne argumenty odzwierciedlają to, co jest FIZYCZNIE podpięte do płytki.
# To nie jest kosmetyka: controller_manager twardo pada (abort całego procesu),
# jeśli zadeklarowany w URDF komponent nie osiągnie stanu "active" - a to się
# dzieje zawsze, gdy urządzenia nie ma na magistrali. Dlatego echosonda i GNSS
# są tu wyłączone, mimo że w samym launchu domyślnie są włączone; włącz je,
# gdy je podepniesz.
#
# Adresy serw (EEPROM serwa, sprawdzone skanem magistrali 2026-08-07):
#   ID 1 = człon 2 (głowica) -> xm540_joint
#   ID 2 = człon 1           -> xm540_joint_z
LAUNCH_ARGS="${LAUNCH_ARGS:-use_servo:=true use_servo_z:=true servo_id:=1 servo_id_z:=2 use_imu:=true use_echosounder:=false use_gnss:=false use_rviz:=false}"

# Sweep sferyczny dalmierzem: sweep_laser.launch.py sam wymusza use_servo,
# use_servo_z i use_laser, a IMU/echosondę/GNSS domyślnie wyłącza - do tego
# eksperymentu potrzebne są tylko serwa i dalmierz na /dev/laser. Tu zostają
# więc tylko parametry samego skanu.
# \$HOME jest tu celowo NIEROZWINIĘTE: rozwinie się dopiero na płytce, gdzie
# katalog domowy to /home/ubuntu, a nie katalog domowy stacji.
SWEEP_ARGS="${SWEEP_ARGS:-az_min_deg:=-15.0 az_max_deg:=15.0 az_step_deg:=2.0 el_min_deg:=-15.0 el_max_deg:=15.0 el_step_deg:=2.0 output_dir:=\$HOME/scans}"

SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=20 -o StrictHostKeyChecking=accept-new)

DO_PROVISION=0 DO_BUILD=1 DO_NET_ONLY=0 DO_STATUS=0 DO_RUN=0 DO_SWEEP=0
for a in "$@"; do case "$a" in
    --provision) DO_PROVISION=1 ;;
    --net)       DO_NET_ONLY=1; DO_BUILD=0 ;;
    --status)    DO_STATUS=1;   DO_BUILD=0 ;;
    --run)       DO_RUN=1;      DO_BUILD=0 ;;
    --sweep)     DO_SWEEP=1;    DO_BUILD=0 ;;
    --no-build)  DO_BUILD=0 ;;
    -h|--help)   sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "nieznany argument: $a" >&2; exit 2 ;;
esac; done

say() { printf '\n\033[1m>> %s\033[0m\n' "$*"; }

# -----------------------------------------------------------------------------
# NAT ze stacji. Bez tego płytka nie ma internetu i apt nie ma jak działać.
#
# PUŁAPKA, która kosztowała nas kilka pomyłek: na tej stacji istnieje kilka
# profili ethernetowych z autoconnect=yes (np. dron-eth). Po KAŻDYM zerwaniu
# linku - czyli po każdym restarcie płytki i po każdym przepięciu kabla -
# NetworkManager przelicza profile i któryś z nich zabiera interfejs. Stacja
# ląduje wtedy na 169.254/16, DHCP znika i płytka jest nieosiągalna, co wygląda
# jak zawieszona płytka, a jest tylko podmienionym profilem. Dlatego profil
# NAT-owy podnosimy JAWNIE przy każdym uruchomieniu.
# -----------------------------------------------------------------------------
net_up() {
    command -v nmcli >/dev/null || { echo "brak nmcli - ta stacja nie jest na NetworkManagerze" >&2; return 1; }
    if ! nmcli -t -f NAME connection show | grep -qx "$NM_CON"; then
        say "Tworzę profil ${NM_CON} (ipv4.method shared)"
        local iface
        iface="$(nmcli -t -f DEVICE,TYPE device status | awk -F: '$2=="ethernet"{print $1; exit}')"
        [ -z "$iface" ] && { echo "nie znalazłem interfejsu ethernet" >&2; return 1; }
        nmcli connection add type ethernet ifname "$iface" con-name "$NM_CON" \
            ipv4.method shared connection.autoconnect no >/dev/null
    fi
    nmcli connection up "$NM_CON" >/dev/null 2>&1 || true
    # Kontrola, że default route NIE przeniósł się na profil NAT-owy: NAT musi
    # mieć skąd brać internet (u nas: osobna karta USB), inaczej udostępnia nic.
    local defdev
    defdev="$(ip route show default | awk '{print $5; exit}')"
    say "NAT: ${NM_CON} podniesiony; default route przez ${defdev:-BRAK}"
    [ -z "$defdev" ] && echo "   UWAGA: stacja nie ma default route - płytka nie dostanie internetu" >&2
    return 0
}

# Autowykrywanie płytki po MAC w podsieci NAT-owej.
#
# Rozgłoszeniowy ping tu NIE działa: Linux domyślnie ma
# net.ipv4.icmp_echo_ignore_broadcasts=1, więc płytka nie odpowie i wpis ARP nie
# powstanie. Zamiast tego zamiatamy podsieć pojedynczymi pingami (równolegle,
# całość schodzi w sekundę) i dopiero potem czytamy tablicę sąsiedztwa, gdzie
# rozpoznajemy płytkę po OUI Raspberry Pi.
# Odczytuje pule, w których w ogóle może siedzieć płytka. Bez tego skanujemy
# podsieć, w której jej nie ma - a objaw (płytka pinguje się z siebie, ale
# stacja mówi "No route to host") wygląda jak awaria sprzętu.
#
# Zwracamy LISTĘ, nie jedną pulę, bo płytka nie musi wisieć na profilu NM_CON.
# Przy przepięciu kabla z karty wbudowanej na adapter USB pulę z płytką trzyma
# profil o CUDZEJ nazwie (u nas wojtek-eth na tym samym adapterze), a bathset-eth
# dostaje własną, pustą podsieć - szukanie tylko pod NM_CON kończy się wtedy
# komunikatem "nie znalazłem płytki", mimo że jest podpięta i odpowiada.
# Profil NM_CON idzie pierwszy, reszta pul 10.42.x ze stacji za nim.
detect_nat_subnets() {
    local dev
    dev="$(nmcli -t -f NAME,DEVICE connection show --active 2>/dev/null \
           | awk -F: -v n="$NM_CON" '$1==n{print $2; exit}')"
    {
        if [ -n "$dev" ]; then
            ip -4 -o addr show dev "$dev" 2>/dev/null | awk '{print $4}' | cut -d/ -f1
        fi
        ip -4 -o addr show scope global 2>/dev/null | awk '{print $4}' \
            | cut -d/ -f1 | grep '^10\.42\.' || true
    } | sed 's/\.[0-9]*$//' | awk 'NF && !seen[$0]++'
}

find_rpi() {
    if [ -n "${BATHSET_HOST:-}" ]; then echo "$BATHSET_HOST"; return 0; fi
    local ip="" try sub i
    for try in 1 2 3; do
        for sub in $NAT_SUBNETS; do
            ip="$(ip -4 neigh show 2>/dev/null | grep -i "$RPI_OUI" | awk '{print $1}' | grep "^${sub}\." | head -1)"
            if [ -n "$ip" ]; then break; fi
        done
        # if/then, nie "[ ... ] && break": pod set -e nieudany test kończy całą
        # funkcję (a razem z nią podpowłokę $(find_rpi)) zamiast dać jej szansę
        # na kolejne podejście i na sensowny komunikat błędu.
        if [ -n "$ip" ]; then break; fi
        for sub in $NAT_SUBNETS; do
            for i in $(seq 2 254); do
                ping -c1 -W1 "${sub}.${i}" >/dev/null 2>&1 &
            done
        done
        wait
        sleep 1
    done
    if [ -z "$ip" ]; then
        echo "nie znalazłem płytki w podsieciach: ${NAT_SUBNETS} (MAC ${RPI_OUI}:*)" >&2
        echo "" >&2
        echo "Najczęstsza przyczyna: płytka trzyma lease z POPRZEDNIEJ puli NAT-u." >&2
        echo "systemd-networkd nie porzuca go sam - 'networkctl renew' odnawia stary" >&2
        echo "adres zamiast prosić o nowy. Wejdź po IPv6 link-local i wymuś DISCOVER:" >&2
        echo "  ssh 'ubuntu@fe80::ba27:ebff:fe78:9e64%<iface>'" >&2
        echo "  sudo rm -f /run/systemd/netif/leases/*; sudo systemctl restart systemd-networkd" >&2
        return 1
    fi
    echo "${RPI_USER}@${ip}"
}

rpi_ssh() { local h="$1"; shift; ssh "${SSH_OPTS[@]}" "$h" "$@"; }

# -----------------------------------------------------------------------------
# NAT podnosimy TYLKO wtedy, gdy sami mamy znaleźć płytkę. Podany BATHSET_HOST
# znaczy, że droga do niej już istnieje - a od 2026-08-30 bywa nią hotspot
# WYSTAWIANY PRZEZ PŁYTKĘ, przy kablu wpiętym gdzie indziej. Bezwarunkowe
# net_up przełączyłoby wtedy kartę kablową stacji na profil `bathset-eth`
# i odcięło stację od internetu, zupełnie bez potrzeby.
if [ -n "${BATHSET_HOST:-}" ]; then
    say "BATHSET_HOST podany - pomijam podnoszenie NAT-u"
else
    net_up
fi
NAT_SUBNETS="$(detect_nat_subnets)"
if [ -z "$NAT_SUBNETS" ]; then NAT_SUBNETS="$NAT_SUBNET_FALLBACK"; fi
NAT_SUBNETS="$(echo $NAT_SUBNETS)"      # lista w jednej linii, do pętli po słowach
HOST="$(find_rpi)"
say "Płytka: ${HOST}"

if [ "$DO_NET_ONLY" = 1 ]; then exit 0; fi

# ------------------------------------------------------------------ provision
if [ "$DO_PROVISION" = 1 ]; then
    say "Przygotowanie płytki (install.sh) - to potrafi trwać ~godzinę na świeżej karcie"
    rpi_ssh "$HOST" "mkdir -p '${REMOTE_WS}/deploy'"
    rsync -az --delete -e "ssh ${SSH_OPTS[*]}" \
        "${HERE}/rpi/" "${HOST}:${REMOTE_WS}/deploy/rpi/"
    rsync -az --delete -e "ssh ${SSH_OPTS[*]}" \
        "${REPO_DIR}/udev/" "${HOST}:${REMOTE_WS}/udev/"
    # install.sh sięga po reguły przez ../../udev, więc układ katalogów na
    # płytce musi odwzorowywać repo.
    rpi_ssh "$HOST" "chmod +x '${REMOTE_WS}/deploy/rpi/install.sh' && bash '${REMOTE_WS}/deploy/rpi/install.sh'"
fi

# ----------------------------------------------------------------- status
if [ "$DO_STATUS" = 1 ]; then
    rpi_ssh "$HOST" "bash -lc '
        echo \"--- ROS ---\";        ls -d /opt/ros/humble 2>/dev/null || echo \"BRAK /opt/ros/humble\"
        echo \"--- workspace ---\";  ls ~/${REMOTE_WS}/install/setup.bash 2>/dev/null || echo \"BRAK install/\"
        echo \"--- paczki ---\";     ros2 pkg list 2>/dev/null | grep -E \"hardware_controller|bathset_description|xm540\" || echo \"nie widać\"
        echo \"--- urządzenia ---\"
        for d in /dev/u2d2 /dev/echosounder /dev/gnss /dev/gnss_aux /dev/serial0 /dev/laser; do
            [ -e \"\$d\" ] && echo \"  [ok]   \$d\" || echo \"  [brak] \$d\"
        done
        echo \"--- zasoby ---\";     free -h | head -3
    '"
    exit 0
fi

# ----------------------------------------------------------- run / sweep
if [ "$DO_RUN" = 1 ] || [ "$DO_SWEEP" = 1 ]; then
    if [ "$DO_SWEEP" = 1 ]; then
        RUN_LAUNCH="sweep_laser.launch.py"; RUN_ARGS="$SWEEP_ARGS"
    else
        RUN_LAUNCH="real_hardware.launch.py"; RUN_ARGS="$LAUNCH_ARGS"
    fi

    say "Odpalam ${RUN_LAUNCH}: ${RUN_ARGS}"
    # setsid + log na płytce: zerwane SSH nie może ubić stacka, a log przeżywa
    # rozłączenie. Logi idą do ~/, NIE do /tmp - reboot czyści tmpfs.
    # UWAGA: ten skrypt startuje z powłoki NIELOGOWANEJ (setsid nohup), więc
    # /etc/profile.d/bathset.sh NIE wykonuje się tutaj. Zmienne środowiskowe
    # trzeba powtórzyć jawnie - inaczej stack wstaje bez profilu Fast DDS i
    # stacja go nie widzi, mimo że po zalogowaniu przez SSH wszystko wygląda
    # poprawnie. To była realna pułapka, nie teoria.
    rpi_ssh "$HOST" "cat > ~/run_stack.sh <<'EOF'
#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/${REMOTE_WS}/install/setup.bash
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}
[ -f /etc/bathset/fastdds_eth.xml ] && export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/bathset/fastdds_eth.xml
ros2 launch hardware_controller ${RUN_LAUNCH} ${RUN_ARGS} > ~/stack.log 2>&1
EOF
chmod +x ~/run_stack.sh; rm -f ~/stack.log
setsid nohup ~/run_stack.sh </dev/null >/dev/null 2>&1 &
sleep 2; echo 'stack wystartował, log: ~/stack.log'"
    exit 0
fi

# ------------------------------------------------------------------ build
if [ "$DO_BUILD" = 1 ]; then
    say "rsync src/ -> ${HOST}:${REMOTE_WS}/src"
    # Wykluczenia NIE są kosmetyczne: src/ waży 3,3 GB przez meshe jeziora dla
    # Isaac Sima (big_lake_simp.obj 1,5 GB + .usd 1,3 GB + kafelki kolizji).
    # setup.py w xm540_bringup instaluje z meshes/ tylko glob *.stl (784 KB),
    # więc te pliki i tak nigdy nie trafiłyby do install/. Bez wykluczeń
    # transfer nie ma końca; z nimi to 43 MB.
    rsync -az --delete -e "ssh ${SSH_OPTS[*]}" \
        --exclude 'build/' --exclude 'install/' --exclude 'log/' \
        --exclude '__pycache__/' --exclude '*.pyc' \
        --exclude 'meshes/big_lake_simp.obj' \
        --exclude 'meshes/big_lake_simp.usd' \
        --exclude 'meshes/big_lake_simp_tiles/' \
        "${REPO_DIR}/src/" "${HOST}:${REMOTE_WS}/src/"

    say "colcon build na płytce"
    # MAKEFLAGS="-j1" jest KRYTYCZNE i nie da się go zastąpić przez
    # --parallel-workers: ta flaga ogranicza liczbę PACZEK budowanych naraz, a
    # nie liczbę zadań make WEWNĄTRZ paczki, która domyślnie idzie na $(nproc).
    # Na Pi 3B daje to cztery cc1plus po ~574 MB szczytu przy 905 MB RAM bez
    # swapu - płytka wiesza się tak, że sshd nie domyka nawet bannera (TCP/22
    # przyjmuje połączenie, bo to kernel; userspace jest zagłodzony) i jedynym
    # wyjściem jest odcięcie zasilania. Z -j1 build schodzi w ~11 minut.
    rpi_ssh "$HOST" "bash -s" <<REMOTE
set -eo pipefail
source /opt/ros/humble/setup.bash
cd "\$HOME/${REMOTE_WS}"
export MAKEFLAGS="-j1"
colcon build --merge-install --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-g0 -DBUILD_TESTING=OFF
REMOTE
    say "Gotowe. Sprawdzenie: $0 --status   |   uruchomienie: $0 --run"
fi
