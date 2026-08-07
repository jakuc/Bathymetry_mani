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
#
# Zmienne:
#   BATHSET_HOST   - user@adres; domyślnie autowykrywanie po MAC w sieci NAT
#   NM_CON         - profil NetworkManagera z NAT-em (domyślnie bathset-eth)
#   LAUNCH_ARGS    - argumenty do real_hardware.launch.py przy --run
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$HERE")"

NM_CON="${NM_CON:-bathset-eth}"
RPI_USER="${RPI_USER:-ubuntu}"
REMOTE_WS="${REMOTE_WS:-bathset_ws}"
RPI_OUI="b8:27:eb"          # pula MAC Raspberry Pi Foundation
NAT_SUBNET="10.42.0"
LAUNCH_ARGS="${LAUNCH_ARGS:-use_servo:=true use_echosounder:=false use_gnss:=false use_rviz:=false servo_id:=2}"

SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=20 -o StrictHostKeyChecking=accept-new)

DO_PROVISION=0 DO_BUILD=1 DO_NET_ONLY=0 DO_STATUS=0 DO_RUN=0
for a in "$@"; do case "$a" in
    --provision) DO_PROVISION=1 ;;
    --net)       DO_NET_ONLY=1; DO_BUILD=0 ;;
    --status)    DO_STATUS=1;   DO_BUILD=0 ;;
    --run)       DO_RUN=1;      DO_BUILD=0 ;;
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
find_rpi() {
    if [ -n "${BATHSET_HOST:-}" ]; then echo "$BATHSET_HOST"; return 0; fi
    local ip="" try
    for try in 1 2 3; do
        ip="$(ip -4 neigh show 2>/dev/null | grep -i "$RPI_OUI" | awk '{print $1}' | grep "^${NAT_SUBNET}\." | head -1)"
        [ -n "$ip" ] && break
        local i
        for i in $(seq 2 254); do
            ping -c1 -W1 "${NAT_SUBNET}.${i}" >/dev/null 2>&1 &
        done
        wait
        sleep 1
    done
    [ -z "$ip" ] && { echo "nie znalazłem płytki w ${NAT_SUBNET}.0/24 (MAC ${RPI_OUI}:*)" >&2; return 1; }
    echo "${RPI_USER}@${ip}"
}

rpi_ssh() { local h="$1"; shift; ssh "${SSH_OPTS[@]}" "$h" "$@"; }

# -----------------------------------------------------------------------------
net_up
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
        for d in /dev/u2d2 /dev/echosounder /dev/gnss /dev/gnss_aux; do
            [ -e \"\$d\" ] && echo \"  [ok]   \$d\" || echo \"  [brak] \$d\"
        done
        echo \"--- zasoby ---\";     free -h | head -3
    '"
    exit 0
fi

# ----------------------------------------------------------------- run
if [ "$DO_RUN" = 1 ]; then
    say "Odpalam stack: ${LAUNCH_ARGS}"
    # setsid + log na płytce: zerwane SSH nie może ubić stacka, a log przeżywa
    # rozłączenie. Logi idą do ~/, NIE do /tmp - reboot czyści tmpfs.
    rpi_ssh "$HOST" "cat > ~/run_stack.sh <<'EOF'
#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/${REMOTE_WS}/install/setup.bash
ros2 launch hardware_controller real_hardware.launch.py ${LAUNCH_ARGS} > ~/stack.log 2>&1
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
