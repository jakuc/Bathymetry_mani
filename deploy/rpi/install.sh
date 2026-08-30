#!/usr/bin/env bash
# install.sh - jednorazowe (i idempotentne) przygotowanie płytki bathset.
#
# Uruchamiane NA PŁYTCE, zwykle przez `deploy/deploy.sh --provision` ze stacji.
# Każda faza sprawdza stan przed zmianą, więc na już przygotowanej płytce jest
# to prawie no-op i można je puszczać bez obaw po każdej zmianie w repo.
#
# Płytka NIE MA Dockera i mieć nie będzie - ROS leży natywnie, kompilacja też
# idzie na płytce. Internet płytka dostaje przez NAT ze stacji (patrz
# deploy/deploy.sh, funkcja net_up) - bez tego apt nie ma jak zadziałać.
#
# Fazy (każdą można pominąć odpowiednią flagą):
#   1 repo ROS   : klucz GPG + packages.ros.org           (--skip-repo)
#   2 paczki     : ROS Humble + narzędzia do kompilacji   (--skip-packages)
#   3 udev       : /dev/u2d2, /dev/echosounder, /dev/gnss,
#                  /dev/laser (dalmierz na Nano)          (--skip-udev)
#   3.5 uart     : zwolnienie ttyS0 pod IMU GY-955        (--skip-uart)
#   3.6 wifi     : hotspot + przelacznik kabel/hotspot,
#                  power_save off na wlan0                  (--skip-wifi)
#   4 środowisko : /etc/profile.d/bathset.sh              (--skip-env)
#   5 zram       : skompresowany swap w RAM               (--skip-zram)
#
#   ./install.sh              # wszystko
#   ./install.sh --dry-run    # pokaż, co by zrobił, nie tykaj niczego
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
DRY_RUN=0
SKIP_REPO=0 SKIP_PACKAGES=0 SKIP_UDEV=0 SKIP_UART=0 SKIP_WIFI=0 SKIP_ENV=0 SKIP_ZRAM=0

for a in "$@"; do case "$a" in
    --dry-run)       DRY_RUN=1 ;;
    --skip-repo)     SKIP_REPO=1 ;;
    --skip-packages) SKIP_PACKAGES=1 ;;
    --skip-udev)     SKIP_UDEV=1 ;;
    --skip-uart)     SKIP_UART=1 ;;
    --skip-wifi)     SKIP_WIFI=1 ;;
    --skip-env)      SKIP_ENV=1 ;;
    --skip-zram)     SKIP_ZRAM=1 ;;
    *) echo "nieznany argument: $a" >&2; exit 2 ;;
esac; done

say()  { printf '\n\033[1m>> %s\033[0m\n' "$*"; }
info() { printf '   %s\n' "$*"; }
run()  { if [ "$DRY_RUN" = 1 ]; then printf '   [dry-run] %s\n' "$*"; else eval "$@"; fi; }

# Lista paczek. ros-base + to, czego wymaga hardware_controller, plus narzędzia
# do kompilacji - workspace budujemy na płytce, więc kompilator musi tu być.
ROS_PKGS="ros-${ROS_DISTRO}-ros-base
ros-${ROS_DISTRO}-robot-state-publisher
ros-${ROS_DISTRO}-xacro
ros-${ROS_DISTRO}-ros2-control
ros-${ROS_DISTRO}-ros2-controllers
ros-${ROS_DISTRO}-dynamixel-sdk
ros-${ROS_DISTRO}-realtime-tools
ros-${ROS_DISTRO}-diagnostic-msgs
ros-${ROS_DISTRO}-std-srvs
ros-${ROS_DISTRO}-tf2-ros
ros-${ROS_DISTRO}-tf2-geometry-msgs"
TOOL_PKGS="python3-colcon-common-extensions python3-rosdep build-essential git time dnsmasq-base iw hostapd"

# --------------------------------------------------------------------- faza 1
provision_repo() {
    say "Faza 1: repozytorium ROS 2 ${ROS_DISTRO}"
    if [ -f /etc/apt/sources.list.d/ros2.list ]; then
        info "repo już jest"
        return
    fi
    run "sudo apt-get update -qq"
    run "sudo apt-get install -y -qq curl gnupg lsb-release"
    # Klucz po HTTPS z GitHuba (działa), ale samo repo po HTTP - packages.ros.org
    # to CNAME na ftp.osuosl.org, który podaje cert *.osuosl.org niepasujący do
    # nazwy, więc HTTPS pod tym adresem nie działa NIGDZIE, także na stacji.
    # Autentyczność paczek zapewnia podpis GPG, nie warstwa transportowa.
    run "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg"
    run "echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo \$VERSION_CODENAME) main\" \
            | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null"
}

# --------------------------------------------------------------------- faza 2
provision_packages() {
    say "Faza 2: ROS ${ROS_DISTRO} + narzędzia"
    if [ -d "/opt/ros/${ROS_DISTRO}" ]; then
        info "/opt/ros/${ROS_DISTRO} już istnieje - doinstaluję tylko brakujące"
    fi
    # UWAGA na czasy: pierwszy `apt-get update` na Pi 3B (pełne indeksy jammy
    # dla arm64) potrafi zająć kilkanaście minut, a instalacja ~445 paczek
    # kolejne ~40. Nie skracać timeoutów wołających ten skrypt.
    run "sudo apt-get update -qq"
    run "sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ${ROS_PKGS//$'\n'/ } ${TOOL_PKGS}"
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        run "sudo rosdep init"
    fi
    run "rosdep update || true"
}

# --------------------------------------------------------------------- faza 3
provision_udev() {
    say "Faza 3: reguły udev"
    run "sudo install -m644 ${HERE}/../../udev/"'*.rules /etc/udev/rules.d/'
    # latency_timer FTDI: domyślne 16 ms powoduje COMM_RX_TIMEOUT na magistrali
    # Dynamixela (zmierzone: ~28% straconych odpowiedzi przy 16 ms). Ustawienie
    # 1 ms wyraźnie poprawia sytuację i musi być trwałe, bo sysfs wraca do
    # domyślnej wartości po każdym przepięciu.
    run "printf 'ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"1\"\n' \
            | sudo tee /etc/udev/rules.d/99-ftdi-latency.rules >/dev/null"
    run "sudo udevadm control --reload-rules"
    run "sudo udevadm trigger --subsystem-match=tty --subsystem-match=usb-serial"
    if [ "$DRY_RUN" != 1 ]; then
        sleep 2
        for d in /dev/u2d2 /dev/echosounder /dev/gnss /dev/gnss_aux /dev/serial0 /dev/laser; do
            [ -e "$d" ] && info "  [ok]   $d" || info "  [brak] $d (urządzenie niepodpięte?)"
        done
    fi
}

# ------------------------------------------------------------------ faza 3.5
provision_uart() {
    say "Faza 3.5: zwolnienie UART-u pod IMU"
    # IMU GY-955 wisi na wbudowanym UART-cie Pi (/dev/serial0 -> ttyS0, piny
    # 8/10), a nie na USB - więc nie ma dla niego reguły udev, ścieżka jest
    # stała. Trzeba za to odebrać port systemowi, bo domyślnie:
    #
    #   1. serial-getty@ttyS0 trzyma na nim konsolę logowania. Dopóki żyje,
    #      ttyS0 należy do grupy `tty` (crw--w---- root tty), a nie `dialout`,
    #      więc nasz proces nie ma jak go otworzyć - i to niezależnie od tego,
    #      w jakich grupach jest użytkownik.
    #   2. jądro dostaje `console=serial0,115200` z cmdline.txt i sypie na ten
    #      port logi rozruchowe, które wyglądają jak śmieci w ramkach IMU.
    #
    # Samo `stop` nie wystarcza - po reboocie getty wraca, stąd `disable`.
    run "sudo systemctl stop serial-getty@ttyS0.service || true"
    run "sudo systemctl disable serial-getty@ttyS0.service || true"
    run "sudo systemctl mask serial-getty@ttyS0.service || true"

    local cmdline=/boot/firmware/cmdline.txt
    [ -f "$cmdline" ] || cmdline=/boot/cmdline.txt
    if [ -f "$cmdline" ] && grep -q "console=serial0" "$cmdline" 2>/dev/null; then
        run "sudo cp ${cmdline} ${cmdline}.bak"
        run "sudo sed -i 's/console=serial0,[0-9]* //' ${cmdline}"
        info "usunięto console=serial0 z ${cmdline} (kopia: ${cmdline}.bak) - wymaga reboota"
    else
        info "console=serial0 nieobecne w ${cmdline} - nic do zrobienia"
    fi

    if [ "$DRY_RUN" != 1 ]; then
        [ -e /dev/serial0 ] && info "  [ok]   /dev/serial0 -> $(readlink -f /dev/serial0)" \
                            || info "  [brak] /dev/serial0"
    fi
}

# ------------------------------------------------------------------ faza 3.6
provision_wifi_powersave() {
    say "Faza 3.6: hotspot, przelacznik kabel/hotspot, power_save"

    # --- 1. Hotspot -------------------------------------------------------
    # Plytka wystawia wlasna siec, gdy nie ma kabla. Netplan z backendem
    # networkd NIE UMIE trybu AP, wiec robi to hostapd, a wlan0 celowo zostaje
    # poza netplanem; adres i serwer DHCP daje wlasny plik .network.
    run "sudo install -m600 ${HERE}/hostapd.conf /etc/hostapd/hostapd.conf"
    if [ -n "${BATHSET_AP_PASS:-}" ]; then
        run "sudo sed -i 's/^wpa_passphrase=.*/wpa_passphrase=${BATHSET_AP_PASS}/' /etc/hostapd/hostapd.conf"
        info "haslo hotspotu wziete z BATHSET_AP_PASS"
    fi
    run "sudo install -m644 ${HERE}/20-wlan0-ap.network /etc/systemd/network/20-wlan0-ap.network"

    # cloud-init regenerowalby 50-cloud-init.yaml z wlan0 jako KLIENTEM domowego
    # WiFi, a wpa_supplicant odebralby wtedy interfejs hostapd. Objaw: hotspot
    # startuje i po chwili znika bez bledu.
    if [ ! -f /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg ]; then
        run "sudo cp -n /etc/netplan/50-cloud-init.yaml /etc/bathset/50-cloud-init.yaml.klient-wifi 2>/dev/null || true"
        if [ "$DRY_RUN" != 1 ]; then
            echo 'network: {config: disabled}' | sudo tee /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg >/dev/null
        fi
        info "cloud-init odciety od konfiguracji sieci (kopia oryginalu w /etc/bathset/)"
    fi
    # Klient WiFi bilby sie z hostapd o wlan0.
    run "sudo systemctl mask --now netplan-wpa-wlan0.service || true"

    # --- 2. Przelacznik kabel/hotspot ------------------------------------
    # hostapd NIE jest wlaczany samodzielnie - o tym, czy ma chodzic, decyduje
    # bathset-netmode na podstawie obecnosci kabla. Gdyby hostapd startowal sam,
    # wstawalby przed ta decyzja i wystawial siec mimo wpietego kabla.
    run "sudo systemctl unmask hostapd || true"
    run "sudo systemctl disable hostapd || true"
    run "sudo install -m755 ${HERE}/bathset-netmode.sh /usr/local/sbin/bathset-netmode"
    run "sudo install -m644 ${HERE}/bathset-netmode.service /etc/systemd/system/bathset-netmode.service"

    # --- 3. Oszczedzanie energii WiFi ------------------------------------
    # brcmfmac wstaje z power_save=on i usypia radio miedzy ramkami. Koszt
    # zmierzony 2026-08-30 (ping ze stacji): on -> avg 34,5 ms, max 168 ms,
    # mdev 37 ms; off -> avg 12-20 ms, mdev 3,6 ms. To wlasnie ten narzut kazal
    # w sierpniu 2026 odrzucic WiFi jako droge do plytki (zmierzono wtedy
    # avg 111,8 ms), nie sprawdziwszy power_save. Ustawienie zyje tylko do
    # restartu, stad jednostka.
    local unit=/etc/systemd/system/wifi-powersave-off.service
    local block
    block="$(cat <<'EOF'
[Unit]
Description=Wylaczenie oszczedzania energii na wlan0 (opoznienia DDS)
After=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
# `|| true` swiadomie: brak wlan0 nie moze wywalac rozruchu.
ExecStart=/bin/sh -c '/usr/sbin/iw dev wlan0 set power_save off || true'

[Install]
WantedBy=multi-user.target
EOF
)"
    if [ "$DRY_RUN" = 1 ]; then
        printf '   [dry-run] zapis %s:\n%s\n' "$unit" "$block"
        return 0
    fi
    printf '%s\n' "$block" | sudo tee "$unit" >/dev/null
    sudo chmod 644 "$unit"
    run "sudo systemctl daemon-reload"
    run "sudo systemctl enable --now wifi-powersave-off.service"
    run "sudo systemctl enable --now bathset-netmode.service"
    info "tryb sieci: $(systemctl is-active hostapd >/dev/null && echo 'hotspot' || echo 'kabel')"
}

# --------------------------------------------------------------------- faza 4
provision_env() {
    say "Faza 4: środowisko ROS w każdej sesji"
    # Profil Fast DDS z discovery unicastem - bez niego stacja nie zobaczy
    # topików płytki, bo multicast między nimi nie przechodzi (pomiar 2026-08-17,
    # szczegóły w samym pliku).
    run "sudo install -d -m755 /etc/bathset"
    run "sudo install -m644 ${HERE}/fastdds_eth.xml /etc/bathset/fastdds_eth.xml"

    # profile.d, żeby `ros2 topic list` działało od razu po zalogowaniu - bez
    # tego każda ręczna sesja ląduje bez ROS-a i diagnostyka jest zgadywanką.
    local block
    block="$(cat <<EOF
# Środowisko ROS dla zestawu batymetrycznego (wgrywane przez deploy/rpi/install.sh)
source /opt/ros/${ROS_DISTRO}/setup.bash
[ -f "\$HOME/bathset_ws/install/setup.bash" ] && source "\$HOME/bathset_ws/install/setup.bash"
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}
# Discovery unicastem do stacji. UWAGA: to dotyczy tylko sesji interaktywnych -
# stack odpalany przez deploy.sh --run startuje z powłoki NIELOGOWANEJ, więc
# ten sam eksport jest powtórzony w generowanym ~/run_stack.sh.
[ -f /etc/bathset/fastdds_eth.xml ] && \\
    export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/bathset/fastdds_eth.xml
EOF
)"
    if [ "$DRY_RUN" = 1 ]; then
        printf '   [dry-run] zapis /etc/profile.d/bathset.sh:\n%s\n' "$block"
    else
        printf '%s\n' "$block" | sudo tee /etc/profile.d/bathset.sh >/dev/null
        sudo chmod 644 /etc/profile.d/bathset.sh
        info "zapisane /etc/profile.d/bathset.sh"
    fi
}

# --------------------------------------------------------------------- faza 5
provision_zram() {
    say "Faza 5: zram (skompresowany swap w RAM)"
    # Zmierzone na tej płytce: szczyt pojedynczego cc1plus to 574 MB, a
    # najniższe 'available' w trakcie builda spadło do 86 MB. Build przechodzi,
    # ale margines jest cienki i wystarczy stack ROS-a w tle, żeby zrobiło się
    # ciasno. zram daje zapas bez ANI JEDNEGO zapisu na kartę SD - swapfile na
    # SD świadomie odrzucony: karta ma skończony TBW, a losowy zapis rzędu
    # 1-3 MB/s zamienia swapowanie w zawieszenie. Uszkodzenie karty to zresztą
    # najczęstsza przyczyna zgonu Pi w terenie, a ta płytka jedzie na wodę.
    if swapon --show 2>/dev/null | grep -q zram; then
        info "zram już aktywny"
        return
    fi
    # Kernel raspi (5.15.0-*-raspi) NIE ma modułu zram w obrazie bazowym - leży
    # w linux-modules-extra. Bez tej paczki zramswap.service pada na
    # "modprobe: FATAL: Module zram not found", a samo zram-tools instaluje się
    # bez słowa skargi, więc objaw wychodzi dopiero przy starcie usługi.
    #
    # WERSJA PRZYPIĘTA DO $(uname -r), nie meta: meta `linux-modules-extra-raspi`
    # śledzi NAJNOWSZY kernel z repo, więc na płytce z jądrem starszym niż
    # bieżące (u nas: działa 5.15.0-1061, meta ciągnie 5.15.0-1105) wrzuca
    # moduły do katalogu innego jądra i modprobe dalej ich nie widzi - zram
    # ruszyłby dopiero po aktualizacji kernela i reboocie. Przypięta wersja
    # trafia do /lib/modules/$(uname -r) i działa od razu, bez restartu.
    # Kosztem jest to, że po podbiciu jądra trzeba ją doinstalować ponownie -
    # ten skrypt jest idempotentny, więc wystarczy puścić go jeszcze raz.
    if ! modinfo zram >/dev/null 2>&1; then
        info "brak modułu zram - instaluję linux-modules-extra-$(uname -r)"
        run "sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq linux-modules-extra-\$(uname -r)"
    fi
    run "sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq zram-tools"
    if [ "$DRY_RUN" != 1 ]; then
        printf 'ALGO=zstd\nPERCENT=50\n' | sudo tee /etc/default/zramswap >/dev/null
        sudo modprobe zram 2>/dev/null || true
        sudo systemctl restart zramswap 2>/dev/null || true
        sleep 2
        if swapon --show 2>/dev/null | grep -q zram; then
            info "zram aktywny: $(swapon --show=NAME,SIZE --noheadings | tr '\n' ' ')"
        else
            info "UWAGA: zram się nie podniósł - sprawdź 'journalctl -u zramswap'"
        fi
    fi
}

# ---------------------------------------------------------------------- main
[ "$DRY_RUN" = 1 ] && say "DRY RUN - nic nie zostanie zmienione"
[ "$SKIP_REPO"     = 1 ] || provision_repo
[ "$SKIP_PACKAGES" = 1 ] || provision_packages
[ "$SKIP_UDEV"     = 1 ] || provision_udev
[ "$SKIP_UART"     = 1 ] || provision_uart
[ "$SKIP_WIFI"     = 1 ] || provision_wifi_powersave
[ "$SKIP_ENV"      = 1 ] || provision_env
[ "$SKIP_ZRAM"     = 1 ] || provision_zram

# Wymuszenie zapisu na kartę: przy nagłym odcięciu zasilania ext4 z opóźnioną
# alokacją potrafi zostawić świeżo zapisane pliki konfiguracyjne jako zerowe.
run "sync"
say "Przygotowanie płytki zakończone."
