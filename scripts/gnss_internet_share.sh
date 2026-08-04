#!/bin/bash
# gnss_internet_share.sh - udostępnia internet hosta odbiornikowi Septentrio
# mosaic-H przez ethernet-over-USB, żeby jego WBUDOWANY klient NTRIP mógł sam
# pobierać korekcje RTK z castera (ASG-EUPOS).
#
# Dlaczego w ogóle: mosaic-H nie ma własnego modemu. Po podpięciu USB wystawia
# interfejs sieciowy, na którym domyślnie SAM jest serwerem (192.168.3.1) i nie
# ma wyjścia na świat. Po komendzie `setUSBInternetAccess, on` role się
# odwracają: odbiornik staje się klientem DHCP i oczekuje, że to host da mu
# adres, bramę, DNS i NAT. Ten skrypt dostarcza tę drugą stronę.
#
# KOLEJNOŚĆ MA ZNACZENIE:
#   1. najpierw `up` (tu) - dnsmasq musi już czekać z adresem,
#   2. dopiero potem `gnss_rtk.py configure` (włącza suia i konfiguruje NTRIP).
# Po włączeniu suia odbiornik PRZESTAJE odpowiadać pod 192.168.3.1 - web UI
# jest wtedy pod adresem z puli DHCP (pokazuje go `status`).
#
# Użycie:
#   sudo ./scripts/gnss_internet_share.sh up [uplink]  - włącz udostępnianie
#   sudo ./scripts/gnss_internet_share.sh down         - wyłącz i posprzątaj
#   ./scripts/gnss_internet_share.sh status            - co jest podniesione,
#                                                        jaki adres ma odbiornik
#   sudo ./scripts/gnss_internet_share.sh install-service
#                                        - usługa systemd odpalana przez udev
#                                          przy podpięciu odbiornika (na łódce
#                                          po każdym boocie samo się podnosi)
#   sudo ./scripts/gnss_internet_share.sh uninstall-service
#
# `uplink` to interfejs z internetem (domyślnie brany z trasy domyślnej) -
# na łódce zwykle wlan0 (hotspot z telefonu), na biurku eth.
#
# Idempotentny: `up` można wołać wielokrotnie, także po repluggu odbiornika.
set -euo pipefail

# Pula dla łącza host <-> odbiornik. Celowo nie 192.168.x - żeby nie kolidować
# z siecią domową ani z fabrycznym 192.168.3.0/24 odbiornika.
SUBNET="172.20.20.0/24"
HOST_IP="172.20.20.1"
DHCP_FROM="172.20.20.10"
DHCP_TO="172.20.20.20"
NETMASK="255.255.255.0"

PID_FILE="/run/gnss-rtk-dnsmasq.pid"
LEASE_FILE="/run/gnss-rtk-dnsmasq.leases"

USB_VENDOR="152a"
USB_PRODUCT="85c0"

usage() {
    echo "Użycie: $0 {up [uplink]|down|status|install-service|uninstall-service}"
    exit 1
}

need_root() {
    if [ "$(id -u)" -ne 0 ]; then
        echo "[gnss-share] Ta operacja wymaga roota: sudo $0 $*" >&2
        exit 1
    fi
}

# Znajduje interfejs sieciowy wystawiany przez odbiornik (nazwa enx... zmienia
# się z adresem MAC, więc szukamy po ID USB, nie po nazwie).
find_gnss_iface() {
    local net dev
    for net in /sys/class/net/*; do
        [ -e "$net/device" ] || continue
        dev="$(readlink -f "$net/device")"
        while [ "$dev" != "/" ] && [ -n "$dev" ]; do
            if [ -r "$dev/idVendor" ] && [ -r "$dev/idProduct" ]; then
                if [ "$(cat "$dev/idVendor")" = "$USB_VENDOR" ] &&
                   [ "$(cat "$dev/idProduct")" = "$USB_PRODUCT" ]; then
                    basename "$net"
                    return 0
                fi
                break
            fi
            dev="$(dirname "$dev")"
        done
    done
    return 1
}

default_uplink() {
    ip route show default 2>/dev/null | awk '{print $5; exit}'
}

# iptables -C zwraca 0 gdy reguła już jest - dzięki temu `up` nie mnoży reguł.
ensure_rule() {
    local table="$1"; shift
    if ! iptables -t "$table" -C "$@" 2>/dev/null; then
        iptables -t "$table" -A "$@"
    fi
}

drop_rule() {
    local table="$1"; shift
    while iptables -t "$table" -C "$@" 2>/dev/null; do
        iptables -t "$table" -D "$@"
    done
}

# Usuwa nasze reguły NAT/FORWARD wskazujące na INNY uplink niż podany.
# Nasze rozpoznajemy po podsieci (NAT) i po interfejsie odbiornika (FORWARD),
# więc cudze reguły zostają nietknięte.
drop_stale_nat_rules() {
    local keep="$1" iface="$2" rule

    iptables -t nat -S POSTROUTING 2>/dev/null | grep -- "-s $SUBNET" | while read -r rule; do
        case "$rule" in
            *"-o $keep "*|*"-o $keep") continue ;;
        esac
        # shellcheck disable=SC2086
        iptables -t nat ${rule/#-A/-D} 2>/dev/null || true
    done

    iptables -S FORWARD 2>/dev/null | grep -E -- "(-i|-o) $iface( |\$)" | while read -r rule; do
        case "$rule" in
            *"$keep"*) continue ;;
        esac
        # shellcheck disable=SC2086
        iptables ${rule/#-A/-D} 2>/dev/null || true
    done
}

cmd_up() {
    need_root up

    local iface uplink
    if ! iface="$(find_gnss_iface)"; then
        echo "[gnss-share] Nie znaleziono interfejsu sieciowego odbiornika ($USB_VENDOR:$USB_PRODUCT)." >&2
        echo "             Sprawdź, czy mosaic-H jest podpięty (lsusb) - ethernet-over-USB pojawia się razem z portami CDC." >&2
        exit 1
    fi

    uplink="${1:-$(default_uplink)}"
    if [ -z "$uplink" ]; then
        echo "[gnss-share] Brak trasy domyślnej - podaj interfejs z internetem: $0 up wlan0" >&2
        exit 1
    fi
    if [ "$uplink" = "$iface" ]; then
        echo "[gnss-share] Trasa domyślna prowadzi przez interfejs odbiornika ($iface) - podaj właściwy uplink ręcznie." >&2
        exit 1
    fi

    if ! command -v dnsmasq >/dev/null 2>&1; then
        echo "[gnss-share] Brak dnsmasq (serwer DHCP+DNS dla odbiornika): sudo apt install dnsmasq" >&2
        exit 1
    fi

    echo "[gnss-share] Odbiornik na interfejsie $iface, internet przez $uplink."

    sysctl -q -w net.ipv4.ip_forward=1
    ip addr replace "$HOST_IP/24" dev "$iface"
    ip link set "$iface" up

    # Reguły spod poprzedniego uplinku muszą zniknąć: przy przesiadce z
    # ethernetu na hotspot telefonu (czyli zawsze, gdy wyjeżdżamy w teren)
    # zostałby MASQUERADE na nieistniejący już interfejs i NAT by nie działał.
    drop_stale_nat_rules "$uplink" "$iface"
    ensure_rule nat POSTROUTING -s "$SUBNET" -o "$uplink" -j MASQUERADE
    # Docker ustawia politykę łańcucha FORWARD na DROP, więc same reguły NAT
    # nie wystarczą - ruch trzeba przepuścić jawnie w obie strony.
    ensure_rule filter FORWARD -i "$iface" -o "$uplink" -j ACCEPT
    ensure_rule filter FORWARD -i "$uplink" -o "$iface" -m state --state RELATED,ESTABLISHED -j ACCEPT

    # Świeży start: stara instancja mogła zostać przypięta do nieistniejącego
    # już interfejsu (po repluggu nazwa enx... bywa ta sama, ale indeks nie).
    stop_dnsmasq

    # --bind-interfaces + --interface: nie wchodzimy w drogę systemowemu
    # dnsmasq/systemd-resolved, słuchamy wyłącznie na łączu do odbiornika.
    # dnsmasq domyślnie rozgłasza siebie jako bramę i DNS (opcje 3 i 6),
    # a zapytania DNS forwarduje do resolwerów hosta.
    if ! dnsmasq \
        --conf-file=/dev/null \
        --pid-file="$PID_FILE" \
        --dhcp-leasefile="$LEASE_FILE" \
        --interface="$iface" \
        --except-interface=lo \
        --bind-interfaces \
        --dhcp-authoritative \
        --dhcp-range="$DHCP_FROM,$DHCP_TO,$NETMASK,1h"; then
        echo "[gnss-share] dnsmasq nie wystartował - port 53 na $iface zajęty przez inny resolwer?" >&2
        exit 1
    fi

    echo "[gnss-share] Gotowe. Odbiornik dostanie adres z puli $DHCP_FROM-$DHCP_TO."
    echo "[gnss-share] Teraz włącz dostęp po stronie odbiornika:"
    echo "             ./scripts/gnss_rtk.py configure   (albo: internet-on)"
}

stop_dnsmasq() {
    if [ -f "$PID_FILE" ]; then
        local pid
        pid="$(cat "$PID_FILE" 2>/dev/null || true)"
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
        rm -f "$PID_FILE"
    fi
}

cmd_down() {
    need_root down

    stop_dnsmasq

    local iface uplink
    iface="$(find_gnss_iface || true)"
    uplink="$(default_uplink || true)"

    if [ -n "$uplink" ]; then
        drop_rule nat POSTROUTING -s "$SUBNET" -o "$uplink" -j MASQUERADE
        if [ -n "$iface" ]; then
            drop_rule filter FORWARD -i "$iface" -o "$uplink" -j ACCEPT
            drop_rule filter FORWARD -i "$uplink" -o "$iface" -m state --state RELATED,ESTABLISHED -j ACCEPT
        fi
    fi

    if [ -n "$iface" ]; then
        ip addr del "$HOST_IP/24" dev "$iface" 2>/dev/null || true
    fi

    echo "[gnss-share] Udostępnianie wyłączone. (Po stronie odbiornika: ./scripts/gnss_rtk.py internet-off)"
}

cmd_status() {
    local iface
    if ! iface="$(find_gnss_iface)"; then
        echo "  odbiornik:  NIE WIDAĆ interfejsu sieciowego ($USB_VENDOR:$USB_PRODUCT)"
        return 1
    fi
    echo "  interfejs:  $iface"
    echo "  adres IP:   $(ip -br addr show "$iface" 2>/dev/null | awk '{$1=$1};1')"
    echo "  forward:    $(cat /proc/sys/net/ipv4/ip_forward)"

    # /proc/<pid>, a nie `kill -0`: dnsmasq chodzi jako root, więc zwykły
    # użytkownik dostaje od kill EPERM i widziałby "NIE DZIAŁA" mimo że działa.
    if [ -f "$PID_FILE" ] && [ -d "/proc/$(cat "$PID_FILE" 2>/dev/null)" ]; then
        echo "  dnsmasq:    działa (pid $(cat "$PID_FILE"))"
    else
        echo "  dnsmasq:    NIE DZIAŁA"
    fi

    if [ -s "$LEASE_FILE" ]; then
        # Format leases: <expiry> <mac> <ip> <hostname> <client-id>
        echo "  odbiornik dostał adres:"
        awk '{printf "    %s  (%s, %s)\n", $3, $4, $2}' "$LEASE_FILE"
        echo "  web UI odbiornika: http://$(awk 'NR==1{print $3}' "$LEASE_FILE")"
    else
        echo "  dzierżawy:  brak - odbiornik jeszcze nie poprosił o adres"
        echo "              (czy włączone jest 'setUSBInternetAccess, on'? patrz gnss_rtk.py status)"
    fi
}

SERVICE_FILE="/etc/systemd/system/gnss-rtk-share.service"
DEFAULTS_FILE="/etc/default/gnss-rtk-share"

cmd_install_service() {
    need_root install-service

    local self
    self="$(readlink -f "$0")"

    if [ ! -e "$DEFAULTS_FILE" ]; then
        cat >"$DEFAULTS_FILE" <<'EOF'
# Interfejs z internetem, przez który odbiornik GNSS ma wychodzić na świat.
# Puste = weź z trasy domyślnej. Na łódce zwykle wlan0 (hotspot z telefonu).
RTK_UPLINK=
EOF
    fi

    # Type=oneshot + RemainAfterExit: dnsmasq odchodzi w tło sam, więc usługa
    # tylko rozstawia konfigurację. Uruchamia ją udev w chwili, gdy pojawi się
    # interfejs sieciowy odbiornika - dlatego nie ma tu WantedBy/enable.
    cat >"$SERVICE_FILE" <<EOF
[Unit]
Description=Udostępnianie internetu odbiornikowi GNSS (RTK/NTRIP)
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
EnvironmentFile=-$DEFAULTS_FILE
ExecStart=$self up \${RTK_UPLINK}
ExecStop=$self down
EOF

    systemctl daemon-reload
    echo "[gnss-share] Zainstalowano $SERVICE_FILE"
    echo "[gnss-share] Uplink ustawisz w $DEFAULTS_FILE (teraz: auto z trasy domyślnej)."
    echo "[gnss-share] Usługę odpali udev przy podpięciu odbiornika - upewnij się,"
    echo "             że reguły są aktualne: ./docker/run_real.sh install-udev"
}

cmd_uninstall_service() {
    need_root uninstall-service
    systemctl stop gnss-rtk-share.service 2>/dev/null || true
    rm -f "$SERVICE_FILE"
    systemctl daemon-reload
    echo "[gnss-share] Usunięto $SERVICE_FILE (plik $DEFAULTS_FILE zostawiam)."
}

case "${1:-}" in
    up) shift; cmd_up "$@" ;;
    down) cmd_down ;;
    status) cmd_status ;;
    install-service) cmd_install_service ;;
    uninstall-service) cmd_uninstall_service ;;
    *) usage ;;
esac
