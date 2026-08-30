#!/bin/bash
# bathset-netmode - wybor drogi komunikacji plytki: KABEL albo WLASNY HOTSPOT.
#
# Regula: jest kabel -> gadamy po eth0, hotspot zgaszony. Nie ma kabla -> plytka
# wystawia siec `bathset` i przyjmuje polaczenie ze stacji.
#
# DLACZEGO POLLING, A NIE ZDARZENIA UDEV: udev nie emituje zdarzen przy samej
# zmianie carrier dla smsc95xx (ethernet Pi 3B wisi na USB), a networkd-dispatcher
# nie jest zainstalowany. Odczyt /sys/class/net/eth0/carrier co kilka sekund jest
# prymitywny, ale rozstrzyga to samo i nie ma zadnych zaleznosci.
#
# CARRIER CZYTA SIE TYLKO NA INTERFEJSIE ADMIN-UP - dlatego petla podnosi eth0
# przy kazdym obiegu. Na interfejsie zgaszonym `carrier` zwraca 0 niezaleznie od
# tego, czy wtyczka jest w gniezdzie, czyli objaw "nie ma kabla" bylby falszywy.
#
# DLACZEGO NIE SAM CARRIER DECYDUJE: kabel wpiety w switch albo router, ktory nie
# da adresu, to carrier=1 i zero lacznosci ze stacja. Gdyby hotspot padal na sam
# carrier, plytka stawalaby sie NIEOSIAGALNA - bez hotspotu i bez uzytecznego
# kabla. Dlatego do trybu kablowego wymagamy adresu IPv4, a gdy nie przyjdzie w
# ciagu GRACE sekund, wracamy do hotspotu. Kabel donikad nie odcina plytki.

set -u

ETH=eth0
WLAN=wlan0
POLL=3          # co ile sekund sprawdzamy stan wtyczki
GRACE=30        # ile czekamy na adres z DHCP, zanim uznamy kabel za bezuzyteczny

has_carrier() { [ "$(cat "/sys/class/net/${ETH}/carrier" 2>/dev/null || echo 0)" = "1" ]; }
has_ipv4()    { ip -4 addr show dev "$ETH" 2>/dev/null | grep -q "inet "; }

mode=""
waiting_since=0

while :; do
    ip link set "$ETH" up 2>/dev/null || true

    if has_carrier; then
        if has_ipv4; then
            want=eth
            waiting_since=0
        else
            now=$(date +%s)
            [ "$waiting_since" = 0 ] && waiting_since=$now
            if [ $(( now - waiting_since )) -ge "$GRACE" ]; then
                want=ap          # kabel jest, ale nie prowadzi donikad
            else
                want="${mode:-ap}"   # DHCP jeszcze moze przyjsc - nic nie ruszamy
            fi
        fi
    else
        want=ap
        waiting_since=0
    fi

    if [ "$want" != "$mode" ]; then
        case "$want" in
            eth) systemctl stop hostapd  2>/dev/null || true ;;
            ap)  systemctl start hostapd 2>/dev/null || true ;;
        esac
        mode="$want"
        logger -t bathset-netmode "tryb: ${mode} (carrier=$(cat "/sys/class/net/${ETH}/carrier" 2>/dev/null || echo ?))"
    fi

    sleep "$POLL"
done
