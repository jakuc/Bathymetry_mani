#!/bin/bash
# run_real.sh - wrapper do budowania obrazu i wchodzenia do kontenera real-hardware
# (docker-compose.real.yml / Dockerfile.real - bez Isaac Sim).
#
# Kontener mani_real jest trwały i chodzi w tle (sleep infinity jako proces
# główny) - "shell" nigdy go nie tworzy na nowo ani nie usuwa po wyjściu z bash,
# tylko doczepia kolejną interaktywną powłokę (docker exec). Dzięki temu można
# wejść z dowolnej liczby terminali naraz - np. jeden z `ros2 launch`, drugi
# do podglądania topików.
#
# Użycie:
#   ./docker/run_real.sh shell   - wejdź do kontenera (tworzy/startuje jeśli
#                                   trzeba, buduje obraz jeśli brak); kolejne
#                                   wywołania z innych terminali tylko doczepiają
#                                   nową powłokę do tego samego, już działającego
#                                   kontenera
#   ./docker/run_real.sh build   - zbuduj obraz mani_ros:real
#   ./docker/run_real.sh stop    - zatrzymaj i usuń trwały kontener mani_real
#                                   (np. żeby "shell" odtworzył go od nowa po
#                                   przebudowaniu obrazu)
#   ./docker/run_real.sh check   - sprawdź (i w razie potrzeby zainstaluj reguły
#                                   udev) stan /dev/u2d2, /dev/echosounder,
#                                   /dev/gnss, /dev/gnss_aux
#   ./docker/run_real.sh install-udev - tylko instalacja/aktualizacja reguł udev
#
# Instalacja reguł udev wymaga sudo - jeśli odpalasz ten skrypt sam, w swoim
# terminalu, sudo normalnie zapyta o hasło. (Nie da się tego zrobić bezobsługowo
# z poziomu sandboksowanego narzędzia bez TTY - stąd czasem robi się to ręcznie.)
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
COMPOSE_FILE="$SCRIPT_DIR/docker-compose.real.yml"
IMAGE="mani_ros:real"
CONTAINER="mani_real"

# Nazwy wolumenów tworzone przez `docker compose -f docker-compose.real.yml ...`
# (prefiks "docker_" bierze się z nazwy katalogu docker/ jako compose project name).
VOLUME_BUILD="docker_ros_ws_build_real"
VOLUME_INSTALL="docker_ros_ws_install_real"

usage() {
    echo "Użycie: $0 {shell|build|stop|check|install-udev}"
    exit 1
}

# Kopiuje brakujące/zmienione reguły udev i przeładowuje demona.
# Idempotentne - bezpieczne do wywoływania za każdym razem.
install_udev_rules() {
    local need_install=0
    for rule in "$REPO_DIR"/udev/*.rules; do
        local target="/etc/udev/rules.d/$(basename "$rule")"
        if ! cmp -s "$rule" "$target" 2>/dev/null; then
            need_install=1
        fi
    done

    if [ "$need_install" -eq 0 ]; then
        return 0
    fi

    echo "[run_real] Instaluję/aktualizuję reguły udev (wymaga sudo)..."
    sudo cp "$REPO_DIR"/udev/*.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    sudo udevadm settle
}

# Zwraca 0 jeśli wszystkie urządzenia są obecne, 1 jeśli czegoś brakuje.
# Wypisuje też listę --device do użycia z tym, co faktycznie jest podpięte.
check_devices() {
    install_udev_rules

    local missing=0
    DEVICE_ARGS=()
    for dev in /dev/u2d2 /dev/echosounder /dev/gnss /dev/gnss_aux /dev/laser; do
        if [ -e "$dev" ]; then
            echo "  [ok]   $dev"
            DEVICE_ARGS+=(--device="$dev:$dev")
        else
            echo "  [brak] $dev"
            missing=1
        fi
    done
    if [ "$missing" -eq 1 ]; then
        echo ""
        echo "Reguły udev są zainstalowane, ale urządzenie się nie pojawiło - sprawdź czy sprzęt jest fizycznie podpięty (lsusb)."
    fi
    return $missing
}

container_running() {
    [ -n "$(docker ps -q -f name="^/${CONTAINER}\$")" ]
}

container_exists() {
    [ -n "$(docker ps -aq -f name="^/${CONTAINER}\$")" ]
}

cmd_build() {
    echo "[run_real] Budowanie obrazu $IMAGE..."
    docker compose -f "$COMPOSE_FILE" build
}

cmd_stop() {
    if container_exists; then
        echo "[run_real] Zatrzymuję i usuwam kontener $CONTAINER..."
        docker rm -f "$CONTAINER" >/dev/null
    else
        echo "[run_real] Kontener $CONTAINER i tak nie istnieje."
    fi
}

# Startuje trwały kontener w tle, jeśli jeszcze nie działa.
ensure_container() {
    if container_running; then
        return 0
    fi

    if container_exists; then
        echo "[run_real] Kontener $CONTAINER istnieje, ale jest zatrzymany - startuję..."
        docker start "$CONTAINER" >/dev/null
        return 0
    fi

    echo "[run_real] Sprawdzam urządzenia:"
    check_devices || echo "[run_real] Kontynuuję - kontener wystartuje tylko z tym, co jest podpięte teraz."
    echo ""

    if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
        echo "[run_real] Obraz $IMAGE nie istnieje - buduję najpierw..."
        cmd_build
    fi

    xhost +local:docker >/dev/null 2>&1 || true

    echo "[run_real] Startuję kontener $CONTAINER w tle..."
    docker run -d \
        --network host \
        -e DISPLAY="${DISPLAY:-}" \
        -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
        -e ROS_LOG_DIR=/workspace/log \
        "${DEVICE_ARGS[@]}" \
        -v "$REPO_DIR/src:/workspace/src" \
        -v "$VOLUME_BUILD:/workspace/build" \
        -v "$VOLUME_INSTALL:/workspace/install" \
        -v "$REPO_DIR/log:/workspace/log" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --name "$CONTAINER" \
        "$IMAGE" sleep infinity >/dev/null
}

cmd_shell() {
    ensure_container
    echo "[run_real] Wchodzę do kontenera $CONTAINER (docker exec)..."
    docker exec -it "$CONTAINER" bash
}

case "${1:-}" in
    shell) cmd_shell ;;
    build) cmd_build ;;
    stop) cmd_stop ;;
    check) check_devices ;;
    install-udev) install_udev_rules ;;
    *) usage ;;
esac
