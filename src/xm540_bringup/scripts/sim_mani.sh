#!/bin/bash
set -e

ISAAC_SCRIPT=$(ros2 pkg prefix xm540_bringup)/share/xm540_bringup/isaac/isaac_sim.py
ISAAC_LOG=/workspace/log/isaac_sim.log
VERBOSE=0
export ISAAC_HEADLESS=0
export ISAAC_SIM_SPEED=10

for arg in "$@"; do
    case $arg in
        --verbose|-v)    VERBOSE=1 ;;
        --headless|-H)   ISAAC_HEADLESS=1 ;;
        --speed=*)       ISAAC_SIM_SPEED="${arg#--speed=}" ;;
    esac
done

if awk "BEGIN { exit !($ISAAC_SIM_SPEED >= 5) }" && [ "$ISAAC_HEADLESS" -eq 0 ]; then
    echo "[sim_mani] speed=${ISAAC_SIM_SPEED}× ≥ 5 — automatycznie włączam headless."
    ISAAC_HEADLESS=1
fi

cleanup() {
    echo "[sim_mani] Zatrzymuję Isaac Sim..."
    kill $ISAAC_PID 2>/dev/null || true
    kill $TAIL_PID 2>/dev/null || true
    wait $ISAAC_PID 2>/dev/null || true
}
trap cleanup EXIT INT TERM

mkdir -p /workspace/log

# Symlinki do dużych plików mesh — nie są kopiowane przez colcon, wskazują na src/
MESHES_SHARE=$(ros2 pkg prefix xm540_bringup)/share/xm540_bringup/meshes
MESHES_SRC=$(ros2 pkg prefix xm540_bringup)/../../src/xm540_bringup/meshes
for MESH in big_lake_simp.obj big_lake_simp_tiles; do
    if [ ! -e "$MESHES_SHARE/$MESH" ]; then
        ln -s "$MESHES_SRC/$MESH" "$MESHES_SHARE/$MESH"
        echo "[sim_mani] Symlink: $MESH"
    fi
done

echo "[sim_mani] Uruchamiam Isaac Sim (pełne logi: $ISAAC_LOG)..."
echo 'Yes' | OMNI_KIT_ALLOW_ROOT=1 python3 "$ISAAC_SCRIPT" > "$ISAAC_LOG" 2>&1 &
ISAAC_PID=$!

if [ $VERBOSE -eq 1 ]; then
    tail -f "$ISAAC_LOG" &
else
    tail -f "$ISAAC_LOG" | grep -v -iE "(shader|PSO|HLSL)" &
fi
TAIL_PID=$!

echo "[sim_mani] Uruchamiam węzły ROS2..."
ros2 launch xm540_bringup isaac.launch.py verbose:=$VERBOSE
