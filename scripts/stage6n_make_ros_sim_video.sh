#!/usr/bin/env bash
# Replay actual ROS-controlled MuJoCo free-base poses; not an independent physics run.
# Works from any clone location / branch. Never modifies tracked source files.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
MODE="${1:-both}"
case "$MODE" in
  40|-40|both) ;;
  *) echo "Usage: bash scripts/stage6n_make_ros_sim_video.sh [40|-40|both]" >&2; exit 2 ;;
esac
for required in Makefile scripts/stage6n_capture_render.py \
    src/humanoid_sim/mujoco_models/humanoid_biped.xml \
    src/humanoid_sim/humanoid_sim/mujoco_bridge_node.py; do
  if [[ ! -f "$required" ]]; then echo "Missing tracked file: $required" >&2; exit 1; fi
done
if ! grep -Fq 'debug_freebase_qpos' src/humanoid_sim/humanoid_sim/mujoco_bridge_node.py; then
  echo 'Missing simulation-only ROS debug qpos topic in MuJoCo bridge.' >&2
  exit 1
fi
if ! command -v docker >/dev/null 2>&1 || ! docker info >/dev/null 2>&1; then
  echo 'Docker daemon inaccessible; run this from the Ubuntu host with Docker running.' >&2
  exit 1
fi
mkdir -p results/local

# Compile static script syntax before invoking the heavier build and simulation.
python3 -m py_compile scripts/stage6n_capture_render.py src/humanoid_sim/humanoid_sim/mujoco_bridge_node.py

echo 'Building image and running automated tests before ROS simulation...'
make test

run_one() {
  local force="$1" case_name="$2" container="humanoid-stage6n"
  echo
  echo "==== Stage 6N ROS-controlled simulation: ${case_name} ===="
  if docker ps -a --format '{{.Names}}' | grep -Fxq "$container"; then
    echo "Another ${container} container exists; close it before starting a new trial." >&2
    exit 1
  fi
  docker run --rm --name "$container" \
    -v "$ROOT/scripts:/scripts:ro" \
    -v "$ROOT/src/humanoid_sim/mujoco_models:/models:ro" \
    -v "$ROOT/results/local:/results:rw" \
    -e STAGE6N_FORCE="$force" -e STAGE6N_CASE="$case_name" \
    humanoid-lab:humble bash -c '
      set -eo pipefail
      source /opt/ros/humble/setup.bash
      source /workspace/humanoid_lab_ws/install/setup.bash
      ros2 launch humanoid_bringup stage6i_com_balance.launch.py \
        > /tmp/stage6n_ros_launch.log 2>&1 &
      launch_pid=$!
      cleanup() {
        kill -INT "$launch_pid" >/dev/null 2>&1 || true
        wait "$launch_pid" >/dev/null 2>&1 || true
      }
      trap cleanup EXIT

      python3 /scripts/stage6n_capture_render.py capture \
        --force "$STAGE6N_FORCE" \
        --output "/results/stage6n_ros_${STAGE6N_CASE}.npz"

      if ! grep -F "Applying $(printf "%.1f" "$STAGE6N_FORCE") N X-force" \
          /tmp/stage6n_ros_launch.log; then
        echo "ERROR: MuJoCo bridge did not confirm the push." >&2
        tail -n 70 /tmp/stage6n_ros_launch.log >&2
        exit 3
      fi

      echo "Rendering actual recorded poses to video (headless)..."
      if ! MUJOCO_GL=osmesa python3 /scripts/stage6n_capture_render.py render \
          --input "/results/stage6n_ros_${STAGE6N_CASE}.npz" \
          --output "/results/stage6n_ros_${STAGE6N_CASE}.mp4"; then
        echo "Trying EGL renderer..." >&2
        MUJOCO_GL=egl python3 /scripts/stage6n_capture_render.py render \
          --input "/results/stage6n_ros_${STAGE6N_CASE}.npz" \
          --output "/results/stage6n_ros_${STAGE6N_CASE}.mp4"
      fi
    '
  test -s "$ROOT/results/local/stage6n_ros_${case_name}.mp4"
  echo "VIDEO: $ROOT/results/local/stage6n_ros_${case_name}.mp4"
}

case "$MODE" in
  40) run_one 40 40N_forward ;;
  -40) run_one -40 40N_backward ;;
  both) run_one 40 40N_forward; run_one -40 40N_backward ;;
esac

echo 'Stage 6N complete. Videos saved under results/local/.'
echo 'This shows ROS-controlled simulator replay, not physically validated walking.'
