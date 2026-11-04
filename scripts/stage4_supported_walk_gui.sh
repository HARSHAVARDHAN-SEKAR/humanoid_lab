#!/usr/bin/env bash
# Stage 4: show the actual ROS 2 supported-stepping joint feedback in MuJoCo.
# Development support rig only: the torso is fixed, NOT free-base walking.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

for required in scripts/stage4_supported_live_viewer.py scripts/stage4_trigger_steps.sh src/humanoid_sim/mujoco_models/humanoid_biped_supported.xml; do
  if [[ ! -f "$required" ]]; then
    echo "Missing tracked file: $required" >&2
    exit 1
  fi
done
if ! command -v docker >/dev/null 2>&1 || ! docker info >/dev/null 2>&1; then
  echo 'Docker daemon is not accessible. Start Docker and check permissions.' >&2
  exit 1
fi
if [[ -z "${DISPLAY:-}" || ! -d /tmp/.X11-unix ]]; then
  echo 'Requires an Ubuntu X11/XWayland desktop with a DISPLAY.' >&2
  exit 1
fi
if ! command -v xhost >/dev/null 2>&1; then
  echo 'Install X11 permissions utility: sudo apt install x11-xserver-utils' >&2
  exit 1
fi
if docker ps -a --format '{{.Names}}' | grep -Fxq humanoid-walk; then
  echo 'Container humanoid-walk already exists. Close the previous viewer before restarting.' >&2
  echo 'To remove it explicitly: docker rm -f humanoid-walk' >&2
  exit 1
fi
if ! docker image inspect humanoid-lab:humble >/dev/null 2>&1; then
  echo 'Docker image not built. Run make test once from the repository root.' >&2
  exit 1
fi

mkdir -p results/local
if ! xhost +si:localuser:root >/dev/null; then
  echo 'Could not grant the local container GUI access through X11.' >&2
  exit 1
fi
cleanup_xhost() { xhost -si:localuser:root >/dev/null 2>&1 || true; }
trap cleanup_xhost EXIT

extra_devices=()
if [[ -d /dev/dri ]]; then extra_devices+=(--device=/dev/dri:/dev/dri); fi

cat <<'MSG'

STAGE 4 — SUPPORTED WALKING / LEG MOTION DEMO
The torso is WORLD-FIXED. This does not prove unsupported walking.
When the 3D MuJoCo window appears, start your Ubuntu screen recording.
In a second host terminal run:
    bash scripts/stage4_trigger_steps.sh
The launch log will be copied into results/local/ on exit.

MSG

docker run --rm -it \
  --name humanoid-walk \
  -e DISPLAY="$DISPLAY" \
  -e MUJOCO_GL=glfw \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID=63 \
  -e PYTHONUNBUFFERED=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$ROOT/scripts/stage4_supported_live_viewer.py:/scripts/stage4_supported_live_viewer.py:ro" \
  -v "$ROOT/src/humanoid_sim/mujoco_models:/models:ro" \
  -v "$ROOT/results/local:/results:rw" \
  "${extra_devices[@]}" \
  humanoid-lab:humble bash -lc '
    # ROS environment setup needs nounset disabled during sourcing.
    set -Eeo pipefail
    source /opt/ros/humble/setup.bash
    source /workspace/humanoid_lab_ws/install/setup.bash
    set -u

    ros2 launch humanoid_bringup stage4_walking.launch.py \
      model_file:=humanoid_biped_supported.xml \
      num_steps:=4 autostart:=false \
      > /tmp/stage4_walk_launch.log 2>&1 &
    launch_pid=$!
    cleanup() {
      kill -INT "$launch_pid" >/dev/null 2>&1 || true
      wait "$launch_pid" >/dev/null 2>&1 || true
      cp /tmp/stage4_walk_launch.log /results/stage4_walk_launch.log 2>/dev/null || true
      echo "Stage 4 launch log: results/local/stage4_walk_launch.log"
    }
    trap cleanup EXIT

    if ! python3 /scripts/stage4_supported_live_viewer.py; then
      echo "=== Stage 4 ROS launch log (last 70 lines) ==="
      tail -n 70 /tmp/stage4_walk_launch.log || true
      exit 1
    fi
  '
