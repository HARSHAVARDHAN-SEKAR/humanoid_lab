#!/usr/bin/env bash
# Run on Ubuntu host AFTER starting screen recording and seeing MuJoCo window.
set -Eeuo pipefail
if ! docker ps --format '{{.Names}}' | grep -Fxq humanoid-walk; then
  echo 'ERROR: humanoid-walk is not running. Start Stage 4 GUI first.' >&2
  exit 1
fi
docker exec humanoid-walk bash -lc '
set -Eeo pipefail
source /opt/ros/humble/setup.bash
source /workspace/humanoid_lab_ws/install/setup.bash
set -u
# Keep four short supported steps, 2 s each, to capture the entire sequence.
ros2 param set /gait_planner step_duration_s 2.0
printf "\nRequesting mission START...\n"
ros2 topic pub --once /mission_manager/start std_msgs/msg/Bool "{data: true}"
sleep 1
printf "\nRequesting 4 supported swing steps...\n"
ros2 topic pub --once /mission_manager/begin_walk std_msgs/msg/Bool "{data: true}"
sleep 1
printf "\nCurrent mission phase:\n"
timeout 5 ros2 topic echo --once /mission_manager/phase || true
printf "\nCurrent safety state:\n"
timeout 5 ros2 topic echo --once /safety_supervisor/state || true
printf "\nCurrent motion controller mode:\n"
timeout 5 ros2 topic echo --once /motion_controller/mode || true
'
