#!/usr/bin/env bash
# Run *inside* the container. Verifies ROS 2 starts and feedback arrives.
# Does NOT prove supported or unsupported dynamic walking.
# ROS 2 Humble's generated setup.bash can read undefined optional variables.
# Source ROS environments BEFORE enabling `nounset`; keep error/pipe checks on.
set -eo pipefail
set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source /workspace/humanoid_lab_ws/install/setup.bash
set -u
log=/tmp/humanoid_ros_smoke.log
ros2 launch humanoid_bringup stage3_stepping.launch.py \
    use_rviz:=false use_joint_state_publisher_gui:=false \
    autostart_delay_s:=3.0 >"$log" 2>&1 &
pid=$!
cleanup() {
    kill "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
}
trap cleanup EXIT
sleep 5
if ! kill -0 "$pid" 2>/dev/null; then
    echo 'ROS launch exited prematurely' >&2
    tail -100 "$log" >&2
    exit 1
fi
nodes="$(ros2 node list)"
for node in /mujoco_bridge /state_estimator /balance_controller /motion_controller /gait_planner; do
    if ! grep -Fxq "$node" <<<"$nodes"; then
        echo "Missing ROS node: $node" >&2
        tail -100 "$log" >&2
        exit 1
    fi
done
for topic in /joint_states /imu/torso /mujoco_bridge/torso_height /mujoco_bridge/joint_commands /state_estimator/attitude /motion_controller/mode; do
    echo "Checking ROS topic: $topic"
    if ! timeout 15s ros2 topic echo --once "$topic" >/tmp/humanoid_topic_once.txt; then
        echo "No message received from $topic" >&2
        tail -100 "$log" >&2
        exit 1
    fi
done
echo 'PASS: ROS bridge/controller topic-liveness smoke (not a gait success claim)'
