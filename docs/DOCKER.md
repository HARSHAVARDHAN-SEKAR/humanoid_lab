# Docker deployment (Ubuntu host)

## Prerequisites

Install Docker Engine on Ubuntu and make sure it is running. First build
requires internet access to download `ros:humble-ros-base`, Ubuntu/ROS apt
packages and Python packages. ROS 2 is installed **inside** Docker; do
not need it on your host. Docker/ROS runtime not independently validated
in the editing environment, so the first local run is your acceptance test.

```bash
docker --version
docker info
make build
make test
make validate-supported
```

The first `make build` can be substantial. Tests in Docker should include
the 8 MuJoCo physics-dependent tests that skipped in the editing sandbox.

## Stages

```bash
make stage1   # simulated joints + TF; no autonomous gait
make stage2   # estimator + balance and IK-to-motor interface
make stage3   # supported two-foot swing sequence, autostarts after 3s
make stage4   # mission manager, safety, six short supported swings
make stage5   # same + CSV logger
```

The default `humanoid_biped_supported.xml` torso is fixed by the physics
model. Even if all commands work, this is **supported stepping**; to
experiment with the unverified floating-base variant:

```bash
make stage4 ARGS="model_file:=humanoid_biped.xml"
```

## Interactive terminal and ROS topics

While one stage is running, in another Ubuntu terminal:

```bash
docker ps
docker exec -it <CONTAINER_ID> bash
source /opt/ros/humble/setup.bash
source /workspace/humanoid_lab_ws/install/setup.bash
ros2 topic list
ros2 topic echo /motion_controller/mode
ros2 topic echo /safety_supervisor/state
ros2 topic echo /mujoco_bridge/torso_height
```

To deliberately inject an IMU delay for Stage 5 from that second shell:

```bash
ros2 topic pub --once /mujoco_bridge/inject_imu_delay std_msgs/msg/Float64 '{data: 0.4}'
ros2 topic pub --once /mujoco_bridge/inject_joint_dropout std_msgs/msg/Bool '{data: true}'
# Restore feedback:
ros2 topic pub --once /mujoco_bridge/inject_joint_dropout std_msgs/msg/Bool '{data: false}'
ros2 topic pub --once /mujoco_bridge/inject_imu_delay std_msgs/msg/Float64 '{data: 0.0}'
ros2 service call /safety_supervisor/reset std_srvs/srv/Trigger '{}'
```

The supervisor can permanently enter SAFE_HOLD after a fault; the reset
service requires valid feedback. A failed reset does not imply the robot
has recovered.

## Local validation data

`make validate-supported` mounts `results/local` on the host and writes
`results/local/supported_trial.csv` and `supported_summary.json` from the
actual physics run. These files are ignored by Git until reviewed.
The acceptance criteria are a finite MuJoCo state, motor tracking RMS
below 0.2 rad and measured right foot lift above 8 mm in the supported
rig. These are **trial criteria, not preclaimed achievements**.

`make stage5` can optionally take `ARGS="log_path:=/tmp/humanoid_lab_log.csv"`.
Files written to `/tmp` inside a disposable container disappear when
`docker run --rm` exits; use bind mounts to preserve them (the validation
command does this automatically).

## GUI

RViz2 needs a local Linux X11 desktop. The headless defaults intentionally
suppress RViz and joint-state-publisher-GUI (which would interfere with the
real `/joint_states` topic). For RViz:

```bash
xhost +si:localuser:root
make stage3-gui
```

The RViz URDF still treats torso as a fixed root, so it does not show the
free-base pose/fall. Use MuJoCo position logs and body pose telemetry to
assess physics, not RViz alone.

## Failures to investigate

- `docker: command not found` -> install Docker Engine.
- `permission denied /var/run/docker.sock` -> run with `sudo` or
  configure the Docker group after understanding root-equivalent access.
- apt/rosdep failure -> check connectivity and ROS apt availability.
- `colcon build` failure -> capture the actual ROS dependency and traceback.
- `make validate-supported` fails -> inspect generated metrics rather than
  changing thresholds to claim success; verify model/contact control.
- `No protocol specified` with GUI -> use headless command or fix X11.

To open a shell:

```bash
make shell
```
