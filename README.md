# HUMANOID-LAB

**ROS 2 Humble · MuJoCo · 12-DOF simplified biped · Gait planning · CoM balance · Safety supervision**

[![Regression tests](https://github.com/HARSHAVARDHAN-SEKAR/humanoid_lab/actions/workflows/tests.yml/badge.svg)](https://github.com/HARSHAVARDHAN-SEKAR/humanoid_lab/actions/workflows/tests.yml)

> **Research prototype — simulation only.** The repository has two distinct demonstrations: **supported stepping with a world-fixed torso** and **free-floating standing / push recovery using experimental CoM-based ankle feedback**. It does **not** demonstrate independent, dynamically balanced walking or real-robot safety.

## Watch the demos

| Experiment | Video | What it demonstrates |
| --- | --- | --- |
| **Stage 4: Supported stepping** | [Four-step MuJoCo screen recording](docs/media/stage4_supported_stepping.mp4) | ROS mission manager, gait targets, leg IK and observed joint motion. World-fixed torso. |
| **Stage 6N: +40 N forward push** | [Free-base ROS simulation replay](docs/media/stage6n_ros_40N_forward.mp4) | Recorded simulator poses under ROS 2 CoM ankle-PD feedback. |
| **Stage 6N: −40 N backward push** | [Free-base ROS simulation replay](docs/media/stage6n_ros_40N_backward.mp4) | Posture recovery, with an identified limitation in contact-polygon continuity. |

Stage 6N MP4s are offline visual replays of **recorded ROS-controlled MuJoCo states**, not independent physics simulations or hardware footage.

## Verified status — 4 November 2026

The Ubuntu Docker run built the ROS 2 Humble image and all 12 colcon packages. **91/91 Python/MuJoCo regression tests passed**; see [captured test output](results/validation/stage6_latest_docker_pytest.txt).

**Stage 4:** the ROS gait planner logged four commanded swing steps, **right → left → right → left**, followed by `Commanded swing sequence complete`. This verifies command-sequence completion, **not** measured touchdown accuracy or unsupported translation.

**Stage 6L:** separate ROS free-base disturbance tests yielded:

| Metric | +40 N forward | −40 N backward |
| --- | ---: | ---: |
| Peak CoM X displacement | 18.37 mm | 35.74 mm |
| Maximum torso tilt | 3.202° | 4.212° |
| Final CoM X displacement | +0.11 mm | −0.16 mm |
| Valid post-push 2D support polygons | **325/325** | **286/325** |
| Invalid/degenerate polygons | **0** | **39** |
| Minimum margin among *valid* polygons | 41.98 mm | 33.43 mm |
| Posture/recovery diagnostic | PASS | PASS |
| Strict continuous-support diagnostic | **PASS** | **FAIL** |

Stage 6M grouped the 39 backward invalid samples into **one approximately 0.78-second interval** (about 2.14–2.90 seconds in that trial). Both feet continued carrying measured load, but loaded contacts do not necessarily form a usable two-dimensional polygon. The backward case must remain **FAIL** under the strict criterion.

Earlier corrected **Stage 6J +20 N** ROS telemetry: 8.95 mm peak CoM X displacement, 1.603° maximum torso tilt, and 0.06 mm final displacement. The Stage 6N video runs are separate trials with their own sampling; their outcomes must not be conflated with the Stage 6L measurements.

[Read the complete ROS push-recovery report](docs/STAGE6_ROS_PUSH_RECOVERY_VALIDATION_2026-11-04.md) and [Stage 6M contact-window analysis](results/validation/stage6m_contact_window_summary.json). Raw measured traces are in [`results/validation/`](results/validation/).

## Architecture

```text
                    ROS 2 mission / safety supervision
                                 |
           +---------------------+--------------------+
           |                                          |
 Stage 4: supported stepping               Stage 6: free-base standing
           |                                          |
    Footstep + swing planner                MuJoCo torso pose + joint states
           |                                          |
    Sagittal leg inverse IK                 Experimental CoM estimator
           |                                          |
    Motion joint controller                 CoM X/velocity ankle-PD feedback
           |                                          |
           +---------------------+--------------------+
                                 |
                   /mujoco_bridge/joint_commands
                                 |
                     12-actuator MuJoCo biped
                                 |
                     Joint/contact/pose feedback
```

These are **different launch/controller configurations**. Do not run both command publishers against the same bridge simultaneously.

Implemented modules include the supported and floating-base MuJoCo models, URDF/xacro description, 2D sagittal FK/IK, footstep and swing planning, command arbitration within each pipeline, ROS 2 CoM feedback, safety and stale-data gating, opt-in bounded push injection, solver-derived foot-force telemetry, support-polygon diagnostics, and reproducible video capture.

**Estimator boundary:** Stage 6 uses simulator-provided base pose plus joint feedback. It is not an independently validated sensor-only localization or state-estimation solution. `humanoid_hardware/` contains example configuration, not an active physical robot interface. This project is **not** a Unitree G1 implementation or an active `mujoco_ros2_control` plugin.

## Quick start (Ubuntu + Docker)

Requirements: Docker with user access to its daemon, GNU Make, and network access for the first image build. ROS 2 Humble runs inside Docker; it is not required on the Ubuntu host.

```bash
git clone --branch experiment/stage6-freebase-balance \
  https://github.com/HARSHAVARDHAN-SEKAR/humanoid_lab.git
cd humanoid_lab

make test                  # build Docker image; run regression tests
make validate-supported    # fixed-torso physics benchmark
make stage4                # headless four-step supported gait sequence
```

`make stage4` runs the **world-fixed-torso** mission-controlled stepping demonstration. It does **not** produce an unsupported walk. See [Stage 4 demonstration notes](docs/STAGE4_SUPPORTED_STEPPING_VIDEO.md).

To reproduce the **Stage 6N** videos from the repository root on Ubuntu:

```bash
bash scripts/stage6n_make_ros_sim_video.sh both
```

This runs separate forward/backward 40 N ROS experiments, captures the simulated base/joint poses and renders MP4 files under `results/local/` (Git-ignored). The published videos are in [`docs/media/`](docs/media/). Small numeric differences between separate trials are expected.

**Reproducibility caveat:** the committed Stage 6J–L CSVs are real measured evidence, but the original local probe-helper scripts used for some of those runs were not all committed. Do not assume a single command reproduces the exact published Stage 6L CSVs. The Stage 6N capture script is committed.

## Scope and limitations

| Area | Demonstrated | Still required |
| --- | --- | --- |
| Supported locomotion | Four commanded alternating swing steps; joint motion | Free-base walking, measured pelvis travel and landing accuracy |
| Free-base standing | Bounded sagittal pushes with ROS CoM ankle-PD recovery | Continuous valid support under backward 40 N; robust multi-direction balance |
| Contact monitoring | Measured normal loads, contact geometry and projected CoM margin | Reliable edge-contact recovery and dynamic contact stability |
| Estimation | CoM reconstructed from simulator base pose and joint state | Sensor-only robust base and CoM state estimation |
| Safety | Simulation supervision and fault handling | Hardware-relevant validation and torque/impact safety |

**Next technical milestones:** investigate backward foot-edge rocking; run matched feedback-on/off controls; design contact-aware ankle/hip correction, lateral weight shift and footstep recovery; then measure pelvis translation and touchdown before claiming independent dynamic walking.

## Repository guide

- [`src/humanoid_sim/`](src/humanoid_sim/) — MuJoCo models and ROS 2 bridge
- [`src/humanoid_kinematics/`](src/humanoid_kinematics/) — leg kinematics
- [`src/humanoid_gait_planner/`](src/humanoid_gait_planner/) — footstep and swing planning
- [`src/humanoid_motion_controller/`](src/humanoid_motion_controller/) — supported-gait joint references
- [`src/humanoid_state_estimator/`](src/humanoid_state_estimator/) — state estimation
- [`src/humanoid_balance_controller/`](src/humanoid_balance_controller/) — ankle balance logic
- [`src/humanoid_mission_manager/`](src/humanoid_mission_manager/) — mission state machine
- [`src/humanoid_safety/`](src/humanoid_safety/) — safety supervision
- [`scripts/`](scripts/) — experiment, diagnostic and video workflows
- [`tests/`](tests/) — automated regression tests

Further reading: [architecture](docs/ARCHITECTURE.md), [Stage 4 supported video](docs/STAGE4_SUPPORTED_STEPPING_VIDEO.md), [Stage 6N replay method](docs/STAGE6N_ROS_VIDEO.md), [Stage 6 ROS validation](docs/STAGE6_ROS_PUSH_RECOVERY_VALIDATION_2026-11-04.md), [earlier offline balance experiments](docs/STAGE6_VALIDATION_2026-11-04.md), [roadmap](docs/ROADMAP.md).

Older host-validation and stage-status notes describe earlier milestones; use the dated latest ROS validation report for current measurements.

## License

MIT — see [LICENSE](LICENSE).
