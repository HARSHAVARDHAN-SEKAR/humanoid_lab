# HUMANOID-LAB


## November 2026 experimental update

- [Stage 4 supported-stepping demonstration](docs/STAGE4_SUPPORTED_STEPPING_VIDEO.md): four commanded steps with a world-fixed torso (not unsupported walking).
- [Stage 6 ROS free-base push-recovery experiments](docs/STAGE6_ROS_PUSH_RECOVERY_VALIDATION_2026-11-04.md): simulated CoM ankle-PD recovery, 91 passing host tests and explicit contact-polygon diagnostics.
- [Stage 6 forward 40 N video](docs/media/stage6n_ros_40N_forward.mp4) and [backward 40 N video](docs/media/stage6n_ros_40N_backward.mp4). The backward 40 N *continuous two-dimensional support* diagnostic failed (39/325 invalid polygon samples in Stage 6L).

**This is an experimental ROS 2/MuJoCo research prototype; dynamic free-base walking is not yet validated.** Historical verification notes below describe earlier revisions; consult the linked Stage 6 report for the latest measurements.

**ROS 2 + MuJoCo simplified humanoid locomotion, state estimation and fault-aware control.**

[![Python and MuJoCo regression tests](https://github.com/HARSHAVARDHAN-SEKAR/humanoid_lab/actions/workflows/tests.yml/badge.svg)](https://github.com/HARSHAVARDHAN-SEKAR/humanoid_lab/actions/workflows/tests.yml)

> **Status: Engineering research prototype; NOT a validated free-standing or dynamically walking humanoid.**
> The default physics model is a **fixed-torso support rig** so actuator control,
> foot IK and supported stepping can be investigated independently of free-base
> balance. The separate free-floating model remains experimental. A simulator
> launch, a published swing trajectory or a green unit test is **not evidence
> that unsupported humanoid walking works**. See [stage status](docs/STAGE_STATUS.md).

## What it demonstrates

- Twelve-DOF simplified biped modeled in MuJoCo and URDF/xacro.
- ROS 2 MuJoCo bridge publishing IMU, joint states, torso pose, height, and simulated foot forces.
- Analytic two-link sagittal-plane leg FK/IK and quintic swing trajectories.
- **One authoritative joint-command node** that combines leg IK with ankle feedback and limits. Rejects out-of-reach requests; holds measured posture on stale feedback or safety faults.
- Complementary IMU estimator and force-based contact detection.
- Mission FSM, gait sequencer and integrated safety supervisor (fault/hold/reset).
- Fault injection (stale IMU, joint dropout, floor friction, actuator torque limits).
- Tested offline math and a separate reproducible **supported-base** MuJoCo motor/foot-swing trial; CSV/JSON logging.
- ROS 2 Humble Docker image, optional RViz on Linux/X11, and GitHub Actions.

The default control backend is a custom ROS 2 to MuJoCo bridge, **not**
`mujoco_ros2_control`. The configuration in `humanoid_hardware/` documents
an alternate future integration and is not an implemented controller-manager
plugin. The kinematics are 2D sagittal IK, not full-body inverse dynamics.
This is a model of a simplified biped, **not** a Unitree G1.

## Start on Ubuntu using Docker

Prerequisites: `docker`, Docker daemon running, `make`, internet access for
the first `docker build` to fetch the ROS image, apt and Python packages.
You do **not** need ROS 2 installed on your Ubuntu host.

```bash
cd humanoid_lab_ws
make build                # ROS 2 Humble + MuJoCo + colcon build
make test                 # run all Python tests, including MuJoCo-dependent tests in Docker
make smoke                # exercise actual ROS 2 nodes/topics inside Docker
make validate-supported   # physical simulation of fixed-torso stepping; writes CSV+JSON to results/local/
make stage2               # ROS 2 headless fixed-torso standing/reference loop
make stage3               # ROS 2 headless fixed-torso stepping pipeline (autostarts)
```

All stage targets are headless by default. `make stage4` exercises mission
sequencing and supported stepping; it does **not** establish successful
free-base walking. Stage 5 brings up the supervisor and logger:

```bash
make stage4
make stage5
```

To run the experimental **free-floating model** (expected to have unsolved
balance problems), use:

```bash
make stage2 ARGS="model_file:=humanoid_biped.xml"
```

To use RViz on an Ubuntu desktop with X11:

```bash
make stage3-gui
```

See [docs/DOCKER.md](docs/DOCKER.md) for Docker permissions, custom commands,
CLI fault injection, logs and debugging.

## Tests without ROS 2

With Python 3.10+:

```bash
python3 -m pip install -r requirements.txt
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q -rs
```

If `mujoco` is absent, physics-dependent tests are skipped; **do not report
those as passing**. Independently in this sandbox, 70 pure-Python/XML/source
contract tests passed and two physics-test modules were skipped due to no
MuJoCo installation. A separate Ubuntu 22.04 host **successfully built** the Docker image and ran
`make validate-supported` (see [host validation log](docs/HOST_VALIDATION_2026-11-04.md)).
The initial host `make test`/`make smoke` attempts failed because of a pytest
plugin conflict and Bash `nounset`, respectively. The fixes in this revision
are not yet rerun on that host. The GitHub Actions job is designed to rerun the pure-Python/MuJoCo checks
and save trial artifacts. Full ROS smoke verification still needs rerunning on the host.

## Validation before you claim a demonstration

| Stage | Minimum evidence before marking it finished |
|---|---|
| 1. Model + joint control | Container builds; actuator/joint feedback responds to commands in MuJoCo |
| 2. Standing + balance | Supported rig benchmark; **separately** show >=30 s unsupported physically upright |
| 3. Stepping | Measured swing-foot lift and tracking in supported rig; unsupported step independently validated |
| 4. Continuous walking | Free-base walking distance measured with video and CSV; repeat trials |
| 5. Push recovery | Repeat controlled pushes, log success rates and limits; verify safety fault containment |

**Currently none of the full unsupported Stage 2–5 criteria are claimed as
achieved.** The included offline trial checks the narrower supported-rig
scope; its pass/fail result must be generated on your own machine, not taken
from this README.

## Package organization

```
src/
  humanoid_description/         URDF/xacro, robot dimensions, RViz
  humanoid_sim/                 supported + floating MJCF, MuJoCo ROS bridge
  humanoid_hardware/            ros2_control config only (not active)
  humanoid_kinematics/          analytic 2D leg FK/IK
  humanoid_state_estimator/     IMU filter and foot contacts
  humanoid_balance_controller/ ankle strategy and capture point
  humanoid_gait_planner/        alternating footstep + swing trajectory
  humanoid_motion_controller/  IK -> 12 actuator targets; safety gating
  humanoid_safety/             watchdog/fault FSM, injectable faults
  humanoid_mission_manager/    high-level mission FSM
  humanoid_bringup/            stages 1-5 launches and parameters
  humanoid_benchmarks/         logging, measured metrics, supported physics trial
```

Read the [architecture](docs/ARCHITECTURE.md), [limitations/status](docs/STAGE_STATUS.md)
and [remaining research milestones](docs/ROADMAP.md) before citing this project
in a CV or interview.

## Publish

Do this **after running `make test` and `make validate-supported` on your own computer**.
The existing public repository is `HARSHAVARDHAN-SEKAR/humanoid_lab` (currently empty). Run:

```bash
cd humanoid_lab_ws
git init
git add -A
git commit -m "HUMANOID-LAB: supported biped control and safety research prototype"
git branch -M main
git remote add origin https://github.com/HARSHAVARDHAN-SEKAR/humanoid_lab.git
git push -u origin main
```

Alternatively, run `bash scripts/publish_github.sh` from the workspace root.
The script targets the *existing* `humanoid_lab` repository and never force-pushes.

Use `gh auth login` or GitHub authentication when Git asks; HTTPS pushes
require a token/credential manager, not your GitHub account password.
GitHub description suggestion: **"ROS 2/MuJoCo 12-DOF humanoid control
research: supported gait IK, balance, fault-aware supervision, reproducible
physics tests; unsupported walking in development."**

## License

MIT. See [LICENSE](LICENSE).
