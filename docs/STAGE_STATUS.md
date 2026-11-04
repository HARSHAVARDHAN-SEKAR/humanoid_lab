# Implementation and verification status — 2026-11-04

## Scope and verification boundary

HUMANOID-LAB has two **different** MuJoCo physics models:

- **`humanoid_biped_supported.xml` (default):** Fixed torso at `(0,0,0.58)`.
  Gravity, rigid-body leg dynamics and ground contacts remain simulated,
  but the upper body cannot fall. This is a *support rig* for testing IK,
  joint controllers, swing clearance, ROS topics, safety and fault wiring.
  Fixed-base stepping cannot validate unsupported balance or walking.
- **`humanoid_biped.xml`:** Free-floating torso, 12 active joints.
  The original gain sweep reported collapse within roughly 0.3–0.5 seconds.
  Those historical findings were not reproduced as part of this revision.
  A controller that holds an IMU pitch to zero is not sufficient to prove
  center-of-mass regulation, dynamic gait or push recovery.

**Independent checks for this revision (in the editing environment):**
`pytest -q -rs`: **70 passed; two modules skipped** because MuJoCo and ROS 2
are not installed. These modules contain 8 physics tests. Python syntax,
XML and configuration parses were checked. A subsequent user-provided Ubuntu host transcript confirms **Docker image build**
and a **passed supported-body MuJoCo trial** (3.5 s, RMS tracking error
0.0740 rad, right foot peak lift 0.0559 m). See
[host validation](HOST_VALIDATION_2026-11-04.md). The initial Docker pytest
and ROS smoke commands failed due to tooling/shell configuration; compatibility
fixes are included but must be rerun on the host. Full unsupported walking
remains unverified. No raw experiment CSV was invented or committed.

## Five stages

| Stage | Implemented | Demonstrated on free-floating humanoid? | Next acceptance gate |
|---|---|---|---|
| 1. Model and joint control | MJCF and URDF, bridge, motor topic, initialization, static tests | No ROS/Docker runtime verified here | Successful container build and physics motor test |
| 2. Standing and balance | Ankle PD, estimator, contact detection, command composition | **No** | Stable unsupported 30 s with measured torso height and pitch |
| 3. Stepping | 2D IK, swing sequencing, command publication; supported-rig physics trial provided | **No** | Repeatable measured unsupported contact change + step |
| 4. Walking | Mission FSM wired to gait completion, small-stride supported demo launch | **No** | Measured unsupported walk in simulation, repeated trials |
| 5. Disturbance recovery | Safety/FSM, fault injector, actuator hold, reset service, CSV logger | **No** | Measured pushes and safe intervention / successful recovery metrics |

## What was repaired from the previous ZIP

- Added `humanoid_motion_controller`: sends the actual 12-joint command
  to MuJoCo from IK and feedback and holds on unsafe/stale inputs.
- Corrected the balance/estimator attitude topic; force contacts no longer
  return true just because a hard-coded foot-height of zero was passed.
- Added measured torso pose/height and requested joint positions to the
  logger for meaningful RMS and upright-time calculations.
- Fixed foot-swing starts to use each foot's own previous landing.
- Replaced mission timer-based synthetic step countdown with gait feedback.
- Wired safety to mission request, measured tracking error and joint age;
  added a guarded reset service.
- Corrected fault injector topics and sent commands after discovery delay.
- Default launch is headless, suppressing the GUI joint-state publisher
  that would race with MuJoCo's `/joint_states`.
- Added fixed-torso simulation specifically for staged joint and step tests,
  plus an offline physics experiment that records actual data and checks
  tracking error and foot clearance without claiming human-like walking.

## Known limitations and open work

- Docker image build and one supported-rig MuJoCo trial were reported
  successful on an Ubuntu/Docker host; full ROS integration smoke and Docker
  pytest after the current fixes still require verification. The editing
  sandbox did not provide a Docker engine.
- Fixed-base support makes balance artificially trivial; the active ankle
  controller is only one piece of complete free-base balance control.
- Pure sagittal IK with fixed pelvis cannot handle lateral CoM shift,
  pelvis translation, arbitrary foot yaw or nonlinear whole-body contact.
- `humanoid_hardware` is a configuration example, *not* a connected
  `mujoco_ros2_control` plugin/`controller_manager` runtime.
- SAFE_HOLD sends a last-measured joint target and stops further gait
  requests; it does not physically guarantee a falling humanoid can stop.
- The contact thresholds and safety tracking limits are illustrative,
  require tuning against measured traces, and have no hardware safety claim.
- No push recovery algorithm has been validated against disturbances.
- The logged `safety_state=STAND` is **not** proof of standing; use
  measured torso height and pitch for physical upright duration.
- The supported-base model is not a Unitree G1.

## Evidence policy

Use `make validate-supported` to write local `results/local/supported_trial.csv`
and `supported_summary.json` generated from MuJoCo. If these pass on your
machine, share the actual logs and commit **reviewed** evidence under
`results/validation/`. The code intentionally does not ship invented results.
A successful supported trial says nothing about free-standing push recovery.
