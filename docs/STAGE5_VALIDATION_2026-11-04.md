# HUMANOID-LAB — Stage 5 Validation

Date: 2026-11-04
Environment: Ubuntu, Docker, ROS 2 Humble, MuJoCo
Model: 12-DOF simplified fixed-torso supported biped

## Validated results

| Test | Result |
|---|---|
| Docker pytest | 79 passed |
| Physics-only standing | 150 simulated seconds, passed |
| ROS 2 standing | 150 seconds, passed |
| Maximum ROS standing joint error | 0.00733 rad |
| Joint-feedback dropout | Detected; SAFE_HOLD activated |
| Joint-dropout reset | Accepted |
| Joint-dropout post-reset observation | 45 seconds, passed |
| Maximum post-dropout error | 0.01134 rad |
| IMU delay injection | 0.60 seconds |
| IMU stale-data fault | Detected at 0.201 s |
| IMU post-reset observation | 45 seconds, passed |
| Maximum post-IMU-reset error | 0.00985 rad |
| Four-step mission safety check | Passed |
| Four-step recording | 2074 rows, 20.76 seconds |
| Maximum four-step joint error | 0.0169 rad |
| Final safety and mission states | STAND / STAND |
| Joint tracking safety threshold | 0.700 rad, unchanged |

## Major engineering correction

Initial tests revealed progressive hip-joint tracking error.

MuJoCo contact diagnostics identified severe unintended
self-collisions between torso, hip, thigh and other internal
geometries, including physically unrealistic contact forces.

The supported-rig collision model was corrected to allow
foot-floor collisions while disabling internal geometry contacts.

In the corrected physics-only standing test, the maximum
joint-tracking error was 0.0073 rad over 150 simulated seconds.

## Safety behavior

Both joint-feedback dropout and delayed IMU injection triggered
FAULT_DETECTED followed by latched SAFE_HOLD.

Following feedback restoration and an accepted reset,
the robot remained in STAND during the recorded 45-second
recovery observation periods.

## Four-step integration

The recorded safety sequence was STAND -> WALK -> STAND.
The mission returned to STAND with no recorded safety fault.

The CSV verifies mission transitions and joint tracking.
Individual foot placement and actual step displacement are
not established by this CSV alone.

## Limitations

This is a mechanically supported, fixed-torso model.
These experiments do not demonstrate unsupported balance,
dynamic walking, or physical push recovery.

The safety logic is research software, not certified hardware safety.
Continuous delayed-message timestamp validation and additional
fault cases remain future work.
