# HUMANOID-LAB — Stage 6 Free-Floating Balance Experiments

Date: 2026-11-04
Environment: MuJoCo, ROS 2 Humble Docker image
Model: 12-DOF simplified free-floating biped

## 6A: Unperturbed standing

30 simulated seconds completed.

- Minimum torso height: 0.5724 m
- Maximum torso tilt: 2.27 degrees
- Maximum joint tracking error: 0.05655 rad
- Maximum torso XY drift: 0.0192 m

Controller: constant standing joint-position commands.

## 6B: Support geometry

- Total robot mass: 24.9 kg
- Minimum CoM support margin: 0.04063 m
- Double-support sample fraction: 100%
- Non-foot ground contacts: 0

The baseline support polygon was computed from active
foot-floor contact points.

## 6C: Single forward push

- Force: 20 N
- Duration: 0.1 s
- Peak CoM deviation: 0.02058 m
- Final CoM deviation: 0.00055 m
- Maximum torso tilt: 2.29445 degrees

## 6D: Initial disturbance matrix

Constant joint-position targets: 6/8 checks passed.

The 40 N forward and backward pushes caused falls,
with torso rotations exceeding 100 degrees.

## 6E: Experimental CoM feedback

A sagittal CoM PD correction applied to ankle-pitch
position references was tested with both correction signs.

Parameters:
- Kp: 2.5
- Kd: 0.35
- Maximum ankle correction: 0.12 rad
- Maximum correction rate: 1.0 rad/s

Negative-sign feedback passed the 40 N forward and
backward tests in the initial comparison.

Positive-sign feedback failed both tests.

## 6F: Extended disturbance regression

Negative CoM feedback passed 9/12 strict diagnostic checks.

The remaining failures were 30 N, 40 N and 45 N
backward pushes due to periods with unavailable
nondegenerate support polygons.

## 6G: Contact diagnosis

During the invalid-polygon samples:
- Both feet continued carrying load
- Four loaded contact points were detected
- No samples had zero loaded contact points
- No samples had only one foot loaded

Invalid-polygon samples:
- 30 N backward: 9
- 40 N backward: 24
- 45 N backward: 40

This is consistent with degenerate contact geometry,
potentially involving foot-edge rocking.
Exact contact-point positions were not yet analyzed.

## Interpretation

The free-floating biped remained upright and returned
near the undisturbed state in the tested feedback scenarios.

The strict continuous-polygon result remains 9/12.
Do not relabel this as 12/12 fully stable scenarios.

## Limitations

These are simulation experiments using exact MuJoCo CoM
state, not sensor-derived state estimation.

The controller is not yet integrated into the ROS 2
balance-control pipeline.

The results do not establish reliable dynamic walking,
stepping recovery, or hardware safety.
