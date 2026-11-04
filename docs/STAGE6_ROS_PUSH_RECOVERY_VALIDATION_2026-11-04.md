# Stage 6 — ROS 2 free-base push-recovery experiment (2026-11-04)

This is **simulation-only** evidence on the simplified free-floating MuJoCo
12-DOF biped. ROS 2 runs a CoM estimator that uses simulator-provided
base pose plus joint states, an ankle CoM PD controller and safety gating.
This is **not a real-hardware or dynamic walking validation**.

## Confirmed host results

- `make test` reported 91 passing tests after Stage 6L changes; the
  publishing helper reruns tests and stores its own pytest result separately.
- Stage 6J +20 N: peak CoM x shift **8.95 mm**, maximum torso tilt **1.603°**,
  final CoM x deviation **0.06 mm**, zero post-push samples with both foot
  loads under 20 N in the corrected force telemetry.
- Stage 6K +40 N: peak shift **18.12 mm**, maximum tilt **3.242°**,
  final shift **0.12 mm**; posture/control diagnostic passed.
- Stage 6K -40 N: peak shift **35.08 mm**, maximum tilt **4.232°**,
  final shift **0.16 mm**; posture/control diagnostic passed.
- Stage 6L +40 N: **325/325** post-push samples had valid contact polygons,
  with no negative projected CoM support margin. Strict polygon diagnostic PASS.
- Stage 6L -40 N: **286/325** post-push samples had valid polygons, **39**
  invalid/degenerate samples, no negative margin *among valid samples*.
  Strict polygon diagnostic **FAIL**. Stage 6M found one approximately
  **0.78-second** invalid sampled interval, from 2.14 to 2.90 s.
- Stage 6N separately recorded ROS-controlled debug-state replay videos:
  [forward](media/stage6n_ros_40N_forward.mp4) and
  [backward](media/stage6n_ros_40N_backward.mp4).
  The Stage 6N backward recording observed 92 invalid sampled polygons in
  its own run, sampled differently from Stage 6L. These must not be
  conflated as identical experiments.

Foot normal forces do not independently prove a nondegenerate 2D support
polygon; Stage 6L distinguishes these. A valid static CoM projection is not
proof of dynamic push stability, and simulator ground truth is not a real
state-estimator validation. Preserve the negative backward-push result.

Reproduce and inspect evidence using:

```bash
make test
bash scripts/stage6n_make_ros_sim_video.sh 40
bash scripts/stage6n_make_ros_sim_video.sh -40
```

Raw CSV measurements (where available) and the Stage 6M JSON are in
`results/validation/`; ROS and MuJoCo video demos are in `docs/media/`.
