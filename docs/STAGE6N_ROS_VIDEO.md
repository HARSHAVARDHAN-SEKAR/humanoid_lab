# Stage 6N — Actual ROS 2 MuJoCo simulation replay

The free-floating 12-DOF biped is run using ROS 2 Stage 6I's CoM feedback
controller. A separate, **simulation-only** debug topic publishes the full
MuJoCo floating-base configuration (time + 19 `qpos` values). A ROS recorder
stores the actual simulated joint/base trajectory during a bounded push.

An offline MuJoCo renderer replays those recorded poses to produce MP4 video.
This avoids slowing the ROS balance loop with live video rendering. Replay
reproduces the visible poses; it is **not** a second physics simulation.

- Run: `bash scripts/stage6n_make_ros_sim_video.sh both` (host shell)
- Outputs: `results/local/stage6n_ros_40N_forward.mp4`,
  `results/local/stage6n_ros_40N_backward.mp4`
- Traces: similarly named `.npz` files containing actual simulator poses.
- Visual overlays: CoM shift, torso tilt, sampled support-polygon validity,
  controller mode, safety state and push-timing marker.

## Results / scope

- Stage 6L ROS feedback: +40 N forward passed 325/325 strict support-polygon
  samples, -40 N backward had 39/325 degenerate contact polygons.
- Stage 6M: the -40 N backward invalidity formed **one interval of about 0.78 s**.
- Both directions passed posture and final recovery criteria, but the 40 N
  backward case **failed strict continuous-2D-support diagnostic**.
- Dynamic stability, full humanoid locomotion and hardware use are not proven.
- `debug_freebase_qpos` is simulator ground truth for **visualization only**;
  it must not be substituted into the control state estimator.

No video or figure should be described as a real-robot demonstration.
