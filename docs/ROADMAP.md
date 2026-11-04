# Remaining milestones and experiment plan

The repo is publishable as an **in-progress engineering prototype**, not
as a completed humanoid-walking robot. The following require actual
measured tests before they can be marked done.

1. **Validate the Docker image on Ubuntu.** `make build`, `make test`, `make smoke`,
   `make validate-supported`; fix CI/ROS build issues, record versions.
2. **Measure the supported stepping trial.** Compare commanded and actual
   knee/hip/ankle trajectories, confirm foot lift and safe fallbacks.
3. **Verify ROS graph and fault transport.** Use `make stage5` plus a
   second shell and inject delayed IMU, joint dropout, floor friction and
   torque reduction. Confirm measured state transitions and an enforced
   actuator command hold.
4. **Implement true floating-base body estimation.** Fuse body pose, CoM,
   contact wrenches, joint positions/velocities. Remove artificial fixed
   pelvis assumption in foot IK.
5. **Achieve unsupported standing** for 30 measured seconds in MuJoCo,
   from at least five independent randomized perturbation trials. Evaluate
   torso height, pitch/roll, CoM and support polygon.
6. **Design double-support CoM shift and 3D leg IK** or a whole-body QP
   controller; prove a single physically supported-to-unsupported step.
7. **Dynamic gait.** Validate >=1 m walking distance without falling,
   contact constraints, repeated-trial distributions, and torque bounds.
8. **Real push recovery.** Apply measured impulse profiles, decide ankle vs
   hip vs stepping strategy, record success/false positives and limits.
9. **Optional Unitree G1 and ros2_control port.** Requires validating the
   model license, real joint interfaces and controller manager integration.

**Evidence rule:** Record CSV/JSON/rosbag plus representative video, cite
measured results and commit only real generated evidence.
