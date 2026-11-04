# Stage 4 — Supported stepping in MuJoCo

[Recorded Ubuntu MuJoCo GUI demonstration](media/stage4_supported_stepping.mp4)

The ROS 2 mission manager requested four supported swing-foot steps, in this
order: right, left, right, left. The local launch log contained `Step 1/4`
through `Step 4/4` and `Commanded swing sequence complete`.
The viewer displays actual `/joint_states` from the ROS-controlled simulator.

**Important scope:** `humanoid_biped_supported.xml` fixes the torso to the
world. The experiment demonstrates mission sequencing, planned swing targets,
leg IK and commanded/observed joint movement. It does **not** validate
unsupported or translating humanoid walking, touchdown accuracy or robust
physical footstep execution.

Run on Ubuntu with Docker:

```bash
bash scripts/stage4_supported_walk_gui.sh
# In another terminal after recording starts:
bash scripts/stage4_trigger_steps.sh
```

The raw screen recording has a terminal overlapping part of the model.
This is development evidence rather than a polished product demonstration.
