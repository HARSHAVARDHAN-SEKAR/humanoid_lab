# Ubuntu host validation — 2026-11-04

Source: user-provided terminal transcript from an Ubuntu 22.04.5 LTS machine
running Docker Engine 29.9.3, 16 CPUs, and 30.65 GiB RAM. This document
transcribes selected **actual reported observations**; it is not a re-run by
the maintainer and it does not invent missing experiment CSV files.

## Verified from terminal transcript

- `make build`: **passed**. The `humanoid-lab:humble` image built successfully,
  including the ROS 2 Humble `colcon build` Docker layer.
- `make validate-supported`: **passed**, per JSON emitted by MuJoCo trial:

  | Metric | Reported value |
  |---|---:|
  | Scope | simplified 12-DOF fixed-torso support |
  | Simulated duration | 3.5 s |
  | Commanded stride | 0.04 m |
  | Joint tracking RMS | 0.07400221305827977 rad |
  | Measured right foot peak lift | 0.05590292985292312 m |
  | Samples | 350 |
  | Finite physics / tracking / foot lift criteria | all `true` |

  This result is **supported joint tracking and foot swing**, not free-base
  balance, unsupported stepping, independent walking or recovery.

## Initial failures in this transcript

- `make test`: `PluginValidationError: Plugin 'launch_testing' for hook
  'pytest_pycollect_makemodule'` caused by third-party pytest plugin
  compatibility. This revision calls pytest with
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` and pins `pytest<8`.
- `make smoke`: `/opt/ros/humble/setup.bash: line 8:
  AMENT_TRACE_SETUP_FILES: unbound variable`; setup scripts were sourced under
  Bash `set -u`. This revision sources the setup scripts before enabling `set -u`.

**The fixes have not yet been re-tested inside the user's Docker host**.
Do not mark these two checks green until they pass there.

## Re-run on Ubuntu

```bash
cd ~/Downloads/humanoid_lab_ws
make build
make test
make smoke
make validate-supported
```

`make validate-supported` writes `results/local/supported_trial.csv` and
`results/local/supported_summary.json`. These local artifacts are ignored by
Git, intentionally. Review and copy relevant raw traces into
`results/validation/` if you want to publish reproducible experiment evidence.
