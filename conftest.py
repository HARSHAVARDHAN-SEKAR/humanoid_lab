"""Puts every package's python module directory on sys.path so tests can
`import humanoid_kinematics.planar_leg` etc. without a colcon build/install.
"""
import sys
import pathlib

ROOT = pathlib.Path(__file__).parent
SRC = ROOT / "src"
for pkg_dir in sorted(SRC.iterdir()):
    if pkg_dir.is_dir():
        sys.path.insert(0, str(pkg_dir))
