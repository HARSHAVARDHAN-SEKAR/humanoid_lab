"""Loads config/robot_params.yaml -- the single source of truth for the
simplified biped's physical dimensions. Other packages (kinematics tests,
the sim bridge, the balance controller) should read from here rather than
hardcoding the numbers a second time.
"""
from dataclasses import dataclass
from pathlib import Path
import yaml

_DEFAULT_PATH = Path(__file__).resolve().parents[1] / "config" / "robot_params.yaml"


@dataclass
class RobotParams:
    thigh_length_m: float
    shank_length_m: float
    hip_width_m: float
    ankle_to_sole_offset_m: float
    standing_foot_x_m: float
    standing_foot_z_m: float


def load_robot_params(path: Path = None) -> RobotParams:
    path = path or _DEFAULT_PATH
    with open(path, "r") as f:
        raw = yaml.safe_load(f)
    return RobotParams(
        thigh_length_m=raw["leg"]["thigh_length_m"],
        shank_length_m=raw["leg"]["shank_length_m"],
        hip_width_m=raw["hip_width_m"],
        ankle_to_sole_offset_m=raw["ankle_to_sole_offset_m"],
        standing_foot_x_m=raw["standing_pose"]["foot_x_m"],
        standing_foot_z_m=raw["standing_pose"]["foot_z_m"],
    )
