"""Testable command pipeline for a *simplified* 12-DOF biped.

The 2-link IK is sagittal-plane only. There is no floating-base QP/WBC:
this controller holds the nominal pelvis transform constant; it is intended
for an externally supported simulation rig or experimental free-base use.
This implementation cannot guarantee stable dynamic walking.
"""
from dataclasses import dataclass, field
import math

from humanoid_kinematics.planar_leg import PlanarLegKinematics, LegKinematicsError

JOINT_NAMES = tuple(
    f"{side}_{joint}_joint"
    for side in ("right", "left")
    for joint in (
        "hip_yaw", "hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll"
    )
)

# Matches the physical MJCF joint range; leave a small safety margin for IK.
JOINT_LIMITS = {
    "hip_yaw": (-0.6, 0.6),
    "hip_roll": (-0.5, 0.5),
    "hip_pitch": (-1.8, 1.2),
    "knee_pitch": (0.0, 2.5),
    "ankle_pitch": (-1.0, 1.0),
    "ankle_roll": (-0.5, 0.5),
}


@dataclass(frozen=True)
class FootTarget:
    x: float
    y: float
    z: float = 0.0


@dataclass(frozen=True)
class CommandResult:
    positions: dict
    mode: str
    max_tracking_error_rad: float = 0.0


@dataclass
class CommandComposer:
    """Compose IK + ankle feedback + safety hold into a single command.

    Coordinates: target foot sole is in a fixed world frame; base hip is
    (base_x, 0, base_height); ankle lies 0.03 m above sole contact surface.
    This intentionally does not compensate for physical floating-base motion.
    """
    base_height_m: float = 0.58
    base_x_m: float = 0.0
    sole_offset_m: float = 0.03
    max_balance_correction_rad: float = 0.12
    kinematics: PlanarLegKinematics = field(
        default_factory=lambda: PlanarLegKinematics(0.30, 0.30)
    )
    feet: dict = field(default_factory=lambda: {
        "right": FootTarget(0.0, -0.10),
        "left": FootTarget(0.0, 0.10),
    })
    latest_feedback: dict = field(default_factory=dict)
    last_command: dict = field(default_factory=dict)
    safety_state: str = "STAND"
    last_fault: str = ""

    def __post_init__(self):
        self.feet = dict(self.feet)
        self.last_command = self.standing_command()

    def update_feedback(self, measured: dict) -> None:
        for name, position in measured.items():
            if name in JOINT_NAMES and math.isfinite(position):
                self.latest_feedback[name] = float(position)

    def set_foot(self, side: str, target: FootTarget) -> None:
        if side not in ("left", "right"):
            raise ValueError(f"unsupported foot: {side}")
        if not all(math.isfinite(v) for v in (target.x, target.y, target.z)):
            raise ValueError("nonfinite foot target")
        # Lateral 3D IK is not implemented; accept only a small delta from hip y.
        expected_y = 0.1 if side == "left" else -0.1
        if abs(target.y - expected_y) > 0.025:
            raise ValueError("lateral foot placement requires 3D IK")
        self.feet[side] = target

    def standing_command(self):
        return self._solve_ik({
            "right": FootTarget(0.0, -0.10),
            "left": FootTarget(0.0, 0.10),
        }, 0.0, 0.0)

    def _solve_ik(self, feet: dict, pitch_corr: float, roll_corr: float):
        if not all(math.isfinite(v) for v in (pitch_corr, roll_corr)):
            raise ValueError("nonfinite balance correction")
        pitch_corr = max(-self.max_balance_correction_rad,
                         min(self.max_balance_correction_rad, pitch_corr))
        roll_corr = max(-self.max_balance_correction_rad,
                        min(self.max_balance_correction_rad, roll_corr))
        positions = {}
        for side in ("right", "left"):
            foot = feet[side]
            pose = self.kinematics.inverse(
                foot.x - self.base_x_m,
                foot.z + self.sole_offset_m - self.base_height_m,
            )
            vals = (0.0, 0.0, pose.hip_pitch, pose.knee_pitch,
                    pose.ankle_pitch + pitch_corr, roll_corr if side == "right" else -roll_corr)
            for joint, value in zip(("hip_yaw", "hip_roll", "hip_pitch", "knee_pitch",
                                     "ankle_pitch", "ankle_roll"), vals):
                lo, hi = JOINT_LIMITS[joint]
                if not math.isfinite(value) or not lo - 1e-8 <= value <= hi + 1e-8:
                    raise LegKinematicsError(f"{side}_{joint} target {value:.4f} outside limits [{lo}, {hi}]")
                positions[f"{side}_{joint}_joint"] = float(value)
        return positions

    def update(self, *, safety: str, feedback_fresh: bool, pitch_corr: float = 0.,
               roll_corr: float = 0., allow_motion: bool = True) -> CommandResult:
        """Return one authoritative 12-joint command, never an unbounded target.

        FAULT/SAFE_HOLD/RECOVERY => target measured posture (no new gait).
        This *does not* disable gravity or guarantee physical balance.
        When feedback is stale, command the last known measured posture.
        """
        self.safety_state = safety
        inhibited = safety not in ("STAND", "WALK") or not feedback_fresh
        if inhibited:
            self.last_fault = f"safety={safety}, feedback_fresh={feedback_fresh}"
            positions = dict(self.latest_feedback or self.last_command)
            mode = "SAFE_HOLD"
        else:
            try:
                foot_refs = self.feet if allow_motion else {
                    "right": FootTarget(0.0, -0.10), "left": FootTarget(0.0, 0.10)
                }
                positions = self._solve_ik(foot_refs, pitch_corr, roll_corr)
                mode = "TRACKING" if allow_motion else "STAND"
            except (LegKinematicsError, ValueError) as exc:
                self.last_fault = str(exc)
                positions = dict(self.latest_feedback or self.last_command)
                mode = "IK_FAULT_HOLD"
        self.last_command = dict(positions)
        if self.latest_feedback:
            err = max(abs(positions[j] - self.latest_feedback[j])
                      for j in positions if j in self.latest_feedback)
        else:
            err = 0.0
        return CommandResult(positions=positions, mode=mode, max_tracking_error_rad=float(err))
