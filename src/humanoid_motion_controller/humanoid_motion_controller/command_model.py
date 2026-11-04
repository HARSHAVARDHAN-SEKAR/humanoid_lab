
"""Joint-command composition for the simplified supported biped.

Combines:
- Planar leg inverse kinematics
- Ankle balance corrections
- Fault-triggered latched position hold
- Rate-limited command recovery

This is a simulation control experiment, not a guarantee
of stable unsupported humanoid balance.
"""

from dataclasses import dataclass, field
import math

from humanoid_kinematics.planar_leg import (
    PlanarLegKinematics,
    LegKinematicsError,
)


JOINT_NAMES = tuple(
    f"{side}_{joint}_joint"
    for side in ("right", "left")
    for joint in (
        "hip_yaw",
        "hip_roll",
        "hip_pitch",
        "knee_pitch",
        "ankle_pitch",
        "ankle_roll",
    )
)


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

    base_height_m: float = 0.58
    base_x_m: float = 0.0
    sole_offset_m: float = 0.03
    max_balance_correction_rad: float = 0.12

    # Recovery commands change by at most 0.25 rad/s
    # at the nominal 100 Hz controller update rate.
    recovery_rate_rad_s: float = 0.25
    control_period_s: float = 0.01

    kinematics: PlanarLegKinematics = field(
        default_factory=lambda: PlanarLegKinematics(
            0.30, 0.30
        )
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

        self._hold_reference = None
        self._was_inhibited = False
        self._recovering = False

    @staticmethod
    def neutral_feet():

        return {
            "right": FootTarget(0.0, -0.10),
            "left": FootTarget(0.0, 0.10),
        }

    def update_feedback(self, measured: dict):

        for name, position in measured.items():

            if name in JOINT_NAMES and math.isfinite(position):
                self.latest_feedback[name] = float(position)

    def set_foot(self, side: str, target: FootTarget):

        if side not in ("left", "right"):
            raise ValueError(
                f"unsupported foot: {side}"
            )

        if not all(
            math.isfinite(value)
            for value in (target.x, target.y, target.z)
        ):
            raise ValueError("nonfinite foot target")

        expected_y = (
            0.10 if side == "left" else -0.10
        )

        if abs(target.y - expected_y) > 0.025:
            raise ValueError(
                "lateral foot placement requires 3D IK"
            )

        self.feet[side] = target

    def standing_command(self):

        return self._solve_ik(
            self.neutral_feet(),
            0.0,
            0.0,
        )

    def _solve_ik(
        self,
        feet: dict,
        pitch_corr: float,
        roll_corr: float,
    ):

        if not all(
            math.isfinite(v)
            for v in (pitch_corr, roll_corr)
        ):
            raise ValueError(
                "nonfinite balance correction"
            )

        pitch_corr = max(
            -self.max_balance_correction_rad,
            min(
                self.max_balance_correction_rad,
                pitch_corr,
            ),
        )

        roll_corr = max(
            -self.max_balance_correction_rad,
            min(
                self.max_balance_correction_rad,
                roll_corr,
            ),
        )

        positions = {}

        for side in ("right", "left"):

            foot = feet[side]

            pose = self.kinematics.inverse(
                foot.x - self.base_x_m,
                foot.z
                + self.sole_offset_m
                - self.base_height_m,
            )

            values = (
                0.0,
                0.0,
                pose.hip_pitch,
                pose.knee_pitch,
                pose.ankle_pitch + pitch_corr,
                roll_corr if side == "right" else -roll_corr,
            )

            joint_types = (
                "hip_yaw",
                "hip_roll",
                "hip_pitch",
                "knee_pitch",
                "ankle_pitch",
                "ankle_roll",
            )

            for joint, value in zip(
                joint_types, values
            ):

                low, high = JOINT_LIMITS[joint]

                if (
                    not math.isfinite(value)
                    or value < low - 1e-8
                    or value > high + 1e-8
                ):
                    raise LegKinematicsError(
                        f"{side}_{joint} target "
                        f"{value:.4f} outside limits "
                        f"[{low}, {high}]"
                    )

                positions[
                    f"{side}_{joint}_joint"
                ] = float(value)

        return positions

    def _enter_hold(self):

        # Capture the posture only ONCE.
        # Do not continuously chase joint feedback.

        self._hold_reference = {
            joint: self.latest_feedback.get(
                joint,
                self.last_command[joint],
            )
            for joint in JOINT_NAMES
        }

        self._was_inhibited = True
        self._recovering = False

        # Discard stale planned footsteps.
        # A reset must not automatically resume an
        # old swing-foot trajectory.

        self.feet = self.neutral_feet()

    def _rate_limit_recovery(self, desired: dict):

        previous = self.last_command

        max_step = (
            self.recovery_rate_rad_s
            * self.control_period_s
        )

        if max_step <= 0.0:
            raise ValueError(
                "Recovery command step must be positive"
            )

        positions = {}
        finished = True

        for joint in JOINT_NAMES:

            current = previous[joint]
            target = desired[joint]

            difference = target - current

            if abs(difference) > max_step:

                finished = False

                change = math.copysign(
                    max_step,
                    difference,
                )

                positions[joint] = (
                    current + change
                )

            else:
                positions[joint] = target

        return positions, finished

    def update(
        self,
        *,
        safety: str,
        feedback_fresh: bool,
        pitch_corr: float = 0.0,
        roll_corr: float = 0.0,
        allow_motion: bool = True,
    ) -> CommandResult:

        self.safety_state = safety

        inhibited = (
            safety not in ("STAND", "WALK")
            or not feedback_fresh
        )

        if inhibited:

            if not self._was_inhibited:
                self._enter_hold()

            self.last_fault = (
                f"safety={safety}, "
                f"feedback_fresh={feedback_fresh}"
            )

            positions = dict(
                self._hold_reference
            )

            mode = "SAFE_HOLD"

        else:

            try:

                foot_refs = (
                    self.feet
                    if allow_motion
                    else self.neutral_feet()
                )

                desired = self._solve_ik(
                    foot_refs,
                    pitch_corr,
                    roll_corr,
                )

                # First nominal cycle after a fault.
                # Begin from the latched hold command,
                # never jump directly to desired IK.

                if self._was_inhibited:

                    self._was_inhibited = False
                    self._recovering = True

                if self._recovering:

                    positions, finished = (
                        self._rate_limit_recovery(
                            desired
                        )
                    )

                    mode = "RECOVERY_TRACKING"

                    if finished:
                        self._recovering = False

                else:

                    positions = desired

                    mode = (
                        "TRACKING"
                        if allow_motion
                        else "STAND"
                    )

            except (LegKinematicsError, ValueError) as exc:

                self.last_fault = str(exc)

                if not self._was_inhibited:
                    self._enter_hold()

                positions = dict(
                    self._hold_reference
                )

                mode = "IK_FAULT_HOLD"

        self.last_command = dict(positions)

        errors = [
            abs(
                positions[joint]
                - self.latest_feedback[joint]
            )
            for joint in positions
            if joint in self.latest_feedback
        ]

        maximum_error = (
            max(errors) if errors else 0.0
        )

        return CommandResult(
            positions=positions,
            mode=mode,
            max_tracking_error_rad=float(
                maximum_error
            ),
        )

