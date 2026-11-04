"""Straight-line alternating-foot footstep sequence generator.

This produces the *nominal* footstep plan (Stage 4). Reactive corrections
from the balance controller's capture-point estimate (Stage 5) are applied
downstream, on top of this nominal plan -- see
humanoid_balance_controller.capture_point.adjust_footstep_for_icp.
"""
from dataclasses import dataclass
from typing import List


@dataclass
class Footstep:
    index: int
    foot: str       # "left" or "right"
    x: float        # forward position of the foot, in the world/odom frame (m)
    y: float        # lateral position of the foot (m)
    theta: float = 0.0  # foot yaw (rad); 0.0 for straight-line walking


def plan_straight_walk(
    num_steps: int,
    stride_length: float,
    step_width: float,
    start_foot: str = "right",
    start_x: float = 0.0,
) -> List[Footstep]:
    """Plan `num_steps` alternating footsteps walking straight in +x.

    The first two steps are a shorter "start-up" half-stride (common
    practice: begin walking with a half step so the CoM has time to shift
    over the stance foot before full-length strides begin).
    """
    if num_steps < 1:
        raise ValueError("num_steps must be >= 1")
    if start_foot not in ("left", "right"):
        raise ValueError("start_foot must be 'left' or 'right'")

    steps: List[Footstep] = []
    foot = start_foot
    x = start_x
    for i in range(num_steps):
        length = stride_length * 0.5 if i == 0 else stride_length
        x += length
        y = step_width / 2.0 if foot == "left" else -step_width / 2.0
        steps.append(Footstep(index=i, foot=foot, x=x, y=y))
        foot = "left" if foot == "right" else "right"
    return steps
