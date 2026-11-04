"""Top-level mission sequencer: Stand -> Shift -> Step -> Walk -> Stop.

This is the "Humanoid mission manager" box at the top of the HUMANOID-LAB
architecture diagram. It issues the high-level phase; the gait planner,
balance controller and kinematics packages turn each phase into joint
commands. It also watches the safety supervisor's state (passed in each
step() call) and immediately drops to STOP/HOLD if the robot is not in a
nominal safety state, so a fault always overrides whatever the mission was
doing.
"""
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

from humanoid_safety.safety_fsm import SafetyState


class MissionPhase(Enum):
    IDLE = auto()
    STAND = auto()
    SHIFT = auto()
    STEP = auto()
    WALK = auto()
    STOP = auto()
    HOLD = auto()  # safety override: freeze and wait


@dataclass
class MissionCommand:
    start: bool = False
    begin_step: bool = False
    begin_walk: bool = False
    stop: bool = False
    steps_remaining: int = 0


class MissionManagerFSM:
    def __init__(self):
        self.phase = MissionPhase.IDLE

    def step(self, cmd: MissionCommand, safety_state: SafetyState) -> MissionPhase:
        # Safety overrides everything else.
        if safety_state in (SafetyState.FAULT_DETECTED, SafetyState.RECOVERY, SafetyState.SAFE_HOLD):
            self.phase = MissionPhase.HOLD
            return self.phase

        if self.phase == MissionPhase.HOLD and safety_state in (SafetyState.STAND, SafetyState.WALK):
            self.phase = MissionPhase.STAND  # safety cleared -> re-enter at STAND

        if self.phase == MissionPhase.IDLE:
            if cmd.start:
                self.phase = MissionPhase.STAND
            return self.phase

        if self.phase == MissionPhase.STAND:
            if cmd.stop:
                self.phase = MissionPhase.STOP
            elif cmd.begin_walk:
                self.phase = MissionPhase.SHIFT
            elif cmd.begin_step:
                self.phase = MissionPhase.SHIFT
            return self.phase

        if self.phase == MissionPhase.SHIFT:
            # Weight has been shifted onto the stance foot -> take the step.
            self.phase = MissionPhase.STEP
            return self.phase

        if self.phase == MissionPhase.STEP:
            if cmd.stop:
                self.phase = MissionPhase.STOP
            elif cmd.steps_remaining > 0:
                self.phase = MissionPhase.WALK if cmd.begin_walk else MissionPhase.SHIFT
            else:
                self.phase = MissionPhase.STAND
            return self.phase

        if self.phase == MissionPhase.WALK:
            if cmd.stop or cmd.steps_remaining <= 0:
                self.phase = MissionPhase.SHIFT if cmd.steps_remaining > 0 else MissionPhase.STOP
            return self.phase

        if self.phase == MissionPhase.STOP:
            self.phase = MissionPhase.STAND
            return self.phase

        return self.phase
