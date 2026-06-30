"""Manual servo positioner control via joystick.

Maps joystick axes to pitch / yaw / roll goals within a ±45° range.

TODO: flesh out with SmartDashboard bindings for gain tuning and
individual-axis homing.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import wpilib
from wpilib import PeriodicOpMode

if TYPE_CHECKING:
    from robot import Robot

from robot import teleop  # noqa: E402


@teleop(name="Manual Operation", group="Bench")
class ManualControlMode(PeriodicOpMode):
    SENSITIVITY_DEG = 45.0

    def __init__(self, robot: Robot) -> None:
        super().__init__()
        self._robot = robot
        self._stick = wpilib.Joystick(0)

    def start(self) -> None:
        pass

    def periodic(self) -> None:
        pitch_rad = math.radians(-self._stick.getY() * self.SENSITIVITY_DEG)
        yaw_rad = math.radians(self._stick.getX() * self.SENSITIVITY_DEG)
        roll_rad = math.radians(self._stick.getZ() * self.SENSITIVITY_DEG)
        self._robot.positioner.set_goal_rad(pitch_rad, yaw_rad, roll_rad)

    def end(self) -> None:
        pass
