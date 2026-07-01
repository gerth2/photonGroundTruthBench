"""Manual servo positioner control via joystick.
Maps joystick axes to pitch / yaw / roll goals within a ±45° range."""

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
    """PeriodicOpMode that maps a joystick's Y/X/Z axes to pitch/yaw/roll
    servo goals within a fixed angular range."""

    SENSITIVITY_DEG = 45.0

    def __init__(self, robot: Robot) -> None:
        """Store the Robot reference and create the Joystick object."""
        super().__init__()
        self._robot = robot
        self._stick = wpilib.Joystick(0)

    def start(self) -> None:
        """No-op."""
        pass

    def periodic(self) -> None:
        """Read joystick axes and command the positioner to the corresponding angular goal."""
        pitch_rad = math.radians(-self._stick.getY() * self.SENSITIVITY_DEG)
        yaw_rad = math.radians(self._stick.getX() * self.SENSITIVITY_DEG)
        roll_rad = math.radians(self._stick.getZ() * self.SENSITIVITY_DEG)
        self._robot.positioner.set_goal_rad(pitch_rad, yaw_rad, roll_rad)

    def end(self) -> None:
        """No-op."""
        pass
