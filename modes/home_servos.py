"""Slew all servos back to the configured centre position.

Reads the current n11 position on start, then delegates the linear
ramp to ``CameraPositioner.set_target_n11()``.  Periodic() is a no-op
— the positioner handles the slew internally.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from wpilib import PeriodicOpMode

from robot import teleop  # noqa: E402

if TYPE_CHECKING:
    from robot import Robot


@teleop(name="Home Servos", group="Bench")
class HomeServosMode(PeriodicOpMode):
    SLEW_DURATION_S = 1.5

    def __init__(self, robot: Robot) -> None:
        super().__init__()
        self._robot = robot

    def start(self) -> None:
        self._robot.positioner.set_target_n11(
            0.0, 0.0, 0.0, self.SLEW_DURATION_S
        )

    def periodic(self) -> None:
        pass

    def end(self) -> None:
        pass
