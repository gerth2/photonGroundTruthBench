"""Slew all servos to their mechanical centre (N11 = 0, 0, 0) via a
linear ramp."""

from __future__ import annotations

from typing import TYPE_CHECKING

from wpilib import PeriodicOpMode

from robot import teleop  # noqa: E402

if TYPE_CHECKING:
    from robot import Robot


@teleop(name="Home Servos", group="Bench")
class HomeServosMode(PeriodicOpMode):
    """PeriodicOpMode that commands all three servos to centre position
    with a smooth linear slew."""

    SLEW_DURATION_S = 1.5

    def __init__(self, robot: Robot) -> None:
        """Store a reference to the Robot instance."""
        super().__init__()
        self._robot = robot

    def start(self) -> None:
        """Begin a linear slew of all servos to centre."""
        self._robot.positioner.set_target_n11(
            0.0, 0.0, 0.0, self.SLEW_DURATION_S
        )

    def periodic(self) -> None:
        """No-op; the slew runs autonomously in CameraPositioner.periodic()."""
        pass

    def end(self) -> None:
        """No-op."""
        pass
