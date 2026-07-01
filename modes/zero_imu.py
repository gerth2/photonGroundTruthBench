"""Re-run gyro-bias zeroing in isolation.
Useful if the IMU drifted or was bumped during a session."""

from __future__ import annotations

from enum import Enum, auto
from typing import TYPE_CHECKING

import wpilib
from wpilib import PeriodicOpMode

if TYPE_CHECKING:
    from robot import Robot

from robot import utility  # noqa: E402


class Phase(Enum):
    """Named phase values for the zero-IMU state machine."""

    ZEROING = auto()
    DONE = auto()


@utility(name="Zero IMU", group="Calibration")
class ZeroIMUMode(PeriodicOpMode):
    """PeriodicOpMode that re-runs gyro-bias estimation in isolation,
    then publishes the estimated bias to SmartDashboard."""

    def __init__(self, robot: Robot) -> None:
        """Store the Robot reference and set the initial phase."""
        super().__init__()
        self._robot = robot
        self._phase: Phase = Phase.ZEROING

    def start(self) -> None:
        """Start IMU zeroing and set the phase to ZEROING."""
        self._robot.sensors.start_zeroing()
        self._phase = Phase.ZEROING

    def periodic(self) -> None:
        """Wait for zeroing to complete, then publish bias and transition to DONE."""
        if self._phase is Phase.ZEROING and self._robot.sensors.is_zeroed():
            bias = self._robot.sensors.get_gyro_bias()
            wpilib.SmartDashboard.putNumberArray("imu/zeroed_bias", list(bias))
            self._phase = Phase.DONE

    def end(self) -> None:
        """No-op."""
        pass
