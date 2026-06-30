"""Re-run gyro-bias zeroing in isolation.

Phase sequence: ZEROING → DONE.
Useful if the IMU drifted or was bumped during a session.
"""

from __future__ import annotations

from enum import Enum, auto
from typing import TYPE_CHECKING

import wpilib
from wpilib import PeriodicOpMode

if TYPE_CHECKING:
    from robot import Robot

from robot import utility  # noqa: E402


class Phase(Enum):
    ZEROING = auto()
    DONE = auto()


@utility(name="Zero IMU", group="Calibration")
class ZeroIMUMode(PeriodicOpMode):
    def __init__(self, robot: Robot) -> None:
        super().__init__()
        self._robot = robot
        self._phase: Phase = Phase.ZEROING

    def start(self) -> None:
        self._robot.sensors.start_zeroing()
        self._phase = Phase.ZEROING

    def periodic(self) -> None:
        if self._phase is Phase.ZEROING and self._robot.sensors.is_zeroed():
            bias = self._robot.sensors.get_gyro_bias()
            wpilib.SmartDashboard.putNumberArray("imu/zeroed_bias", list(bias))
            self._phase = Phase.DONE

    def end(self) -> None:
        pass
