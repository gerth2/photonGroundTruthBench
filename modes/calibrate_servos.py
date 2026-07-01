"""State machine that drives the servo positioner through random N11
triplets while recording IMU attitude for calibration-map fitting."""

from __future__ import annotations

import os
import random
import time as _time
from enum import Enum, auto
from typing import TYPE_CHECKING

import wpilib
from wpilib import PeriodicOpMode, Timer
from wpimath import Rotation3d

from config.bench_config import BenchConfig
from utilities.math_utils import average_rotations, quaternion_to_euler

if TYPE_CHECKING:
    from robot import Robot

from robot import utility  # noqa: E402


class Phase(Enum):
    """Named phase values for the calibrate-servos state machine."""

    HOME = auto()
    ZEROING = auto()
    MOVING = auto()
    SLEWING = auto()
    SETTLING = auto()
    SAMPLING = auto()
    RECORD = auto()
    DONE = auto()


@utility(name="Calibrate Servos", group="Calibration")
class CalibrateServosMode(PeriodicOpMode):
    """PeriodicOpMode that homes servos, zeros the IMU, then iterates
    through random servo positions, recording ground-truth IMU rotation
    at each point."""

    def __init__(self, robot: Robot) -> None:
        """Configure the state machine from BenchConfig calibration parameters."""
        super().__init__()
        self._robot = robot

        cfg = BenchConfig.calibration
        self._num_points = cfg.num_points
        self._slew_duration_s = cfg.slew_duration_s
        self._settle_duration_s = cfg.settle_duration_s
        self._sample_duration_s = cfg.sample_duration_s
        self._storage_path = cfg.storage_path

        self._phase: Phase = Phase.HOME
        self._point_index = 0
        self._phase_start_time = 0.0
        self._points: list[tuple[float, float, float]] = []
        self._sample_buffer: list[Rotation3d] = []
        self._results: list[tuple[float, float, float, float, float, float]] = []
        self._completed = False

    def start(self) -> None:
        """Home the positioner, reset state, and generate the random point list."""
        self._robot.positioner.disable_feedback()
        self._point_index = 0
        self._sample_buffer = []
        self._results = []

        self._points = [
            (
                random.uniform(-0.8, 0.8),
                random.uniform(-0.8, 0.8),
                random.uniform(-0.8, 0.8),
            )
            for _ in range(self._num_points)
        ]

        self._robot.positioner.set_raw_n11(0.0, 0.0, 0.0)
        self._phase = Phase.HOME
        self._phase_start_time = Timer.getTimestamp()

    def periodic(self) -> None:
        """Advance the calibration state machine each cycle."""
        now = Timer.getTimestamp()

        if self._phase is Phase.HOME:
            if now - self._phase_start_time >= self._settle_duration_s:
                self._phase = Phase.ZEROING
                self._robot.sensors.start_zeroing()
            return

        if self._phase is Phase.ZEROING:
            if self._robot.sensors.is_zeroed():
                self._phase = Phase.MOVING
            return

        if self._point_index >= self._num_points or not self._points:
            self._phase = Phase.DONE
            self._completed = True
            self._write_csv()
            return

        sr, sp, sy = self._points[self._point_index]

        if self._phase is Phase.MOVING:
            self._robot.positioner.set_target_n11(
                sp, sy, sr, self._slew_duration_s
            )
            self._phase = Phase.SLEWING
            self._phase_start_time = now

        elif self._phase is Phase.SLEWING:
            if self._robot.positioner.n11_slew_finished():
                self._phase = Phase.SETTLING
                self._phase_start_time = now

        elif self._phase is Phase.SETTLING:
            if now - self._phase_start_time >= self._settle_duration_s:
                self._phase = Phase.SAMPLING
                self._phase_start_time = now
                self._sample_buffer = []

        elif self._phase is Phase.SAMPLING:
            self._sample_buffer.append(self._robot.sensors.get_rotation())
            if now - self._phase_start_time >= self._sample_duration_s:
                self._phase = Phase.RECORD

        elif self._phase is Phase.RECORD:
            avg = average_rotations(self._sample_buffer)
            q = avg.getQuaternion()
            roll_rad, pitch_rad, yaw_rad = quaternion_to_euler(
                q.W(), q.X(), q.Y(), q.Z()
            )
            self._results.append((sr, sp, sy, roll_rad, pitch_rad, yaw_rad))
            self._point_index += 1
            self._phase = Phase.MOVING
            self._phase_start_time = now

    def end(self) -> None:
        """Write CSV if calibration completed, then re-enable positioner feedback."""
        if self._completed and self._results:
            self._write_csv()
        self._robot.positioner.enable_feedback()

    def _write_csv(self) -> None:
        """Write the accumulated (servo, actual) results to a timestamped CSV file."""
        storage = (
            os.path.join(os.getcwd(), "calibration_data")
            if wpilib.RobotBase.isSimulation()
            else self._storage_path
        )
        os.makedirs(storage, exist_ok=True)
        ts = _time.strftime("%Y%m%d_%H%M%S")
        systemcore_serial = wpilib.RobotController.getSerialNumber()
        path = os.path.join(storage, f"servo_calib_{ts}.csv")
        with open(path, "w", newline="") as f:
            f.write(f"# SystemCore serial: {systemcore_serial}\n")
            f.write(f"# generated: {_time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(
                "servo_roll,servo_pitch,servo_yaw,"
                "actual_roll_rad,actual_pitch_rad,actual_yaw_rad\n"
            )
            for row in self._results:
                f.write(f"{row[0]},{row[1]},{row[2]},{row[3]},{row[4]},{row[5]}\n")
