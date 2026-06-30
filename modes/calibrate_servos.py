"""Zero IMU then run open-loop calibration sweep.

Generates a CSV of random servo commands and the corresponding IMU
attitude readings.  The offline tool scripts/download_calibration.py
fits the inverse map from this file.

State machine in periodic():
  1. ZEROING  — start sensor bias estimation
  2. MOVING   — command a random servo triplet via set_raw_n11()
  3. SETTLING — wait N cycles for mechanical settling
  4. SAMPLING — accumulate M IMU readings
  5. RECORD   — write average to CSV buffer, advance to next point
  6. DONE     — all points collected, write CSV
"""

from __future__ import annotations

import os
import random
import time as _time
from enum import Enum, auto
from typing import TYPE_CHECKING

import wpilib
from wpilib import PeriodicOpMode
from wpimath import Rotation3d

from config.bench_config import BenchConfig
from utilities.math_utils import average_rotations, quaternion_to_euler

if TYPE_CHECKING:
    from robot import Robot

from robot import utility  # noqa: E402 — runtime decorator


class Phase(Enum):
    ZEROING = auto()
    MOVING = auto()
    SETTLING = auto()
    SAMPLING = auto()
    RECORD = auto()
    DONE = auto()


@utility(name="Calibrate Servos", group="Calibration")
class CalibrateServosMode(PeriodicOpMode):
    def __init__(self, robot: Robot) -> None:
        super().__init__()
        self._robot = robot

        cfg = BenchConfig.calibration
        self._num_points = cfg.num_points
        self._settle_cycles = cfg.settle_cycles
        self._samples_per_point = cfg.samples_per_point
        self._storage_path = cfg.storage_path

        self._phase: Phase = Phase.ZEROING
        self._point_index = 0
        self._cycle_count = 0
        self._points: list[tuple[float, float, float]] = []
        self._sample_buffer: list[Rotation3d] = []
        self._results: list[tuple[float, float, float, float, float, float]] = []
        self._completed = False

    def start(self) -> None:
        self._robot.positioner.disable_feedback()
        self._phase = Phase.ZEROING
        self._point_index = 0
        self._cycle_count = 0
        self._points = []
        self._sample_buffer = []
        self._results = []
        self._robot.sensors.start_zeroing()

        for _ in range(self._num_points):
            self._points.append(
                (
                    random.uniform(-0.8, 0.8),
                    random.uniform(-0.8, 0.8),
                    random.uniform(-0.8, 0.8),
                )
            )

    def periodic(self) -> None:
        if self._phase is Phase.ZEROING:
            if self._robot.sensors.is_zeroed():
                self._phase = Phase.MOVING
            return

        if self._point_index >= self._num_points:
            self._phase = Phase.DONE
            self._completed = True
            self._write_csv()
            return

        sr, sp, sy = self._points[self._point_index]

        if self._phase is Phase.MOVING:
            self._robot.positioner.set_raw_n11(sp, sy, sr)
            self._phase = Phase.SETTLING
            self._cycle_count = 0

        elif self._phase is Phase.SETTLING:
            self._cycle_count += 1
            if self._cycle_count >= self._settle_cycles:
                self._phase = Phase.SAMPLING
                self._cycle_count = 0
                self._sample_buffer = []

        elif self._phase is Phase.SAMPLING:
            self._sample_buffer.append(self._robot.sensors.get_rotation())
            self._cycle_count += 1
            if self._cycle_count >= self._samples_per_point:
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

    def end(self) -> None:
        if self._completed and self._results:
            self._write_csv()
        self._robot.positioner.enable_feedback()

    def _write_csv(self) -> None:
        storage = (
            os.path.join(os.getcwd(), "calibration_data")
            if wpilib.RobotBase.isSimulation()
            else self._storage_path
        )
        os.makedirs(storage, exist_ok=True)
        ts = _time.strftime("%Y%m%d_%H%M%S")
        roborio_serial = wpilib.RobotController.getSerialNumber()
        path = os.path.join(storage, f"servo_calib_{ts}.csv")
        with open(path, "w", newline="") as f:
            f.write(f"# roboRIO serial: {roborio_serial}\n")
            f.write(f"# generated: {_time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(
                "servo_roll,servo_pitch,servo_yaw,"
                "actual_roll_rad,actual_pitch_rad,actual_yaw_rad\n"
            )
            for row in self._results:
                f.write(f"{row[0]},{row[1]},{row[2]},{row[3]},{row[4]},{row[5]}\n")
