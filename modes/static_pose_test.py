"""Iterate a set of pre-planned camera poses where targets are visible.

At each pose the robot waits for the positioner to converge (PI + motion
profile), then samples N frames each of IMU attitude and PhotonVision
pose estimate and compares them.

Results CSV includes commanded pose, IMU-measured pose, PV-estimated pose,
and the per-axis translation / rotation errors.

TODOs (flesh out systematically):
  - Load the pose list from a config file or generate from ValidationConfig.
  - Implement convergence check via CameraPositioner.at_goal().
  - Sample IMU and PV in parallel over N frames.
  - Compare via utilities/measurement.py or comparison.py.
  - Write detailed CSV with all error metrics.
"""

from __future__ import annotations

import math
from enum import Enum, auto
from typing import TYPE_CHECKING

from wpilib import OpMode

from config.bench_config import BenchConfig

if TYPE_CHECKING:
    from robot import Robot

from robot import autonomous  # noqa: E402


class Phase(Enum):
    ZEROING = auto()
    MOVING = auto()
    CONVERGING = auto()
    MEASURING = auto()
    RECORD = auto()
    DONE = auto()


@autonomous(name="Static Pose Test", group="Validation")
class StaticPoseTest(OpMode):
    """Validate PV accuracy at a set of static poses.

    Phase machine:
      ZEROING → MOVING → CONVERGING → MEASURING → RECORD → (loop) → DONE
    """

    def __init__(self, robot: Robot) -> None:
        self._robot = robot

        vc = BenchConfig.validation
        self._settle_cycles = vc.static_settle_cycles
        self._samples_per_pose = vc.static_samples_per_pose
        self._storage_path = BenchConfig.calibration.storage_path

        # Build a list of (pitch_rad, yaw_rad, roll_rad) tuples from config.
        self._poses: list[tuple[float, float, float]] = []
        for pitch_deg, yaw_deg, roll_deg in vc.static_pose_deg:
            self._poses.append((
                math.radians(pitch_deg),
                math.radians(yaw_deg),
                math.radians(roll_deg),
            ))

        self._phase: Phase = Phase.ZEROING
        self._pose_index = 0
        self._cycle_count = 0
        self._completed = False

    def start(self) -> None:
        self._phase = Phase.ZEROING
        self._pose_index = 0
        self._cycle_count = 0
        self._completed = False
        self._robot.sensors.start_zeroing()

    def periodic(self) -> None:
        if self._phase is Phase.ZEROING:
            if self._robot.sensors.is_zeroed():
                self._phase = Phase.MOVING
            return

        if self._pose_index >= len(self._poses):
            self._phase = Phase.DONE
            self._completed = True
            return

        if self._phase is Phase.MOVING:
            pitch, yaw, roll = self._poses[self._pose_index]
            self._robot.positioner.set_goal_rad(pitch, yaw, roll)
            self._phase = Phase.CONVERGING
            self._cycle_count = 0

        elif self._phase is Phase.CONVERGING:
            self._cycle_count += 1
            if self._cycle_count >= self._settle_cycles:
                self._phase = Phase.MEASURING
                self._cycle_count = 0

        elif self._phase is Phase.MEASURING:
            # TODO: accumulate IMU + PV samples
            self._cycle_count += 1
            if self._cycle_count >= self._samples_per_pose:
                self._phase = Phase.RECORD

        elif self._phase is Phase.RECORD:
            # TODO: average samples, compute errors, write row
            self._pose_index += 1
            self._phase = Phase.MOVING

    def end(self) -> None:
        if self._completed:
            # TODO: flush accumulated results to CSV
            pass
