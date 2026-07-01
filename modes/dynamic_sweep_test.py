"""Smooth sinusoidal sweeps at increasing angular velocities to quantify
how pose-estimation error grows with angular velocity."""

from __future__ import annotations

import math
from enum import Enum, auto
from typing import TYPE_CHECKING

from wpilib import PeriodicOpMode

from config.bench_config import BenchConfig

if TYPE_CHECKING:
    from robot import Robot

from robot import autonomous  # noqa: E402


class Phase(Enum):
    """Named phase values for the dynamic-sweep state machine."""

    ZEROING = auto()
    SWEEP_PITCH = auto()
    SWEEP_YAW = auto()
    SWEEP_ROLL = auto()
    DONE = auto()


@autonomous(name="Dynamic Sweep Test", group="Validation")
class DynamicSweepTest(PeriodicOpMode):
    """PeriodicOpMode that drives sinusoidal positioner trajectories on
    each axis at stepped velocity levels while logging PhotonVision and
    IMU data for frequency-response analysis."""

    def __init__(self, robot: Robot) -> None:
        """Read sweep parameters from BenchConfig and initialise phase state."""
        super().__init__()
        self._robot = robot

        vc = BenchConfig.validation
        self._amplitude_rad = math.radians(vc.sweep_amplitude_deg)
        self._velocity_steps_radps = [
            math.radians(v) for v in vc.sweep_velocity_steps_degps
        ]
        self._duration_per_step_s = vc.sweep_duration_s_per_step
        self._storage_path = BenchConfig.validation.test_results_path

        self._phase: Phase = Phase.ZEROING
        self._axis: str = ""
        self._vel_idx = 0
        self._t = 0.0
        self._dt = 0.02
        self._completed = False

    def start(self) -> None:
        """Reset phase state and begin IMU zeroing."""
        self._phase = Phase.ZEROING
        self._axis = ""
        self._vel_idx = 0
        self._t = 0.0
        self._completed = False
        self._robot.sensors.start_zeroing()

    def periodic(self) -> None:
        """Advance the sweep state machine each cycle."""
        if self._phase is Phase.ZEROING:
            if self._robot.sensors.is_zeroed():
                self._phase = Phase.SWEEP_PITCH
                self._axis = "pitch"
                self._vel_idx = 0
                self._t = 0.0
            return

        if self._phase is Phase.SWEEP_PITCH:
            # TODO: run sinusoidal sweeps on pitch at each velocity step
            # When all velocities done → SWEEP_YAW
            pass

        elif self._phase is Phase.SWEEP_YAW:
            # TODO: sweep yaw axis
            pass

        elif self._phase is Phase.SWEEP_ROLL:
            # TODO: sweep roll axis
            pass

        elif self._phase is Phase.DONE:
            self._completed = True

    def end(self) -> None:
        """Flush logged data to CSV if the sweep completed."""
        if self._completed:
            # TODO: flush logged data to CSV
            pass
