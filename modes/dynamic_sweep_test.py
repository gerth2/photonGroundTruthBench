"""Smooth sinusoidal sweeps at increasing angular velocities.

For each axis (pitch, yaw, roll) and each velocity step the robot
drives a sinusoid while logging PV position estimates continuously.
The resulting data lets us quantify how pose-estimation error grows
with angular velocity.

Trajectory:
  θ(t) = A · sin(2π · f · t)
  f = v_peak / (2π · A)    so peak angular velocity = A · 2πf = v_peak

Phase machine per axis:
  ZEROING → SWEEPING_AXIS (each vel step) → (next axis) → DONE

TODOs (flesh out systematically):
  - Implement setpoint generation (sinusoid) per axis.
  - Log IMU attitude + PV estimate + commanded angle at each cycle.
  - Compare measured vs. commanded to extract phase lag and amplitude
    attenuation (classic frequency-response analysis).
  - Write CSV with columns: timestamp, axis, commanded, imu, pv, vel_step.
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
    SWEEP_PITCH = auto()
    SWEEP_YAW = auto()
    SWEEP_ROLL = auto()
    DONE = auto()


@autonomous(name="Dynamic Sweep Test", group="Validation")
class DynamicSweepTest(OpMode):
    """Frequency-response analysis of PV at increasing angular rates.

    Phase machine:
      ZEROING → SWEEP_PITCH → SWEEP_YAW → SWEEP_ROLL → DONE
    """

    def __init__(self, robot: Robot) -> None:
        self._robot = robot

        vc = BenchConfig.validation
        self._amplitude_rad = math.radians(vc.sweep_amplitude_deg)
        self._velocity_steps_radps = [
            math.radians(v) for v in vc.sweep_velocity_steps_degps
        ]
        self._cycles_per_step = vc.sweep_cycles_per_step
        self._storage_path = BenchConfig.calibration.storage_path

        self._phase: Phase = Phase.ZEROING
        self._axis: str = ""
        self._vel_idx = 0
        self._t = 0.0
        self._dt = 0.02
        self._completed = False

    def start(self) -> None:
        self._phase = Phase.ZEROING
        self._axis = ""
        self._vel_idx = 0
        self._t = 0.0
        self._completed = False
        self._robot.sensors.start_zeroing()

    def periodic(self) -> None:
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
        if self._completed:
            # TODO: flush logged data to CSV
            pass
