"""Sim-run each registered opmode for ~1 s to catch runtime crashes.

Uses raw WPILib sim APIs (not pyfrc) so it works with robotpy 2027.
"""

from __future__ import annotations

import sys

import pytest
from wpilib import simulation as wsim
from wpilib.simulation import stepTiming

# Import modes at module level to populate _registry before Robot is created.
sys.path.insert(0, "")
import modes.calibrate_servos  # noqa: F401
import modes.dynamic_sweep_test  # noqa: F401
import modes.manual_control  # noqa: F401
import modes.static_pose_test  # noqa: F401
import modes.zero_imu  # noqa: F401

from robot import Robot, _registry  # noqa: E402


# ── Fixtures ──────────────────────────────────────────────────────────


@pytest.fixture(scope="module")
def robot() -> Robot:
    """Create the Robot once; HAL/PWM can't be re-initialised per test."""
    robot_instance = Robot()

    # Sanity check: all 5 modes should be visible to the sim DS
    opts = wsim.DriverStationSim.getOpModeOptions()
    assert len(opts) == 5, f"Expected 5 opmodes, got {len(opts)}"

    return robot_instance


# ── Helpers ───────────────────────────────────────────────────────────


def _run_opmode(robot: Robot, cls: type, label: str) -> None:
    """Construct *cls*, run start + 50×periodic + end, expect no crash."""
    try:
        instance = cls(robot)
    except TypeError:
        instance = cls()

    instance.start()

    for _ in range(50):  # 50 × 20 ms = 1 s
        instance.periodic()
        robot.robotPeriodic()
        stepTiming(0.02)

    instance.end()


# ── Tests ─────────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    "entry",
    [(cls, name) for cls, _mode, name, _group, _desc in _registry],
    ids=lambda x: x[1] if isinstance(x, tuple) else str(x),
)
def test_opmode_runs(robot: Robot, entry: tuple[type, str]) -> None:
    """Each opmode must survive 1 s of simulated run time."""
    cls, name = entry
    _run_opmode(robot, cls, name)
