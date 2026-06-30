"""Sim-run each registered opmode for ~1 s to catch runtime crashes.

Uses raw WPILib sim APIs (not pyfrc) so it works with robotpy 2027.
"""

from __future__ import annotations

import os
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
from modes.static_pose_test import StaticPoseTest, WindowResult  # noqa: E402


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


# ── Profile advancement tests ──────────────────────────────────────


def test_profile_does_not_snap_on_first_call(robot: Robot) -> None:
    """set_goal_rad() + one periodic() must NOT snap profiled to goal."""
    pos = robot.positioner
    pos.set_goal_rad(pitch_rad=0.5, yaw_rad=0.0, roll_rad=0.0)
    pos.periodic()
    after = pos._profiled_pitch
    assert after < 0.5 - 1e-6, f"Snapped to goal: {after}"


def test_profile_advances_smoothly(robot: Robot) -> None:
    """Repeated periodic() calls advance profiled pitch toward goal."""
    pos = robot.positioner
    pos.set_goal_rad(pitch_rad=0.5, yaw_rad=0.0, roll_rad=0.0)

    samples = []
    for _ in range(200):
        pos.periodic()
        samples.append(pos._profiled_pitch)

    # Monotonically increasing
    for i in range(1, len(samples)):
        assert samples[i] >= samples[i - 1] - 1e-9, (
            f"Not monotonic at step {i}: {samples[i - 1]} → {samples[i]}"
        )

    # Did not snap on first step
    assert 0.0 < samples[0] < 0.01, f"First profiled value unexpected: {samples[0]}"

    # After 4 s at 10 deg/s the profiled pitch should be at the goal.
    assert abs(samples[-1] - 0.5) < 0.01, (
        f"Did not converge to 0.5 after 4 s: {samples[-1]}"
    )


# ── profile_finished tests ─────────────────────────────────────────


def _reset_positioner(pos: object) -> None:
    """Wipe the positioner's goal and profiled state so next test starts clean."""
    pos._desired_pitch = None  # type: ignore[attr-defined]
    pos._desired_yaw = None  # type: ignore[attr-defined]
    pos._desired_roll = None  # type: ignore[attr-defined]
    pos._profiled_pitch = 0.0  # type: ignore[attr-defined]
    pos._profiled_yaw = 0.0  # type: ignore[attr-defined]
    pos._profiled_roll = 0.0  # type: ignore[attr-defined]
    pos._profiled_pitch_vel = 0.0  # type: ignore[attr-defined]
    pos._profiled_yaw_vel = 0.0  # type: ignore[attr-defined]
    pos._profiled_roll_vel = 0.0  # type: ignore[attr-defined]


def test_profile_finished_no_goal(robot: Robot) -> None:
    """profile_finished returns False when no goal has been commanded."""
    _reset_positioner(robot.positioner)
    assert not robot.positioner.profile_finished()


def test_profile_finished_at_origin(robot: Robot) -> None:
    """profile_finished returns True when goal matches profiled (both 0)."""
    pos = robot.positioner
    _reset_positioner(pos)
    pos.set_goal_rad(pitch_rad=0.0, yaw_rad=0.0, roll_rad=0.0)
    assert pos.profile_finished()


def test_profile_finished_not_yet_converged(robot: Robot) -> None:
    """profile_finished returns False right after set_goal_rad with non-zero."""
    pos = robot.positioner
    _reset_positioner(pos)
    pos.set_goal_rad(pitch_rad=0.5, yaw_rad=0.0, roll_rad=0.0)
    assert not pos.profile_finished(), (
        "Profile should not be finished immediately after setting a non-zero goal"
    )


def test_profile_finished_after_convergence(robot: Robot) -> None:
    """profile_finished returns True once the profile reaches the goal."""
    pos = robot.positioner
    _reset_positioner(pos)
    pos.set_goal_rad(pitch_rad=0.5, yaw_rad=0.0, roll_rad=0.0)

    for _ in range(300):
        pos.periodic()
        if pos.profile_finished():
            return

    assert False, "Profile did not converge to 0.5 rad after 300 cycles (6 s)"


# ── Storage-path fallback tests ────────────────────────────────────


def test_static_pose_storage_uses_cwd_in_sim(robot: Robot) -> None:
    """_flush_csv must use cwd-relative path when RobotBase.isSimulation() is True."""
    mode = StaticPoseTest(robot)
    mode._results = [
        WindowResult(
            pose_idx=0,
            expected_tags=(6, 7),
            cmd_r=0.0,
            cmd_p=0.0,
            cmd_y=0.0,
            imu_count=50,
            pv_count=10,
            imu_mean=(0.0, 0.0, 0.0),
            imu_std=(0.0, 0.0, 0.0),
            rms_errors={
                "dx": 0.001,
                "dy": 0.002,
                "dz": 0.003,
                "droll": 0.01,
                "dpitch": 0.02,
                "dyaw": 0.03,
            },
        )
    ]
    # Configured path is irrelevant when isSimulation() — must use cwd-relative.
    mode._storage_path = "/some/unwritable/path"
    mode._flush_csv()

    expected = os.path.join(os.getcwd(), "test_results", "static_pose_results.csv")
    assert os.path.isfile(expected), f"CSV not written to cwd-relative path: {expected}"

    os.remove(expected)
    os.rmdir(os.path.dirname(expected))
