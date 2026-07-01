"""Unit tests for the Mahony complementary filter and related math utilities."""

import math

from utilities.imu_filter import MahonyFilter
from utilities.math_utils import average_rotations, quaternion_to_euler
from wpimath import Rotation3d


def test_quaternion_to_euler_identity() -> None:
    """Verify that the identity quaternion (1,0,0,0) maps to zero Euler angles."""
    r, p, y = quaternion_to_euler(1.0, 0.0, 0.0, 0.0)
    assert abs(r) < 1e-9
    assert abs(p) < 1e-9
    assert abs(y) < 1e-9


def test_quaternion_to_euler_pitch() -> None:
    """Verify a pure pitch rotation of 30 degrees is correctly decomposed."""
    half = math.sin(math.radians(15))
    w = math.cos(math.radians(15))
    r, p, y = quaternion_to_euler(w, 0.0, half, 0.0)
    assert abs(r) < 1e-6
    assert abs(math.degrees(p) - 30.0) < 0.001
    assert abs(y) < 1e-6


def test_mahony_filter_gyro_only() -> None:
    """Verify the filter integrates a constant yaw rate correctly with zero proportional feedback."""
    f = MahonyFilter(kp=0.0, ki=0.0)
    dt = 0.02
    for _ in range(100):
        f.update(0.0, 0.0, 0.5, 0.0, 0.0, 9.81, dt)
    r, p, y = f.get_euler_angles()
    assert abs(y - 1.0) < 0.02


def test_mahony_filter_reset() -> None:
    """Verify that resetting the filter returns the attitude estimate to identity."""
    f = MahonyFilter()
    f.update(0.1, 0.0, 0.0, 0.0, 0.0, 9.81, 0.02)
    f.reset()
    r, p, y = f.get_euler_angles()
    assert abs(r) < 1e-6
    assert abs(p) < 1e-6
    assert abs(y) < 1e-6


def test_mahony_filter_stationary_gravity() -> None:
    """Verify that a stationary sensor aligned with gravity produces near-zero roll and pitch."""
    f = MahonyFilter(kp=0.5, ki=0.0)
    for _ in range(50):
        f.update(0.0, 0.0, 0.0, 0.0, 0.0, 9.81, 0.02)
    r, p, y = f.get_euler_angles()
    assert abs(r) < 0.1
    assert abs(p) < 0.1
    assert abs(y) < 0.1


def test_average_rotations_single() -> None:
    """Verify that averaging a single identity rotation returns the identity."""
    r = Rotation3d()
    avg = average_rotations([r])
    q = avg.getQuaternion()
    assert abs(q.W() - 1.0) < 1e-6


def test_average_rotations_empty() -> None:
    """Verify that averaging an empty list returns the identity rotation."""
    avg = average_rotations([])
    q = avg.getQuaternion()
    assert abs(q.W() - 1.0) < 1e-6


def test_average_rotations_two() -> None:
    """Verify that averaging two opposing small rotations returns near-identity."""
    r1 = Rotation3d(0.1, 0.0, 0.0)
    r2 = Rotation3d(-0.1, 0.0, 0.0)
    avg = average_rotations([r1, r2])
    q = avg.getQuaternion()
    assert abs(q.X()) < 0.01


def test_get_rotation_vs_get_euler() -> None:
    """Verify that get_rotation and get_euler_angles are consistent after a single update."""
    f = MahonyFilter()
    f.update(0.05, 0.02, 0.01, 0.1, 0.0, 9.81, 0.02)
    e2 = f.get_euler_angles()
    assert len(e2) == 3
