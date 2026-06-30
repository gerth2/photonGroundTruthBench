"""Pure-math utilities for quaternion ↔ Euler conversion and averaging."""

import math

from wpimath import Quaternion, Rotation3d


def quaternion_to_euler(
    w: float, x: float, y: float, z: float
) -> tuple[float, float, float]:
    """Convert a normalised quaternion (w, x, y, z) to (roll, pitch, yaw) in radians.

    ZYX intrinsic Tait-Bryan convention, matching Rotation3d internal layout.
    Gimbal-lock at |pitch| = π/2 is handled by copysign.
    """
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


def rotation3d_to_euler(r: Rotation3d) -> tuple[float, float, float]:
    """Shorthand: pull the quaternion from a Rotation3d and convert."""
    q = r.getQuaternion()
    return quaternion_to_euler(q.W(), q.X(), q.Y(), q.Z())


def average_rotations(rotations: list[Rotation3d]) -> Rotation3d:
    """Spherical linear average of a list of Rotation3d via quaternion summing.

    Antipodal quaternions (which represent the same rotation but have a
    negative dot-product) are flipped before summing to avoid cancellation.
    The summed quaternion is normalised to unit length.
    """
    if not rotations:
        return Rotation3d()

    qs = [r.getQuaternion() for r in rotations]
    q0 = qs[0]
    s0, s1, s2, s3 = 0.0, 0.0, 0.0, 0.0
    for q in qs:
        dot = q.W() * q0.W() + q.X() * q0.X() + q.Y() * q0.Y() + q.Z() * q0.Z()
        flip = -1.0 if dot < 0.0 else 1.0
        s0 += q.W() * flip
        s1 += q.X() * flip
        s2 += q.Y() * flip
        s3 += q.Z() * flip
    norm = math.sqrt(s0**2 + s1**2 + s2**2 + s3**2)
    if norm < 1e-9:
        return Rotation3d()
    return Rotation3d(Quaternion(s0 / norm, s1 / norm, s2 / norm, s3 / norm))
