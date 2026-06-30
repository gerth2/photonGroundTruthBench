"""Mahony complementary filter for IMU attitude estimation.

Fuses gyroscope integration (fast, drifts) with accelerometer gravity-vector
measurement (slow, no roll/pitch drift) into a unit quaternion.

Gyro dominates on short timescales; the proportional term (kp) corrects
gradual drift using the accelerometer's estimate of "down".  The integral
term (ki) tracks gyro bias over longer periods.

Yaw is unobservable from accelerometer-only correction (no magnetometer),
so it drifts freely — acceptable for short bench runs.
"""

import math

from wpimath import Quaternion, Rotation3d

from utilities.math_utils import rotation3d_to_euler


class MahonyFilter:
    def __init__(self, kp: float = 0.5, ki: float = 0.0) -> None:
        self._kp = kp
        self._ki = ki

        # Quaternion state: identity initially (gravity aligned with -Z).
        self._qw = 1.0
        self._qx = 0.0
        self._qy = 0.0
        self._qz = 0.0

        # Running bias estimate subtracted from raw gyro readings.
        self._bias_x = 0.0
        self._bias_y = 0.0
        self._bias_z = 0.0

    def reset(self) -> None:
        self._qw = 1.0
        self._qx = 0.0
        self._qy = 0.0
        self._qz = 0.0

    def set_gyro_bias(self, bx: float, by: float, bz: float) -> None:
        self._bias_x = bx
        self._bias_y = by
        self._bias_z = bz

    def update(
        self,
        gyro_x: float,
        gyro_y: float,
        gyro_z: float,
        accel_x: float,
        accel_y: float,
        accel_z: float,
        dt: float,
    ) -> None:
        # Remove estimated bias from raw gyro readings.
        gx = gyro_x - self._bias_x
        gy = gyro_y - self._bias_y
        gz = gyro_z - self._bias_z

        # Normalise accelerometer to get unit gravity vector.
        a_norm = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        if a_norm < 1e-6:
            self._integrate_gyro(gx, gy, gz, dt)
            return

        ax = accel_x / a_norm
        ay = accel_y / a_norm
        az = accel_z / a_norm

        # Estimate gravity direction from current quaternion (rotate +Z
        # by the inverse quaternion, then take the third column).
        g_est_x = 2.0 * (self._qx * self._qz - self._qw * self._qy)
        g_est_y = 2.0 * (self._qw * self._qx + self._qy * self._qz)
        g_est_z = (
            self._qw * self._qw
            - self._qx * self._qx
            - self._qy * self._qy
            + self._qz * self._qz
        )

        # Cross product between measured and estimated gravity → error.
        err_x = ay * g_est_z - az * g_est_y
        err_y = az * g_est_x - ax * g_est_z
        err_z = ax * g_est_y - ay * g_est_x

        # Integral term accumulates gyro bias over time.
        if self._ki > 0.0:
            self._bias_x += self._ki * err_x * dt
            self._bias_y += self._ki * err_y * dt
            self._bias_z += self._ki * err_z * dt

        # Correct gyro reading with proportional feedback.
        gx += self._kp * err_x
        gy += self._kp * err_y
        gz += self._kp * err_z

        self._integrate_gyro(gx, gy, gz, dt)

    def _integrate_gyro(self, gx: float, gy: float, gz: float, dt: float) -> None:
        """First-order quaternion integration with unit-normalisation."""
        half_dt = 0.5 * dt

        dqw = (-self._qx * gx - self._qy * gy - self._qz * gz) * half_dt
        dqx = (self._qw * gx + self._qy * gz - self._qz * gy) * half_dt
        dqy = (self._qw * gy + self._qz * gx - self._qx * gz) * half_dt
        dqz = (self._qw * gz + self._qx * gy - self._qy * gx) * half_dt

        self._qw += dqw
        self._qx += dqx
        self._qy += dqy
        self._qz += dqz

        # Re-normalise to prevent drift from integration error.
        norm = math.sqrt(self._qw**2 + self._qx**2 + self._qy**2 + self._qz**2)
        if norm > 1e-6:
            inv_norm = 1.0 / norm
            self._qw *= inv_norm
            self._qx *= inv_norm
            self._qy *= inv_norm
            self._qz *= inv_norm

    def get_rotation(self) -> Rotation3d:
        return Rotation3d(Quaternion(self._qw, self._qx, self._qy, self._qz))

    def get_euler_angles(self) -> tuple[float, float, float]:
        """Convenience: decompose internal quaternion to (roll, pitch, yaw)."""
        return rotation3d_to_euler(self.get_rotation())

    def get_gyro_bias(self) -> tuple[float, float, float]:
        return (self._bias_x, self._bias_y, self._bias_z)
