"""Mahony complementary filter for IMU attitude estimation.

Fuses gyroscope integration (fast, drifts) with accelerometer gravity-vector
measurement (slow, no roll/pitch drift) into a unit quaternion.

Gyro dominates on short timescales; the proportional term (kp) corrects
gradual drift using the accelerometer's estimate of "down".  The integral
term (ki) tracks gyro bias over longer periods.

Yaw is unobservable from accelerometer-only correction (no magnetometer),
so it drifts freely -- acceptable for short bench runs.
"""

import math

from wpimath import Quaternion, Rotation3d

from utilities.math_utils import rotation3d_to_euler


class MahonyFilter:
    """Complementary (Mahony) filter fusing gyroscope integration with accelerometer gravity-vector correction into a unit quaternion. Updated incrementally via update(); the caller supplies gyro and accel readings each cycle. Yaw drifts freely (no magnetometer)."""

    def __init__(self, kp: float = 0.5, ki: float = 0.0) -> None:
        """Initialise filter gains and reset quaternion state to identity."""
        self._kp = kp
        self._ki = ki

        self._qw = 1.0
        self._qx = 0.0
        self._qy = 0.0
        self._qz = 0.0

        self._bias_x = 0.0
        self._bias_y = 0.0
        self._bias_z = 0.0

    def reset(self) -> None:
        """Reset quaternion state to identity (zero rotation)."""
        self._qw = 1.0
        self._qx = 0.0
        self._qy = 0.0
        self._qz = 0.0

    def set_gyro_bias(self, bx: float, by: float, bz: float) -> None:
        """Set the gyroscope bias offsets subtracted from each raw reading."""
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
        """Advance the filter one timestep with gyroscope and accelerometer readings and the elapsed time in seconds."""
        gx = gyro_x - self._bias_x
        gy = gyro_y - self._bias_y
        gz = gyro_z - self._bias_z

        a_norm = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        if a_norm < 1e-6:
            self._integrate_gyro(gx, gy, gz, dt)
            return

        ax = accel_x / a_norm
        ay = accel_y / a_norm
        az = accel_z / a_norm

        g_est_x = 2.0 * (self._qx * self._qz - self._qw * self._qy)
        g_est_y = 2.0 * (self._qw * self._qx + self._qy * self._qz)
        g_est_z = (
            self._qw * self._qw
            - self._qx * self._qx
            - self._qy * self._qy
            + self._qz * self._qz
        )

        err_x = ay * g_est_z - az * g_est_y
        err_y = az * g_est_x - ax * g_est_z
        err_z = ax * g_est_y - ay * g_est_x

        if self._ki > 0.0:
            self._bias_x += self._ki * err_x * dt
            self._bias_y += self._ki * err_y * dt
            self._bias_z += self._ki * err_z * dt

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

        norm = math.sqrt(self._qw**2 + self._qx**2 + self._qy**2 + self._qz**2)
        if norm > 1e-6:
            inv_norm = 1.0 / norm
            self._qw *= inv_norm
            self._qx *= inv_norm
            self._qy *= inv_norm
            self._qz *= inv_norm

    def get_rotation(self) -> Rotation3d:
        """Return the current attitude estimate as a wpimath Rotation3d."""
        return Rotation3d(Quaternion(self._qw, self._qx, self._qy, self._qz))

    def get_euler_angles(self) -> tuple[float, float, float]:
        """Convenience: return the current attitude as (roll, pitch, yaw) in radians."""
        return rotation3d_to_euler(self.get_rotation())

    def get_gyro_bias(self) -> tuple[float, float, float]:
        """Return the current gyroscope bias estimate as (bx, by, bz)."""
        return (self._bias_x, self._bias_y, self._bias_z)
