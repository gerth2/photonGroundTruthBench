import wpilib
from wpilib import Timer
from wpimath import Rotation3d

from utilities.imu_filter import MahonyFilter
from core.subsystem import Subsystem
from hardware.mpu6050 import MPU6050


class GroundTruthSensors(Subsystem):
    def __init__(
        self,
        imu: MPU6050,
        filter_kp: float = 0.5,
        filter_ki: float = 0.0,
        zeroing_samples: int = 100,
    ) -> None:
        super().__init__()

        self._imu = imu
        self._filter = MahonyFilter(kp=filter_kp, ki=filter_ki)

        self._last_timestamp = Timer.getTimestamp()

        self._zeroing_samples = zeroing_samples
        self._zeroing_active = False
        self._zeroing_idx = 0
        self._zeroing_sum_x = 0.0
        self._zeroing_sum_y = 0.0
        self._zeroing_sum_z = 0.0
        self._zeroed = False
        self._zero_count = 0

        self._last_gx = 0.0
        self._last_gy = 0.0
        self._last_gz = 0.0
        self._last_ax = 0.0
        self._last_ay = 0.0
        self._last_az = 0.0

    def start_zeroing(self) -> None:
        self._zeroing_active = True
        self._zeroed = False
        self._zeroing_idx = 0
        self._zeroing_sum_x = 0.0
        self._zeroing_sum_y = 0.0
        self._zeroing_sum_z = 0.0

    def is_zeroed(self) -> bool:
        return self._zeroed

    def is_zeroing(self) -> bool:
        return self._zeroing_active

    def get_zero_count(self) -> int:
        return self._zero_count

    def get_rotation(self) -> Rotation3d:
        return self._filter.get_rotation()

    def get_euler_angles(self) -> tuple[float, float, float]:
        return self._filter.get_euler_angles()

    def get_gyro_bias(self) -> tuple[float, float, float]:
        return self._filter.get_gyro_bias()

    def periodic(self) -> None:
        now = Timer.getTimestamp()
        dt = now - self._last_timestamp
        self._last_timestamp = now

        gx, gy, gz = self._imu.read_gyro_radps()
        ax, ay, az = self._imu.read_accel_mps2()
        self._last_gx, self._last_gy, self._last_gz = gx, gy, gz
        self._last_ax, self._last_ay, self._last_az = ax, ay, az
        self._filter.update(gx, gy, gz, ax, ay, az, dt)

        if self._zeroing_active:
            self._zeroing_sum_x += gx
            self._zeroing_sum_y += gy
            self._zeroing_sum_z += gz
            self._zeroing_idx += 1

            if self._zeroing_idx >= self._zeroing_samples:
                n = float(self._zeroing_samples)
                self._filter.set_gyro_bias(
                    self._zeroing_sum_x / n,
                    self._zeroing_sum_y / n,
                    self._zeroing_sum_z / n,
                )
                self._filter.reset()
                self._zeroed = True
                self._zeroing_active = False
                self._zero_count += 1

        sd = wpilib.SmartDashboard

        sd.putNumber("imu/accel_x", self._last_ax)
        sd.putNumber("imu/accel_y", self._last_ay)
        sd.putNumber("imu/accel_z", self._last_az)
        sd.putNumber("imu/gyro_x", self._last_gx)
        sd.putNumber("imu/gyro_y", self._last_gy)
        sd.putNumber("imu/gyro_z", self._last_gz)

        roll, pitch, yaw = self._filter.get_euler_angles()
        sd.putNumberArray("imu/filtered_rpy", [roll, pitch, yaw])

        bx, by, bz = self._filter.get_gyro_bias()
        sd.putNumberArray("imu/gyro_bias", [bx, by, bz])

        sd.putBoolean("imu/is_zeroed", self._zeroed)
        sd.putBoolean("imu/is_zeroing", self._zeroing_active)
        sd.putNumber("imu/zero_count", self._zero_count)
