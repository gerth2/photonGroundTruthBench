"""MPU6050-based IMU ground-truth sensor with Mahony filter and on-line zeroing."""

import ntcore
import wpilib
from wpilib import Timer
from wpimath import Pose3d, Rotation3d, Translation3d

from utilities.imu_filter import MahonyFilter
from core.subsystem import Subsystem
from hardware.mpu6050 import MPU6050


class GroundTruthSensors(Subsystem):
    """Reads the MPU6050 each cycle and fuses gyro + accelerometer via Mahony filter.

    Lifecycle: call ``start_zeroing()`` to begin gyro bias estimation over
    ``zeroing_samples`` cycles.  Once ``is_zeroed()`` returns True the filter
    produces usable orientation estimates.  ``periodic()`` must be called each
    robot loop to keep the filter updated.
    """

    def __init__(
        self,
        imu: MPU6050,
        imu_translation: Translation3d = Translation3d(),
        filter_kp: float = 0.5,
        filter_ki: float = 0.0,
        zeroing_samples: int = 100,
    ) -> None:
        """Initialise the IMU driver, Mahony filter, and zeroing accumulators.

        Args:
            imu: Initialised MPU6050 hardware driver instance.
            imu_translation: Bench-frame translation of the IMU chip (from CAD).
            filter_kp: Mahony filter proportional gain.
            filter_ki: Mahony filter integral gain.
            zeroing_samples: Number of gyro samples averaged during zeroing.
        """
        super().__init__()

        self._imu = imu
        self._imu_translation = imu_translation
        self._filter = MahonyFilter(kp=filter_kp, ki=filter_ki)

        inst = ntcore.NetworkTableInstance.getDefault()
        self._pub_pose = inst.getStructTopic("sensors/imu_pose", Pose3d).publish()

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
        """Begin the gyro bias zeroing sequence.

        Resets accumulators and sets the internal zeroing state machine
        active.  ``periodic()`` will collect samples until ``zeroing_samples``
        is reached.
        """
        self._zeroing_active = True
        self._zeroed = False
        self._zeroing_idx = 0
        self._zeroing_sum_x = 0.0
        self._zeroing_sum_y = 0.0
        self._zeroing_sum_z = 0.0

    def is_zeroed(self) -> bool:
        """Return True if gyro bias estimation has completed at least once."""
        return self._zeroed

    def is_zeroing(self) -> bool:
        """Return True if the zeroing state machine is currently collecting samples."""
        return self._zeroing_active

    def get_zero_count(self) -> int:
        """Return the number of times zeroing has completed."""
        return self._zero_count

    def get_rotation(self) -> Rotation3d:
        """Return the latest fused orientation from the Mahony filter."""
        return self._filter.get_rotation()

    def get_euler_angles(self) -> tuple[float, float, float]:
        """Return the latest Euler angles (roll, pitch, yaw) in radians."""
        return self._filter.get_euler_angles()

    def get_gyro_bias(self) -> tuple[float, float, float]:
        """Return the current gyro bias estimate (x, y, z) in rad/s."""
        return self._filter.get_gyro_bias()

    def periodic(self) -> None:
        """Read sensor data, update the Mahony filter, and manage zeroing.

        Also pushes IMU telemetry to SmartDashboard each cycle.
        """
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

        imu_pose = Pose3d(self._imu_translation, self._filter.get_rotation())
        self._pub_pose.set(imu_pose)
