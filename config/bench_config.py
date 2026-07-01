"""Physical constants, PID gains, profile limits, and calibration parameters for the calibration bench.

Each inner dataclass groups a related set of constants.  BenchConfig provides
a single import point that aggregates all sub-configs for the Robot instance.
"""

import math

from wpimath import Pose3d, Rotation3d, Transform3d, Translation3d


class PositionerConfig:
    """PWM channels and soft limits for the three-axis servo positioner.

    Each axis (pitch, yaw, roll) defines a center, min, and max in N11
    (-1..1) servo space.  All commanded values are clamped to these bounds.
    """

    pitch_servo_channel: int = 0
    yaw_servo_channel: int = 1
    roll_servo_channel: int = 2

    pitch_center: float = 0.0
    pitch_min: float = -0.8
    pitch_max: float = 0.8

    yaw_center: float = 0.0
    yaw_min: float = -0.8
    yaw_max: float = 0.8

    roll_center: float = 0.0
    roll_min: float = -0.8
    roll_max: float = 0.8


class IMUConfig:
    """I²C bus parameters, sensor range settings, and Mahony filter gains for the MPU6050 IMU."""

    i2c_port: int = 1
    i2c_address: int = 0x68  # MPU6050 default when AD0 is low

    gyro_full_scale_dps: int = 250
    accel_full_scale_g: int = 2

    zeroing_samples: int = 100
    zeroing_interval_ms: int = 10

    filter_kp: float = 0.5
    filter_ki: float = 0.001


class CADConstants:
    """Fixed transforms and tag poses derived from the CAD model of the bench.

    All values are in the bench coordinate frame defined in AGENTS.md.
    Update when the CAD model changes.
    """

    camera_to_imu = Transform3d(
        Translation3d(0.02, 0.0, 0.01),
        Rotation3d(0.0, 0.0, 0.0),
    )

    camera_focal_point_offset = Translation3d(0.01, 0.0, 0.0)

    tag_size_m: float = 0.1524  # 6-inch AprilTag (family 36h11)

    left_tag_id: int = 6
    right_tag_id: int = 7

    left_tag_pose = Pose3d(
        Translation3d(0.5, -0.3, 0.0),
        Rotation3d(0.0, 0.0, math.pi),
    )

    right_tag_pose = Pose3d(
        Translation3d(0.5, 0.3, 0.0),
        Rotation3d(0.0, 0.0, math.pi),
    )

    charuco_board_pose = Pose3d(
        Translation3d(0.5, 0.0, -0.02),
        Rotation3d(0.0, 0.0, math.pi),
    )

    camera_pose_in_bench = Pose3d(
        Translation3d(-0.5, 0.0, 0.0),
        Rotation3d(0.0, 0.0, 0.0),
    )


class PIDConfig:
    """Per-axis proportional and integral gains for the closed-loop positioner feedback."""

    pitch_kp: float = 0.05
    pitch_ki: float = 0.3
    yaw_kp: float = 0.05
    yaw_ki: float = 0.3
    roll_kp: float = 0.05
    roll_ki: float = 0.3

    integral_limit: float = 0.5
    position_tolerance_rad: float = 0.05


class ProfileConfig:
    """Trapezoidal motion-profile velocity and acceleration limits per axis."""

    pitch_max_velocity_degps: float = 10.0
    pitch_max_acceleration_degps2: float = 60.0
    yaw_max_velocity_degps: float = 10.0
    yaw_max_acceleration_degps2: float = 60.0
    roll_max_velocity_degps: float = 10.0
    roll_max_acceleration_degps2: float = 60.0


class CalibrationConfig:
    """Parameters that control the servo-calibration sweep and data-storage path."""

    num_points: int = 100
    slew_duration_s: float = 1.0
    settle_duration_s: float = 3.0
    sample_duration_s: float = 1.0
    storage_path: str = "/home/lvuser/calibration_data"


class ValidationConfig:
    """Static-pose test points, expected tag IDs, and dynamic-sweep parameters for validation."""

    static_pose_deg: list[tuple[float, float, float]] = [
        (0.0, 0.0, 0.0),
        (0.0, 15.0, 0.0),
        (5.0, -10.0, 0.0),
        (10.0, 0.0, 3.0),
        (-10.0, 0.0, 15.0),
        (5.0, 10.0, 0.0),
        (-5.0, -10.0, 0.0),
    ]
    static_expected_tags: list[tuple[str, ...]] = [
        ("left", "right"),
        ("right",),
        ("left",),
        ("left", "right"),
        ("left", "right"),
        ("right",),
        ("left",),
    ]
    static_settle_duration_s: float = 3.0
    static_sample_duration_s: float = 1.0

    test_results_path: str = "/home/lvuser/test_results"

    sweep_velocity_steps_degps: list[float] = [5.0, 10.0, 20.0, 40.0]
    sweep_amplitude_deg: float = 20.0
    sweep_duration_s_per_step: float = 6.0


class BenchConfig:
    """Aggregate configuration that groups all sub-configs for injection into the Robot.

    Each attribute references the relevant inner class so callers can access
    e.g. ``BenchConfig.positioner.pitch_min``.
    """

    positioner: type[PositionerConfig] = PositionerConfig
    imu: type[IMUConfig] = IMUConfig
    cad: type[CADConstants] = CADConstants
    pid: type[PIDConfig] = PIDConfig
    profile: type[ProfileConfig] = ProfileConfig
    calibration: type[CalibrationConfig] = CalibrationConfig
    validation: type[ValidationConfig] = ValidationConfig

    ground_truth_tolerance_m: float = 0.001
    ground_truth_tolerance_deg: float = 0.1
