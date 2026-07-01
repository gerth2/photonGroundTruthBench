"""Physical constants for the ground-truth calibration bench.

Update CADConstants when the CAD model changes.  All other config classes
define sensible defaults for the MG90S-servo / MPU6050 build described in
AGENTS.md.
"""

import math

from wpimath import Pose3d, Rotation3d, Transform3d, Translation3d


class PositionerConfig:
    """PWM channels and N11 soft-limit configuration per axis.

    ``center`` is the N11 value for mechanical centre (0 N11 when no
    calibration map is present).  ``min`` / ``max`` define the mechanism-
    safe soft limits within the servo's full [-1, 1] N11 range.
    """

    pitch_servo_channel: int = 0
    yaw_servo_channel: int = 1
    roll_servo_channel: int = 2

    # Servo output in -1..1 space.
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
    """I²C bus layout and MPU6050 initial settings."""

    i2c_port: int = 1  # SystemCore I²C port
    i2c_address: int = 0x68  # MPU6050 default (AD0 low)

    gyro_full_scale_dps: int = 250
    accel_full_scale_g: int = 2

    zeroing_samples: int = 100  # frames averaged for gyro bias
    zeroing_interval_ms: int = 10

    filter_kp: float = 0.5  # Mahony proportional gain
    filter_ki: float = 0.001  # Mahony integral gain (gyro bias tracking)


class CADConstants:
    """Bench-frame poses that must be updated from CAD revisions."""

    # Transform3d from the camera reference point to the IMU chip centre.
    camera_to_imu = Transform3d(
        Translation3d(0.02, 0.0, 0.01),
        Rotation3d(0.0, 0.0, 0.0),
    )

    # Distance from camera reference to the lens focal point (along +X).
    camera_focal_point_offset = Translation3d(0.01, 0.0, 0.0)

    # 6-inch AprilTag (family 36h11).
    tag_size_m: float = 0.1524

    apriltag6_pose = Pose3d(
        Translation3d(0.5, -0.3, 0.0),
        Rotation3d(0.0, 0.0, math.pi),
    )

    apriltag7_pose = Pose3d(
        Translation3d(0.5, 0.3, 0.0),
        Rotation3d(0.0, 0.0, math.pi),
    )

    charuco_board_pose = Pose3d(
        Translation3d(0.5, 0.0, -0.02),
        Rotation3d(0.0, 0.0, math.pi),
    )

    # Fixed camera translation in bench frame (the positioner only rotates
    # the camera). Origin is halfway between targets and camera; targets
    # at x ≈ 0.5 m, so camera is at x ≈ –0.5 m.
    camera_pose_in_bench = Pose3d(
        Translation3d(-0.5, 0.0, 0.0),
        Rotation3d(0.0, 0.0, 0.0),
    )


class PIDConfig:
    """Closed-loop servo correction gains.

    The integral (I) term is expected to dominate because the 3D-printed
    mechanism has enough slop that a pure P controller leaves significant
    steady-state error.
    """

    pitch_kp: float = 0.05
    pitch_ki: float = 0.3
    yaw_kp: float = 0.05
    yaw_ki: float = 0.3
    roll_kp: float = 0.05
    roll_ki: float = 0.3

    integral_limit: float = 0.5  # anti-windup clamp in servo units
    position_tolerance_rad: float = 0.05  # ~3°, per-axis convergence check


class ProfileConfig:
    """1-D trapezoidal motion-profile limits for each servo axis.

    Velocity in deg/s, acceleration in deg/s².  These smooth step inputs
    to avoid jerking the 3D-printed mechanism.
    """

    pitch_max_velocity_degps: float = 10.0
    pitch_max_acceleration_degps2: float = 60.0
    yaw_max_velocity_degps: float = 10.0
    yaw_max_acceleration_degps2: float = 60.0
    roll_max_velocity_degps: float = 10.0
    roll_max_acceleration_degps2: float = 60.0


class CalibrationConfig:
    """Sweep parameters for the open-loop CalibrateServos command.

    Durations in seconds — compared against ``Timer.getTimestamp()``
    so timing is independent of actual loop rate.
    """

    num_points: int = 100
    slew_duration_s: float = 1.0  # seconds to linearly ramp n11 between points
    settle_duration_s: float = 3.0  # seconds of mechanical settling after slew
    sample_duration_s: float = 1.0  # seconds to accumulate IMU readings
    storage_path: str = "/home/lvuser/calibration_data"


class ValidationConfig:
    """Pose sequences and sweep parameters for the validation opmodes."""

    # ── Static pose test ──────────────────────────────────────────────
    # Each entry is (pitch_deg, yaw_deg, roll_deg).  Omitted poses will
    # be filled at runtime by sweeping ±range in pitch & yaw.
    static_pose_deg: list[tuple[float, float, float]] = [
        (0.0, 0.0, 0.0),
        (0.0, 15.0, 0.0),
        (5.0, -10.0, 0.0),
        (10.0, 0.0, 3.0),
        (-10.0, 0.0, 15.0),
        (5.0, 10.0, 0.0),
        (-5.0, -10.0, 0.0),
    ]
    # Which AprilTag IDs are expected to be in view at each corresponding
    # pose above.  Empty tuple means "no tags expected, PV may be absent".
    static_expected_tags: list[tuple[int, ...]] = [
        (6, 7),
        (7,),
        (6,),
        (6, 7),
        (6, 7),
        (7,),
        (6,),
    ]
    static_settle_duration_s: float = 3.0
    static_sample_duration_s: float = 1.0

    # ── Output storage ────────────────────────────────────────────────
    # Test results (AUTONOMOUS modes — static pose, dynamic sweep).
    test_results_path: str = "/home/lvuser/test_results"

    # ── Dynamic sweep test ───────────────────────────────────────────
    # For each axis run a sinusoid at each listed peak velocity (deg/s).
    sweep_velocity_steps_degps: list[float] = [5.0, 10.0, 20.0, 40.0]
    sweep_amplitude_deg: float = 20.0
    sweep_duration_s_per_step: float = 6.0


class BenchConfig:
    """Top-level namespace grouping all sub-configs."""

    positioner: type[PositionerConfig] = PositionerConfig
    imu: type[IMUConfig] = IMUConfig
    cad: type[CADConstants] = CADConstants
    pid: type[PIDConfig] = PIDConfig
    profile: type[ProfileConfig] = ProfileConfig
    calibration: type[CalibrationConfig] = CalibrationConfig
    validation: type[ValidationConfig] = ValidationConfig

    # Tolerances used by error-comparison code.
    ground_truth_tolerance_m: float = 0.001
    ground_truth_tolerance_deg: float = 0.1
