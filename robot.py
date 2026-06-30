#!/usr/bin/env python3

"""Robot entry point using OpModeRobot framework.

Decorator helpers:
    @teleop(name, group)     — register a TELEOPERATED OpMode
    @autonomous(name, group) — register an AUTONOMOUS OpMode
    @utility(name, group)    — register a UTILITY OpMode

Usage in a mode file::

    from robot import utility
    from wpilib import PeriodicOpMode

    @utility("Calibrate Servos")
    class MyMode(PeriodicOpMode):
        def __init__(self, robot: Robot):
            self._robot = robot
        def start(self): ...
        def periodic(self): ...
"""

from collections.abc import Callable

import wpilib
from hal import RobotMode
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from robotpy_apriltag import AprilTagFieldLayout
from wpilib import OpModeRobot
from wpimath import Pose3d, Transform3d

from config.bench_config import BenchConfig
from config.servo_calibration_map import CalibrationMap
from hardware import CameraPositioner, GroundTruthSensors, MPU6050, VisionProcessor


# ── OpMode decorators ──────────────────────────────────────────────────

_registry: list[tuple[type, RobotMode, str, str, str]] = []


def teleop(
    name: str = "",
    group: str = "",
    description: str = "",
) -> Callable[[type], type]:
    """Decorator: register a TELEOPERATED OpMode."""

    def deco(cls: type) -> type:
        _registry.append(
            (cls, RobotMode.TELEOPERATED, name or cls.__name__, group, description)
        )
        return cls

    return deco


def autonomous(
    name: str = "",
    group: str = "",
    description: str = "",
) -> Callable[[type], type]:
    """Decorator: register an AUTONOMOUS OpMode."""

    def deco(cls: type) -> type:
        _registry.append(
            (cls, RobotMode.AUTONOMOUS, name or cls.__name__, group, description)
        )
        return cls

    return deco


def utility(
    name: str = "",
    group: str = "",
    description: str = "",
) -> Callable[[type], type]:
    """Decorator: register a UTILITY OpMode."""

    def deco(cls: type) -> type:
        _registry.append(
            (cls, RobotMode.UTILITY, name or cls.__name__, group, description)
        )
        return cls

    return deco


# ── Robot ──────────────────────────────────────────────────────────────


class Robot(OpModeRobot):
    """Test-bench robot.

    Owns all hardware subsystems and calls their ``periodic()`` from
    ``robotPeriodic()`` so sensors / actuators update regardless of which
    OpMode is active.
    """

    def __init__(self) -> None:
        super().__init__()  # type: ignore[no-untyped-call]
        cfg = BenchConfig
        pc = cfg.positioner
        ic = cfg.imu
        pid = cfg.pid

        imu = MPU6050(
            port=ic.i2c_port,
            address=ic.i2c_address,
            gyro_scale_dps=ic.gyro_full_scale_dps,
            accel_scale_g=ic.accel_full_scale_g,
        )

        self.sensors = GroundTruthSensors(
            imu=imu,
            filter_kp=ic.filter_kp,
            filter_ki=ic.filter_ki,
            zeroing_samples=ic.zeroing_samples,
        )

        cal_map = (
            CalibrationMap
            if hasattr(CalibrationMap, "INVERSE_COEFFS_SERVO_R")
            and CalibrationMap.INVERSE_COEFFS_SERVO_R
            else None
        )

        self.positioner = CameraPositioner(
            pitch_channel=pc.pitch_servo_channel,
            yaw_channel=pc.yaw_servo_channel,
            roll_channel=pc.roll_servo_channel,
            pitch_center=pc.pitch_center,
            pitch_min=pc.pitch_min,
            pitch_max=pc.pitch_max,
            pitch_range_deg=pc.pitch_range_deg,
            yaw_center=pc.yaw_center,
            yaw_min=pc.yaw_min,
            yaw_max=pc.yaw_max,
            yaw_range_deg=pc.yaw_range_deg,
            roll_center=pc.roll_center,
            roll_min=pc.roll_min,
            roll_max=pc.roll_max,
            roll_range_deg=pc.roll_range_deg,
            sensors=self.sensors,
            calibration_map=cal_map,
            pitch_kp=pid.pitch_kp,
            pitch_ki=pid.pitch_ki,
            yaw_kp=pid.yaw_kp,
            yaw_ki=pid.yaw_ki,
            roll_kp=pid.roll_kp,
            roll_ki=pid.roll_ki,
            integral_limit=pid.integral_limit,
            position_tolerance_rad=pid.position_tolerance_rad,
        )

        field_layout = AprilTagFieldLayout()
        self.vision = VisionProcessor(
            camera_name="ground_truth_cam",
            pose_estimator=PhotonPoseEstimator(
                fieldTags=field_layout,
                robotToCamera=Transform3d(),
            ),
            camera_to_robot=Transform3d(),
        )

        # Register all decorated OpModes (from the late-imported mode files).
        for cls_, mode, name, group, desc in _registry:
            self.addOpMode(cls_, mode, name, group, desc)
        self.publishOpModes()

        self._publish_static_targets(cfg)

    def robotPeriodic(self) -> None:
        self.sensors.periodic()
        self.positioner.periodic()
        self.vision.periodic()

    @staticmethod
    def _publish_static_targets(cfg: type[BenchConfig]) -> None:
        sd = wpilib.SmartDashboard

        def _pub(label: str, pose: Pose3d) -> None:
            t = pose.translation()
            r = pose.rotation()
            sd.putNumberArray(
                f"targets/{label}",
                [t.X(), t.Y(), t.Z(), r.X(), r.Y(), r.Z()],
            )

        _pub("apriltag_6", cfg.cad.apriltag6_pose)
        _pub("apriltag_7", cfg.cad.apriltag7_pose)
        _pub("charuco_board", cfg.cad.charuco_board_pose)


# ── Late imports: trigger OpMode decorator registration ─────────────

import modes.calibrate_servos  # noqa: E402, F401
import modes.dynamic_sweep_test  # noqa: E402, F401
import modes.manual_control  # noqa: E402, F401
import modes.static_pose_test  # noqa: E402, F401
import modes.zero_imu  # noqa: E402, F401

if __name__ == "__main__":
    OpModeRobot.main(Robot)
