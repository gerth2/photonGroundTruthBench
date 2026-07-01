#!/usr/bin/env python3

"""Entry-point module for the ground-truth calibration bench robot application.

Owns hardware subsystem instantiation and OpMode registration via the
OpModeRobot framework.  Delegates per-mode logic to subclasses in ``modes/``.
"""

import math

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


_registry: list[tuple[type, RobotMode, str, str, str]] = []


def teleop(
    name: str = "",
    group: str = "",
    description: str = "",
) -> Callable[[type], type]:
    """Register a class as a TELEOPERATED OpMode.

    Parameters
    ----------
    name : str
        Display name (defaults to the class name).
    group : str
        Group for DS organisation.
    description : str
        Human-readable description.

    Returns
    -------
    Callable[[type], type]
        The decorator that appends to the global registry.
    """
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
    """Register a class as an AUTONOMOUS OpMode.

    Parameters
    ----------
    name : str
        Display name (defaults to the class name).
    group : str
        Group for DS organisation.
    description : str
        Human-readable description.

    Returns
    -------
    Callable[[type], type]
        The decorator that appends to the global registry.
    """
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
    """Register a class as a UTILITY OpMode.

    Parameters
    ----------
    name : str
        Display name (defaults to the class name).
    group : str
        Group for DS organisation.
    description : str
        Human-readable description.

    Returns
    -------
    Callable[[type], type]
        The decorator that appends to the global registry.
    """
    def deco(cls: type) -> type:
        _registry.append(
            (cls, RobotMode.UTILITY, name or cls.__name__, group, description)
        )
        return cls

    return deco


class Robot(OpModeRobot):
    """Top-level robot class for the ground-truth calibration bench.

    Creates and wires all hardware subsystems (sensors, positioner, vision),
    registers OpModes from the global decorator registry, and publishes static
    calibration-target poses to SmartDashboard.
    """

    def __init__(self) -> None:
        """Initialise subsystems and register all decorated OpModes."""
        super().__init__()  # type: ignore[no-untyped-call]

        for cls_, mode, name, group, desc in _registry:
            self.addOpMode(cls_, mode, name, group, desc)
        self.publishOpModes()

        cfg = BenchConfig
        pc = cfg.positioner
        ic = cfg.imu
        pid = cfg.pid
        prc = cfg.profile

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

        cal_map = CalibrationMap

        self.positioner = CameraPositioner(
            pitch_channel=pc.pitch_servo_channel,
            yaw_channel=pc.yaw_servo_channel,
            roll_channel=pc.roll_servo_channel,
            pitch_center=pc.pitch_center,
            pitch_min=pc.pitch_min,
            pitch_max=pc.pitch_max,
            yaw_center=pc.yaw_center,
            yaw_min=pc.yaw_min,
            yaw_max=pc.yaw_max,
            roll_center=pc.roll_center,
            roll_min=pc.roll_min,
            roll_max=pc.roll_max,
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
            pitch_max_velocity=math.radians(prc.pitch_max_velocity_degps),
            pitch_max_acceleration=math.radians(prc.pitch_max_acceleration_degps2),
            yaw_max_velocity=math.radians(prc.yaw_max_velocity_degps),
            yaw_max_acceleration=math.radians(prc.yaw_max_acceleration_degps2),
            roll_max_velocity=math.radians(prc.roll_max_velocity_degps),
            roll_max_acceleration=math.radians(prc.roll_max_acceleration_degps2),
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

        self._publish_static_targets(cfg)

    def robotPeriodic(self) -> None:
        """Advance all hardware subsystems by one 20 ms cycle."""
        self.sensors.periodic()
        self.positioner.periodic()
        self.vision.periodic()

    @staticmethod
    def _publish_static_targets(cfg: type[BenchConfig]) -> None:
        """Write known calibration-target poses to SmartDashboard.

        Parameters
        ----------
        cfg : type[BenchConfig]
            Configuration class containing CAD pose constants.
        """
        sd = wpilib.SmartDashboard

        def _pub(label: str, pose: Pose3d) -> None:
            """Write a single target's translation and rotation to SmartDashboard."""
            t = pose.translation()
            r = pose.rotation()
            sd.putNumberArray(
                f"targets/{label}",
                [t.X(), t.Y(), t.Z(), r.X(), r.Y(), r.Z()],
            )

        _pub("left_tag", cfg.cad.left_tag_pose)
        _pub("right_tag", cfg.cad.right_tag_pose)
        _pub("charuco_board", cfg.cad.charuco_board_pose)


import modes.calibrate_servos  # noqa: E402, F401
import modes.dynamic_sweep_test  # noqa: E402, F401
import modes.manual_control  # noqa: E402, F401
import modes.static_pose_test  # noqa: E402, F401
import modes.zero_imu  # noqa: E402, F401

if __name__ == "__main__":
    OpModeRobot.main(Robot)
