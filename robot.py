#!/usr/bin/env python3

"""Entry-point module for the ground-truth calibration bench robot application.

Owns hardware subsystem instantiation and OpMode registration via the
OpModeRobot framework.  Delegates per-mode logic to subclasses in ``modes/``.
"""

import math

from collections.abc import Callable

import ntcore
from hal import RobotMode
from wpilib import OpModeRobot
from wpimath import Pose3d

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
            imu_translation=cfg.cad.camera_to_imu.translation(),
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
            camera_translation=cfg.cad.camera_pose_in_bench.translation(),
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

        self.vision = VisionProcessor(
            camera_name="ground_truth_cam",
            bench_tag_poses={
                cfg.cad.left_tag_id: cfg.cad.left_tag_pose,
                cfg.cad.right_tag_id: cfg.cad.right_tag_pose,
            },
        )

        self._camera_translation = cfg.cad.camera_pose_in_bench.translation()

        self._init_publishers(cfg)

    def _init_publishers(self, cfg: type[BenchConfig]) -> None:
        """Create struct-topic publishers for fixed targets and camera poses."""
        inst = ntcore.NetworkTableInstance.getDefault()

        def _pose_pub(table: ntcore.NetworkTable, path: str) -> ntcore.StructPublisher:
            """Return a Pose3d struct publisher at ``table/path``."""
            topic = table.getStructTopic(path, Pose3d)
            return topic.publish()

        tgt = inst.getTable("targets")
        self._pub_target_left = _pose_pub(tgt, "left_tag")
        self._pub_target_right = _pose_pub(tgt, "right_tag")
        self._pub_charuco = _pose_pub(tgt, "charuco_board")

        self._pub_target_left.set(cfg.cad.left_tag_pose)
        self._pub_target_right.set(cfg.cad.right_tag_pose)
        self._pub_charuco.set(cfg.cad.charuco_board_pose)

        sr = inst.getTable("sensors")
        self._pub_gt_pose = _pose_pub(sr, "camera_pose")

    def robotPeriodic(self) -> None:
        """Advance all hardware subsystems and publish ground-truth camera pose.

        The ground-truth camera pose is computed each cycle from the fixed CAD
        camera translation and the latest IMU filter rotation.
        """
        self.sensors.periodic()
        self.positioner.periodic()
        self.vision.periodic()

        gt_pose = Pose3d(
            self._camera_translation,
            self.sensors.get_rotation(),
        )
        self._pub_gt_pose.set(gt_pose)


import modes.calibrate_servos  # noqa: E402, F401
import modes.dynamic_sweep_test  # noqa: E402, F401
import modes.manual_control  # noqa: E402, F401
import modes.static_pose_test  # noqa: E402, F401
import modes.zero_imu  # noqa: E402, F401

if __name__ == "__main__":
    OpModeRobot.main(Robot)
