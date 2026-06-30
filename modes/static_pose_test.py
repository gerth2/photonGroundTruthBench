"""Iterate a set of pre-planned camera poses where targets are visible.

At each pose:
  1. Command the positioner to the pose.
  2. Wait for the trapezoid profile to finish (profiled == desired).
  3. Wait for the IMU to stabilise (±0.5° per axis for 1 s).
  4. Collect a 1 s window of IMU + PhotonVision data (50 cycles).
  5. Compute:
       - IMU mean roll/pitch/yaw (ground-truth camera rotation)
       - IMU roll/pitch/yaw std dev
       - IMU sample count (always 50)
       - PV valid-sample count
       - Per-sample error: GT⁻¹ * PV (relative transform in camera frame)
       - RMS of translation x/y/z and rotation roll/pitch/yaw over window
  6. Write row to CSV.

Timeout after 5 s waiting for stability so a stuck pose doesn't hang.
"""

from __future__ import annotations

import csv
import math
import os
import statistics
from collections import deque
from dataclasses import dataclass
from enum import Enum, auto
from typing import TYPE_CHECKING

from wpilib import PeriodicOpMode, SmartDashboard
from wpimath import Pose3d, Rotation3d, Transform3d

from config.bench_config import BenchConfig

if TYPE_CHECKING:
    from robot import Robot

from robot import autonomous  # noqa: E402

_RAD_PER_DEG = math.pi / 180.0
_STABILITY_DEG = 0.5  # max per-axis range for "stable"
_STABILITY_RAD = _STABILITY_DEG * _RAD_PER_DEG
_STABILITY_CYCLES = 50  # 1 s at 20 ms
_STABILITY_TIMEOUT = 250  # 5 s
_SAMPLE_WINDOW = 50  # 1 s at 20 ms


class Phase(Enum):
    ZEROING = auto()
    MOVING = auto()
    PROFILE_WAIT = auto()
    STABILIZING = auto()
    SAMPLING = auto()
    RECORD = auto()
    DONE = auto()


@dataclass
class WindowResult:
    pose_idx: int
    expected_tags: tuple[int, ...]
    cmd_r: float
    cmd_p: float
    cmd_y: float
    imu_count: int
    pv_count: int
    imu_mean: tuple[float, float, float]
    imu_std: tuple[float, float, float]
    rms_errors: dict[str, float]  # "dx","dy","dz","droll","dpitch","dyaw"


@autonomous(name="Static Pose Test", group="Validation")
class StaticPoseTest(PeriodicOpMode):
    """Validate PV accuracy at a set of static poses."""

    def __init__(self, robot: Robot) -> None:
        super().__init__()
        self._robot = robot

        vc = BenchConfig.validation
        self._storage_path = BenchConfig.calibration.storage_path
        self._camera_translation = BenchConfig.cad.camera_pose_in_bench.translation()

        # Build pose list from config.
        self._poses: list[tuple[float, float, float]] = []
        self._expected_tags: list[tuple[int, ...]] = []
        for i, (pitch_deg, yaw_deg, roll_deg) in enumerate(vc.static_pose_deg):
            self._poses.append(
                (
                    math.radians(pitch_deg),
                    math.radians(yaw_deg),
                    math.radians(roll_deg),
                )
            )
            tags = (
                vc.static_expected_tags[i] if i < len(vc.static_expected_tags) else ()
            )
            self._expected_tags.append(tags)

        self._phase: Phase = Phase.ZEROING
        self._pose_index = 0
        self._cycle_count = 0
        self._done = False

        # Stability buffer (deque of (r, p, y) tuples).
        self._stability_buf: deque[tuple[float, float, float]] = deque(
            maxlen=_STABILITY_CYCLES
        )

        # Sampling window buffers.
        self._window_imu: list[tuple[float, float, float]] = []
        self._window_pv: list[Pose3d] = []

        # Accumulated results.
        self._results: list[WindowResult] = []

    # ── Lifecycle ──────────────────────────────────────────────────────

    def start(self) -> None:
        self._phase = Phase.ZEROING
        self._pose_index = 0
        self._cycle_count = 0
        self._done = False
        self._stability_buf.clear()
        self._window_imu.clear()
        self._window_pv.clear()
        self._results.clear()
        self._robot.sensors.start_zeroing()

    def periodic(self) -> None:
        if self._phase is Phase.ZEROING:
            self._zeroing()
            return
        if self._pose_index >= len(self._poses):
            self._phase = Phase.DONE
            self._done = True
            return
        if self._phase is Phase.MOVING:
            self._moving()
        elif self._phase is Phase.PROFILE_WAIT:
            self._profile_wait()
        elif self._phase is Phase.STABILIZING:
            self._stabilizing()
        elif self._phase is Phase.SAMPLING:
            self._sampling()
        elif self._phase is Phase.RECORD:
            self._record()

        self._publish_nt()

    def end(self) -> None:
        if self._results:
            self._flush_csv()
        SmartDashboard.putBoolean("static_pose/running", False)
        SmartDashboard.putBoolean("static_pose/completed", self._done)

    # ── Phase handlers ─────────────────────────────────────────────────

    def _zeroing(self) -> None:
        if self._robot.sensors.is_zeroed():
            self._phase = Phase.MOVING

    def _moving(self) -> None:
        pitch, yaw, roll = self._poses[self._pose_index]
        self._robot.positioner.set_goal_rad(pitch, yaw, roll)
        self._phase = Phase.PROFILE_WAIT

    def _profile_wait(self) -> None:
        if self._robot.positioner.profile_finished():
            self._stability_buf.clear()
            self._cycle_count = 0
            self._phase = Phase.STABILIZING

    def _stabilizing(self) -> None:
        r, p, y = self._robot.sensors.get_euler_angles()
        self._stability_buf.append((r, p, y))
        self._cycle_count += 1

        if len(self._stability_buf) < _STABILITY_CYCLES:
            return

        rolls = [s[0] for s in self._stability_buf]
        pitches = [s[1] for s in self._stability_buf]
        yaws = [s[2] for s in self._stability_buf]

        if (
            max(rolls) - min(rolls) < _STABILITY_RAD
            and max(pitches) - min(pitches) < _STABILITY_RAD
            and max(yaws) - min(yaws) < _STABILITY_RAD
        ):
            self._start_sampling()
            return

        if self._cycle_count >= _STABILITY_TIMEOUT:
            SmartDashboard.putString(
                "static_pose/warning",
                f"pose {self._pose_index}: stability timeout after 5 s",
            )
            self._start_sampling()

    def _start_sampling(self) -> None:
        self._window_imu.clear()
        self._window_pv.clear()
        self._cycle_count = 0
        self._phase = Phase.SAMPLING

    def _sampling(self) -> None:
        imu_r, imu_p, imu_y = self._robot.sensors.get_euler_angles()
        self._window_imu.append((imu_r, imu_p, imu_y))

        pv_pose = self._robot.vision.get_latest_pose()
        if pv_pose is not None:
            self._window_pv.append(pv_pose.estimatedPose)

        self._cycle_count += 1
        if self._cycle_count < _SAMPLE_WINDOW:
            return

        # Compute IMU statistics over the window.
        rolls = [s[0] for s in self._window_imu]
        pitches = [s[1] for s in self._window_imu]
        yaws = [s[2] for s in self._window_imu]

        imu_mean = (
            statistics.mean(rolls),
            statistics.mean(pitches),
            statistics.mean(yaws),
        )
        imu_std = (
            statistics.stdev(rolls) if len(rolls) > 1 else 0.0,
            statistics.stdev(pitches) if len(pitches) > 1 else 0.0,
            statistics.stdev(yaws) if len(yaws) > 1 else 0.0,
        )

        # Ground-truth camera pose: fixed translation (from CAD) + IMU mean
        # rotation.
        gt_pose = Pose3d(
            self._camera_translation,
            Rotation3d(imu_mean[0], imu_mean[1], imu_mean[2]),
        )

        # Per-sample error: GT⁻¹ * PV (relative transform in camera frame).
        err_xs: list[float] = []
        err_ys: list[float] = []
        err_zs: list[float] = []
        err_rs: list[float] = []
        err_ps: list[float] = []
        err_ys_list: list[float] = []

        for pv in self._window_pv:
            err_tf = Transform3d(gt_pose, pv)
            t = err_tf.translation()
            r = err_tf.rotation()
            err_xs.append(t.X())
            err_ys.append(t.Y())
            err_zs.append(t.Z())
            err_rs.append(r.X())
            err_ps.append(r.Y())
            err_ys_list.append(r.Z())

        cmd_p, cmd_y, cmd_r = self._poses[self._pose_index]

        def _rms(vals: list[float]) -> float:
            if not vals:
                return 0.0
            return math.sqrt(sum(v * v for v in vals) / len(vals))

        result = WindowResult(
            pose_idx=self._pose_index,
            expected_tags=self._expected_tags[self._pose_index],
            cmd_r=cmd_r,
            cmd_p=cmd_p,
            cmd_y=cmd_y,
            imu_count=len(self._window_imu),
            pv_count=len(self._window_pv),
            imu_mean=imu_mean,
            imu_std=imu_std,
            rms_errors={
                "dx": _rms(err_xs),
                "dy": _rms(err_ys),
                "dz": _rms(err_zs),
                "droll": _rms(err_rs),
                "dpitch": _rms(err_ps),
                "dyaw": _rms(err_ys_list),
            },
        )

        self._results.append(result)
        self._phase = Phase.RECORD

    def _record(self) -> None:
        self._pose_index += 1
        self._stability_buf.clear()
        self._window_imu.clear()
        self._window_pv.clear()
        self._cycle_count = 0
        self._phase = Phase.MOVING

    # ── NT telemetry ───────────────────────────────────────────────────

    def _publish_nt(self) -> None:
        sd = SmartDashboard
        sd.putBoolean("static_pose/running", True)
        sd.putNumber("static_pose/pose_index", self._pose_index)
        sd.putNumber("static_pose/total_poses", len(self._poses))
        sd.putBoolean("static_pose/completed", self._done)

    # ── CSV I/O ────────────────────────────────────────────────────────

    def _flush_csv(self) -> None:
        try:
            os.makedirs(self._storage_path, exist_ok=True)
        except OSError:
            self._storage_path = os.path.join(os.getcwd(), "calibration_data")
            os.makedirs(self._storage_path, exist_ok=True)
        path = os.path.join(self._storage_path, "static_pose_results.csv")

        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(
                [
                    "pose_idx",
                    "cmd_roll",
                    "cmd_pitch",
                    "cmd_yaw",
                    "imu_count",
                    "pv_count",
                    "imu_mean_roll",
                    "imu_mean_pitch",
                    "imu_mean_yaw",
                    "imu_std_roll",
                    "imu_std_pitch",
                    "imu_std_yaw",
                    "rms_dx",
                    "rms_dy",
                    "rms_dz",
                    "rms_droll",
                    "rms_dpitch",
                    "rms_dyaw",
                ]
            )
            for r in self._results:
                w.writerow(
                    [
                        r.pose_idx,
                        f"{r.cmd_r:.6f}",
                        f"{r.cmd_p:.6f}",
                        f"{r.cmd_y:.6f}",
                        r.imu_count,
                        r.pv_count,
                        f"{r.imu_mean[0]:.6f}",
                        f"{r.imu_mean[1]:.6f}",
                        f"{r.imu_mean[2]:.6f}",
                        f"{r.imu_std[0]:.6f}",
                        f"{r.imu_std[1]:.6f}",
                        f"{r.imu_std[2]:.6f}",
                        f"{r.rms_errors['dx']:.6f}",
                        f"{r.rms_errors['dy']:.6f}",
                        f"{r.rms_errors['dz']:.6f}",
                        f"{r.rms_errors['droll']:.6f}",
                        f"{r.rms_errors['dpitch']:.6f}",
                        f"{r.rms_errors['dyaw']:.6f}",
                    ]
                )

        SmartDashboard.putString("static_pose/csv_path", path)
