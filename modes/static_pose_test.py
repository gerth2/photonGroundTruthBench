"""Autonomous routine that commands the positioner through a list of
static poses, samples IMU and PhotonVision at each, and writes RMS
error results to CSV."""

from __future__ import annotations

import csv
import math
import os
import statistics
import time
from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING

import wpilib
from wpilib import PeriodicOpMode, SmartDashboard, Timer
from wpimath import Pose3d, Rotation3d, Transform3d

from config.bench_config import BenchConfig

if TYPE_CHECKING:
    from robot import Robot

from robot import autonomous  # noqa: E402

_STABILITY_DURATION_S = 1.0
_STABILITY_RAD = math.radians(0.5)


class Phase:
    """String sentinel values for the static-pose state machine."""

    ZEROING = "ZEROING"
    MOVING = "MOVING"
    PROFILE_WAIT = "PROFILE_WAIT"
    STABILIZING = "STABILIZING"
    SAMPLING = "SAMPLING"
    RECORD = "RECORD"
    DONE = "DONE"


@dataclass
class WindowResult:
    """Accumulated sample data and computed RMS errors for one static-pose
    measurement window."""

    pose_idx: int
    expected_tags: tuple[str, ...]
    cmd_r: float
    cmd_p: float
    cmd_y: float
    imu_count: int
    pv_count: int
    imu_mean: tuple[float, float, float]
    imu_std: tuple[float, float, float]
    rms_errors: dict[str, float]


@autonomous(name="Static Pose Test", group="Bench")
class StaticPoseMode(PeriodicOpMode):
    """PeriodicOpMode that iterates a pose list, waits for stability at
    each, samples IMU and PhotonVision simultaneously, then writes
    comparison results."""

    def __init__(self, robot: Robot) -> None:
        """Read validation config, build the pose list with expected tag IDs,
        and initialise all sampling buffers."""
        super().__init__()
        self._robot = robot

        vc = BenchConfig.validation
        cad = BenchConfig.cad
        self._camera_translation = cad.camera_pose_in_bench.translation()
        self._storage_path = vc.test_results_path
        self._settle_duration_s = vc.static_settle_duration_s
        self._sample_duration_s = vc.static_sample_duration_s

        self._poses: list[tuple[float, float, float]] = [
            (math.radians(p), math.radians(y), math.radians(r))
            for p, y, r in vc.static_pose_deg
        ]
        self._expected_tags: list[tuple[str, ...]] = []
        for i in range(len(self._poses)):
            tags = (
                vc.static_expected_tags[i] if i < len(vc.static_expected_tags) else ()
            )
            self._expected_tags.append(tags)

        self._phase: str = Phase.ZEROING
        self._pose_index = 0
        self._phase_start_time = 0.0
        self._done = False
        self._completed_at: str | None = None

        self._stability_buf: deque[tuple[float, float, float, float]] = deque()
        self._window_imu: list[tuple[float, float, float]] = []
        self._window_pv: list[tuple[int, Pose3d]] = []
        self._results: list[WindowResult] = []

        self._gt_pose: Pose3d | None = None
        self._pv_pose: Pose3d | None = None

    def start(self) -> None:
        """Reset all state and begin IMU zeroing."""
        self._phase = Phase.ZEROING
        self._pose_index = 0
        self._phase_start_time = Timer.getTimestamp()
        self._done = False
        self._completed_at = None
        self._stability_buf.clear()
        self._window_imu.clear()
        self._window_pv.clear()
        self._results.clear()
        self._robot.sensors.start_zeroing()

    def periodic(self) -> None:
        """Advance the static-pose state machine each cycle and publish status
        to NetworkTables."""
        if self._phase is Phase.ZEROING:
            self._zeroing()
            return
        if self._pose_index >= len(self._poses):
            if self._phase is not Phase.DONE:
                self._flush_csv()
                p0, y0, r0 = self._poses[0]
                self._robot.positioner.set_goal_rad(p0, y0, r0)
                self._done = True
                self._completed_at = time.strftime("%Y-%m-%d %H:%M:%S")
            self._phase = Phase.DONE
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
        """Clear the NetworkTables running flag and set the completed flag."""
        SmartDashboard.putBoolean("static_pose/running", False)
        SmartDashboard.putBoolean("static_pose/completed", self._done)

    def _zeroing(self) -> None:
        """Transition to MOVING once the IMU bias is estimated."""
        if self._robot.sensors.is_zeroed():
            self._phase = Phase.MOVING

    def _moving(self) -> None:
        """Command the current pose goal and advance to PROFILE_WAIT."""
        pitch, yaw, roll = self._poses[self._pose_index]
        self._robot.positioner.set_goal_rad(pitch, yaw, roll)
        self._phase = Phase.PROFILE_WAIT

    def _profile_wait(self) -> None:
        """Transition to STABILIZING once the positioner's motion profile completes."""
        if self._robot.positioner.profile_finished():
            self._stability_buf.clear()
            self._phase_start_time = Timer.getTimestamp()
            self._phase = Phase.STABILIZING

    def _stabilizing(self) -> None:
        """Monitor IMU attitude over a sliding window and transition to SAMPLING
        when stable or after a timeout."""
        now = Timer.getTimestamp()
        r, p, y = self._robot.sensors.get_euler_angles()
        self._stability_buf.append((r, p, y, now))

        while self._stability_buf and self._stability_buf[0][3] < now - _STABILITY_DURATION_S:
            self._stability_buf.popleft()

        elapsed = now - self._phase_start_time
        if elapsed < _STABILITY_DURATION_S:
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

        if elapsed >= self._settle_duration_s:
            SmartDashboard.putString(
                "static_pose/warning",
                f"pose {self._pose_index}: stability timeout after {self._settle_duration_s} s",
            )
            self._start_sampling()

    def _start_sampling(self) -> None:
        """Clear sample buffers and start the sampling window."""
        self._window_imu.clear()
        self._window_pv.clear()
        self._phase_start_time = Timer.getTimestamp()
        self._phase = Phase.SAMPLING

    def _sampling(self) -> None:
        """Collect IMU and PhotonVision samples for the configured duration,
        then compute RMS errors.

        For each cycle, accumulates IMU euler angles and per-tag camera-pose
        estimates from PhotonVision.  At window end, computes the ground-truth
        camera pose (IMU mean + CAD translation) and RMS error vs every tag-
        based PV camera pose recorded in the window.
        """
        imu_r, imu_p, imu_y = self._robot.sensors.get_euler_angles()
        self._window_imu.append((imu_r, imu_p, imu_y))

        tag_cam_poses = self._robot.vision.get_tag_camera_poses()
        for tag_id, cam_pose in tag_cam_poses.items():
            self._window_pv.append((tag_id, cam_pose))

        if Timer.getTimestamp() - self._phase_start_time < self._sample_duration_s:
            return

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

        gt_pose = Pose3d(
            self._camera_translation,
            Rotation3d(imu_mean[0], imu_mean[1], imu_mean[2]),
        )

        err_xs: list[float] = []
        err_ys: list[float] = []
        err_zs: list[float] = []
        err_rs: list[float] = []
        err_ps: list[float] = []
        err_ys_list: list[float] = []

        for _tag_id, pv_pose in self._window_pv:
            err_tf = Transform3d(gt_pose, pv_pose)
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
            """Return the root-mean-square of the given values."""
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
        """Increment the pose index and reset buffers for the next pose."""
        self._pose_index += 1
        self._stability_buf.clear()
        self._window_imu.clear()
        self._window_pv.clear()
        self._phase = Phase.MOVING

    def _publish_nt(self) -> None:
        """Publish current progress and completion status to SmartDashboard."""
        sd = SmartDashboard
        sd.putBoolean("static_pose/running", True)
        sd.putNumber("static_pose/pose_index", self._pose_index)
        sd.putNumber("static_pose/total_poses", len(self._poses))
        sd.putBoolean("static_pose/completed", self._done)
        if self._completed_at is not None:
            sd.putString("static_pose/completed_at", self._completed_at)

    def _flush_csv(self) -> None:
        """Write all accumulated WindowResult records to a CSV file on disk."""
        storage = (
            os.path.join(os.getcwd(), "test_results")
            if wpilib.RobotBase.isSimulation()
            else self._storage_path
        )
        os.makedirs(storage, exist_ok=True)
        path = os.path.join(storage, "static_pose_results.csv")

        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(
                [
                    "pose_idx",
                    "expected_tags",
                    "cmd_roll (rad)",
                    "cmd_pitch (rad)",
                    "cmd_yaw (rad)",
                    "imu_count",
                    "pv_tag_detections",
                    "imu_mean_roll (rad)",
                    "imu_mean_pitch (rad)",
                    "imu_mean_yaw (rad)",
                    "imu_std_roll (rad)",
                    "imu_std_pitch (rad)",
                    "imu_std_yaw (rad)",
                    "rms_dx (m)",
                    "rms_dy (m)",
                    "rms_dz (m)",
                    "rms_droll (rad)",
                    "rms_dpitch (rad)",
                    "rms_dyaw (rad)",
                ]
            )
            for r in self._results:
                w.writerow(
                    [
                        r.pose_idx,
                        ",".join(str(t) for t in r.expected_tags),
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
