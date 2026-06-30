"""Data types for ground-truth vs PhotonVision comparison."""

import dataclasses
from wpimath import Pose3d


@dataclasses.dataclass
class PoseMeasurement:
    timestamp_s: float
    pose: Pose3d


@dataclasses.dataclass
class GroundTruthSample:
    timestamp_s: float
    camera_pose: Pose3d
    sensor_readings: dict[str, float]


@dataclasses.dataclass
class ComparisonRecord:
    timestamp_s: float
    ground_truth: GroundTruthSample
    vision_estimate: PoseMeasurement | None
    error: Pose3d | None
