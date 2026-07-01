"""Data types for ground-truth vs PhotonVision comparison."""

import dataclasses
from wpimath import Pose3d


@dataclasses.dataclass
class PoseMeasurement:
    """A timestamped PhotonVision pose estimate."""

    timestamp_s: float
    pose: Pose3d


@dataclasses.dataclass
class GroundTruthSample:
    """A timestamped ground-truth camera pose and associated sensor readings."""

    timestamp_s: float
    camera_pose: Pose3d
    sensor_readings: dict[str, float]


@dataclasses.dataclass
class ComparisonRecord:
    """The result of comparing a ground-truth sample with its paired vision estimate, including the computed error."""

    timestamp_s: float
    ground_truth: GroundTruthSample
    vision_estimate: PoseMeasurement | None
    error: Pose3d | None
