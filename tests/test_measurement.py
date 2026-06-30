import math

from utilities.comparison import compute_error
from utilities.ground_truth import GroundTruthCalculator
from utilities.measurement import GroundTruthSample, PoseMeasurement
from wpimath import Pose3d, Translation3d, Rotation3d


def test_ground_truth_calculator_default() -> None:
    calc = GroundTruthCalculator((0.0, 0.0, 0.0))
    pose = calc.compute_camera_pose(0.0, 0.0, 0.0)
    t = pose.translation()
    assert abs(t.x) < 1e-9
    assert abs(t.y) < 1e-9
    assert abs(t.z) < 1e-9


def test_ground_truth_calculator_offset() -> None:
    calc = GroundTruthCalculator((0.1, 0.2, 0.3))
    pose = calc.compute_camera_pose(0.0, 0.0, 0.0)
    t = pose.translation()
    assert abs(t.x - 0.1) < 1e-9
    assert abs(t.y - 0.2) < 1e-9
    assert abs(t.z - 0.3) < 1e-9


def test_compare_no_vision() -> None:
    gt = GroundTruthSample(timestamp_s=1.0, camera_pose=Pose3d(), sensor_readings={})
    record = compute_error(gt, None)
    assert record.error is None
    assert record.vision_estimate is None


def test_compare_with_vision() -> None:
    gt = GroundTruthSample(timestamp_s=1.0, camera_pose=Pose3d(), sensor_readings={})
    ve = PoseMeasurement(timestamp_s=1.0, pose=Pose3d())
    record = compute_error(gt, ve)
    assert record.error is not None
    assert record.vision_estimate is not None


def test_compare_rotation_error_non_identity() -> None:
    """Rotation error should be R_gt * R_ve⁻¹, not R_ve⁻¹ * R_gt."""
    Rx = Rotation3d(math.pi / 4, 0.0, 0.0)  # 45° roll
    Ry = Rotation3d(0.0, math.pi / 4, 0.0)  # 45° pitch

    gt = GroundTruthSample(
        timestamp_s=1.0,
        camera_pose=Pose3d(Translation3d(), Rotation3d(math.pi / 4, math.pi / 4, 0)),
        sensor_readings={},
    )
    ve = PoseMeasurement(timestamp_s=1.0, pose=Pose3d(Translation3d(), Rx))

    record = compute_error(gt, ve)
    assert record.error is not None

    expected = Ry  # R_gt * R_ve⁻¹ = (Ry*Rx) * Rx⁻¹ = Ry
    actual = record.error.rotation()
    q_exp = expected.getQuaternion()
    q_act = actual.getQuaternion()

    assert abs(q_exp.W() - q_act.W()) < 1e-6
    assert abs(q_exp.X() - q_act.X()) < 1e-6
    assert abs(q_exp.Y() - q_act.Y()) < 1e-6
    assert abs(q_exp.Z() - q_act.Z()) < 1e-6


def test_compare_translation_error() -> None:
    """Translation error should be t_gt - t_ve."""
    gt = GroundTruthSample(
        timestamp_s=1.0,
        camera_pose=Pose3d(Translation3d(1.0, 0.0, 0.0), Rotation3d()),
        sensor_readings={},
    )
    ve = PoseMeasurement(
        timestamp_s=1.0, pose=Pose3d(Translation3d(0.8, 0.2, 0.0), Rotation3d())
    )

    record = compute_error(gt, ve)
    assert record.error is not None

    actual = record.error.translation()
    assert abs(actual.X() - 0.2) < 1e-6
    assert abs(actual.Y() - (-0.2)) < 1e-6
    assert abs(actual.Z()) < 1e-6
