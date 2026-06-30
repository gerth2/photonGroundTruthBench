"""Pure-math comparison between ground truth and vision estimates."""

from wpimath import Pose3d
from utilities.measurement import ComparisonRecord, GroundTruthSample, PoseMeasurement


def compute_error(
    ground_truth: GroundTruthSample,
    vision_estimate: PoseMeasurement | None,
) -> ComparisonRecord:
    """Return the translation + rotation error from vision estimate to ground truth.

    Mathematically:  error_rot = R_gt * R_ve⁻¹
    (rotation that corrects the vision estimate to match ground truth).

    Because WPILib's rotateBy implements `a.rotateBy(b) = b * a`
    (first apply *this, then the argument), we call
    `ve.inverse().rotateBy(gt)` to get `gt * ve⁻¹ = R_gt * R_ve⁻¹`.
    """
    if vision_estimate is None:
        return ComparisonRecord(
            timestamp_s=ground_truth.timestamp_s,
            ground_truth=ground_truth,
            vision_estimate=None,
            error=None,
        )

    gt = ground_truth.camera_pose
    ve = vision_estimate.pose

    error = Pose3d(
        gt.translation() - ve.translation(),
        ve.rotation().inverse().rotateBy(gt.rotation()),
    )

    return ComparisonRecord(
        timestamp_s=ground_truth.timestamp_s,
        ground_truth=ground_truth,
        vision_estimate=vision_estimate,
        error=error,
    )
