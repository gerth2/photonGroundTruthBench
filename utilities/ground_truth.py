"""Domain logic for computing camera pose from servo commands.

No wpilib imports — pure Python for testability.
"""

from wpimath import Pose3d, Rotation3d, Translation3d


class GroundTruthCalculator:
    """Transforms commanded servo angles into an assumed-correct camera pose.

    The open-loop servo position is NOT trustworthy (3D-printed parts have
    slop), so this class is used only for sanity-checking and as a fallback
    when the IMU-based ground truth is unavailable.
    """

    def __init__(self, camera_to_bench_m: tuple[float, float, float]) -> None:
        self._camera_offset = Translation3d(*camera_to_bench_m)

    def compute_camera_pose(
        self,
        pitch_rad: float,
        yaw_rad: float,
        roll_rad: float,
        base_pose: Pose3d = Pose3d(),
    ) -> Pose3d:
        rotation = Rotation3d(pitch_rad, yaw_rad, roll_rad)
        translation = base_pose.translation() + self._camera_offset
        return Pose3d(translation, rotation)
