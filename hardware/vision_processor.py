"""Subsystem wrapping PhotonVision camera + pose estimator.

Polls the camera at the loop rate, holds the latest estimated pose, and
publishes the camera pose plus detected tag transforms to NetworkTables.
"""

import wpilib
from photonlibpy import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from photonlibpy.estimatedRobotPose import EstimatedRobotPose
from wpimath import Transform3d

from core.subsystem import Subsystem


class VisionProcessor(Subsystem):
    def __init__(
        self,
        camera_name: str,
        pose_estimator: PhotonPoseEstimator,
        camera_to_robot: Transform3d,
    ) -> None:
        super().__init__()

        self._camera = PhotonCamera(camera_name)
        self._estimator = pose_estimator
        self._camera_to_robot = camera_to_robot

        self._latest_pose: EstimatedRobotPose | None = None

    def get_latest_pose(self) -> EstimatedRobotPose | None:
        return self._latest_pose

    def periodic(self) -> None:
        sd = wpilib.SmartDashboard

        result = self._camera.getLatestResult()

        targets = result.getTargets() if result is not None else []
        for target in targets:
            id_ = target.getFiducialId()
            cam2tgt = target.getBestCameraToTarget()
            xyz = cam2tgt.translation()
            rpy = cam2tgt.rotation()
            sd.putNumberArray(
                f"vision/tag_{id_}/camera_to_target",
                [xyz.X(), xyz.Y(), xyz.Z(), rpy.X(), rpy.Y(), rpy.Z()],
            )
            sd.putNumber(f"vision/tag_{id_}/pose_ambiguity", target.getPoseAmbiguity())

        if not result.hasTargets():
            self._latest_pose = None
            sd.putNumberArray("vision/estimated_pose", [0.0] * 6)
            sd.putNumber("vision/pose_timestamp", 0.0)
            return

        pose = self._estimator.estimateCoprocMultiTagPose(result)
        if pose is None:
            pose = self._estimator.estimateLowestAmbiguityPose(result)

        self._latest_pose = pose

        if pose is not None:
            p3d = pose.estimatedPose
            t3d2 = p3d.translation()
            rot = p3d.rotation()
            sd.putNumberArray(
                "vision/estimated_pose",
                [t3d2.X(), t3d2.Y(), t3d2.Z(), rot.X(), rot.Y(), rot.Z()],
            )
            sd.putNumber("vision/pose_timestamp", pose.timestampSeconds)
