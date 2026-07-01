"""Wraps PhotonCamera and computes camera pose from individual tag detections.

Uses known bench-frame tag poses to invert PV's camera-to-target transform
into a camera-pose estimate per visible bench tag.
"""

import ntcore
import wpilib
from photonlibpy import PhotonCamera
from wpimath import Pose3d, Transform3d

from core.subsystem import Subsystem


class VisionProcessor(Subsystem):
    """Reads PhotonCamera results and computes camera-pose estimates per visible bench tag.

    Lifecycle: constructed with a camera name and a dict mapping known bench
    tag IDs to their bench-frame Pose3d.  ``periodic()`` fetches the latest
    camera result; call ``get_tag_camera_poses()`` to obtain per-tag camera-
    pose estimates for tags visible in that frame.
    """

    def __init__(
        self,
        camera_name: str,
        bench_tag_poses: dict[int, Pose3d],
    ) -> None:
        """Initialise the PhotonCamera, store the bench-tag-pose map, and
        create NetworkTables struct publishers for per-tag results.

        Args:
            camera_name: NetworkTables name of the PhotonVision camera.
            bench_tag_poses: Mapping from tag ID to its known bench-frame
                ``Pose3d`` (e.g. ``{6: left_tag_pose, 7: right_tag_pose}``).
        """
        super().__init__()

        self._camera = PhotonCamera(camera_name)
        self._bench_tag_poses = bench_tag_poses

        self._tag_camera_poses: dict[int, Pose3d] = {}

        inst = ntcore.NetworkTableInstance.getDefault()
        vt = inst.getTable("vision")

        self._pub_visible_count = vt.getIntegerTopic("visible_bench_tags").publish()

        self._pubs_tag_pose: dict[int, ntcore.StructPublisher] = {}
        self._pubs_tag_tx: dict[int, ntcore.StructPublisher] = {}
        for tag_id in bench_tag_poses:
            t = vt.getStructTopic(f"tag_{tag_id}/camera_pose", Pose3d)
            self._pubs_tag_pose[tag_id] = t.publish()
            t = vt.getStructTopic(f"tag_{tag_id}/camera_to_target", Transform3d)
            self._pubs_tag_tx[tag_id] = t.publish()

    def get_tag_camera_poses(self) -> dict[int, Pose3d]:
        """Return latest camera-pose estimate per visible bench tag.

        Returns a copy of the internal dict so callers can iterate without
        worrying about mutation during the next periodic() call.
        """
        return dict(self._tag_camera_poses)

    def periodic(self) -> None:
        """Fetch the latest camera result and compute camera poses.

        For each detected target whose tag ID is present in
        ``_bench_tag_poses``, computes::

            camera_pose = tag_pose * inverse(camera_to_target)

        and publishes both the raw ``camera_to_target`` transform (as a
        ``Transform3d`` struct) and the computed camera pose (as a ``Pose3d``
        struct) to NetworkTables.
        """
        sd = wpilib.SmartDashboard

        result = self._camera.getLatestResult()
        self._tag_camera_poses.clear()

        if result is None or not result.hasTargets():
            self._pub_visible_count.set(0)
            sd.putNumber("vision/pose_timestamp", 0.0)
            return

        visible = 0
        for target in result.getTargets():
            tag_id = target.getFiducialId()
            if tag_id not in self._bench_tag_poses:
                continue
            visible += 1

            cam_to_tgt = target.getBestCameraToTarget()
            sd.putNumber(
                f"vision/tag_{tag_id}/pose_ambiguity",
                target.getPoseAmbiguity(),
            )

            self._pubs_tag_tx[tag_id].set(cam_to_tgt)

            tag_pose = self._bench_tag_poses[tag_id]
            camera_pose = tag_pose.transformBy(cam_to_tgt.inverse())
            self._tag_camera_poses[tag_id] = camera_pose
            self._pubs_tag_pose[tag_id].set(camera_pose)

        self._pub_visible_count.set(visible)
