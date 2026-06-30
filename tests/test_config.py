"""Config consistency checks — no simulation needed."""

from config.bench_config import ValidationConfig


def test_static_expected_tags_matches_pose_count() -> None:
    """static_expected_tags must have one entry per pose in static_pose_deg."""
    vc = ValidationConfig
    assert len(vc.static_expected_tags) == len(vc.static_pose_deg), (
        f"Expected {len(vc.static_pose_deg)} tag entries, "
        f"got {len(vc.static_expected_tags)}"
    )
