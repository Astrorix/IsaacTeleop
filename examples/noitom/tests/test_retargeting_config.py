# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Phase-B tests for declarative Noitom-to-G1 arm mapping."""

from __future__ import annotations

import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from noitom_retargeting import (
    DEFAULT_NOITOM_IK_CONFIG_PATH,
    NoitomG1Retargeter,
    NoitomRetargetingSettings,
    load_noitom_ik_config,
)
from noitom_tasks import DEFAULT_NOITOM_G1_SETTINGS


def _config_settings() -> NoitomRetargetingSettings:
    return NoitomRetargetingSettings(
        wrist_orientation_mode="twist",
        track_aligned_mocap_wrists=True,
        track_elbow_ik_targets=True,
        track_shoulder_ik_targets=True,
        use_posture_based_arms=True,
        sync_nominal_at_calibration=True,
        motion_scale=1.0,
        position_smoothing=1.0,
        rotation_smoothing=1.0,
        ik_config_path=str(DEFAULT_NOITOM_IK_CONFIG_PATH),
    )


def test_config_covers_only_six_upper_body_targets() -> None:
    config = load_noitom_ik_config(DEFAULT_NOITOM_IK_CONFIG_PATH)

    assert len(config.ik_match_table) == 6
    assert set(config.arm_chains) == {"left", "right"}
    mapped_joints = {match.human_joint for match in config.ik_match_table.values()}
    assert mapped_joints == {
        "LEFT_SHOULDER",
        "LEFT_ELBOW",
        "LEFT_WRIST",
        "RIGHT_SHOULDER",
        "RIGHT_ELBOW",
        "RIGHT_WRIST",
    }


def test_config_wrist_offsets_normalize_mirrored_hand_frames() -> None:
    config = load_noitom_ik_config(DEFAULT_NOITOM_IK_CONFIG_PATH)

    left_offset = config.match("left", "wrist").rotation_offset_xyzw
    right_offset = config.match("right", "wrist").rotation_offset_xyzw
    np.testing.assert_allclose(left_offset, [0.0, np.sqrt(0.5), np.sqrt(0.5), 0.0])
    np.testing.assert_allclose(right_offset, [-np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)])
    for offset in (left_offset, right_offset):
        assert np.linalg.norm(offset) == pytest.approx(1.0)
        assert np.linalg.det(Rotation.from_quat(offset).as_matrix()) == pytest.approx(
            1.0
        )


def test_config_rejects_incomplete_match_table(tmp_path) -> None:
    raw = json.loads(DEFAULT_NOITOM_IK_CONFIG_PATH.read_text(encoding="utf-8"))
    del raw["ik_match_table"]["left_wrist_yaw_link"]
    invalid_path = tmp_path / "invalid.json"
    invalid_path.write_text(json.dumps(raw), encoding="utf-8")

    with pytest.raises(ValueError, match="ik_match_table must match arm_chains"):
        load_noitom_ik_config(invalid_path)


def test_calibration_applies_per_segment_human_scales(tpose_frame) -> None:
    retargeter = NoitomG1Retargeter(_config_settings())

    assert retargeter.calibrate(tpose_frame)
    scales = retargeter.calibration_bone_scales
    assert scales is not None
    left, right = scales
    # Fixture bones are 0.30 m; upper arms use 0.8 and forearms use 0.7.
    assert left.upper_arm == pytest.approx(0.24)
    assert left.forearm == pytest.approx(0.21)
    assert right.upper_arm == pytest.approx(0.24)
    assert right.forearm == pytest.approx(0.21)


def test_neutral_targets_match_configured_segment_lengths(tpose_frame) -> None:
    retargeter = NoitomG1Retargeter(_config_settings())
    assert retargeter.calibrate(tpose_frame)

    targets = retargeter.current_arm_targets
    for shoulder, elbow, wrist in (
        (targets.left_shoulder, targets.left_elbow, targets.left_wrist),
        (targets.right_shoulder, targets.right_elbow, targets.right_wrist),
    ):
        assert np.linalg.norm(elbow.position - shoulder.position) == pytest.approx(
            0.24, abs=1.0e-9
        )
        assert np.linalg.norm(wrist.position - elbow.position) == pytest.approx(
            0.21, abs=1.0e-9
        )


def test_raised_arm_uses_shortest_arc_and_configured_reach(
    tpose_frame, left_raised_frame
) -> None:
    retargeter = NoitomG1Retargeter(_config_settings())
    assert retargeter.calibrate(tpose_frame)

    targets = retargeter.retarget(left_raised_frame)
    assert targets is not None
    shoulder = targets.left_shoulder.position
    elbow = targets.left_elbow.position
    wrist = targets.left_wrist.position

    # With motion_scale=1, both links rotate 90 degrees and point upward.
    np.testing.assert_allclose(elbow - shoulder, [0.0, 0.0, 0.24], atol=1.0e-9)
    np.testing.assert_allclose(wrist - elbow, [0.0, 0.0, 0.21], atol=1.0e-9)
    assert np.linalg.norm(targets.left_wrist.quaternion_xyzw) == pytest.approx(1.0)


def test_config_change_from_fixed_lengths_is_bounded(
    tpose_frame, left_raised_frame
) -> None:
    config = NoitomG1Retargeter(_config_settings())
    fixed_settings = _config_settings()
    fixed_settings.ik_config_path = None
    fixed = NoitomG1Retargeter(fixed_settings)
    assert config.calibrate(tpose_frame)
    assert fixed.calibrate(tpose_frame)

    config_targets = config.retarget(left_raised_frame)
    fixed_targets = fixed.retarget(left_raised_frame)
    assert config_targets is not None and fixed_targets is not None
    position_delta = np.linalg.norm(
        config_targets.left_wrist.position - fixed_targets.left_wrist.position
    )
    # Fixed-length mode has 0.378 m reach; the calibrated config has 0.45 m reach.
    assert position_delta < 0.11


@pytest.mark.parametrize("mode", ["forearm", "twist", "full"])
def test_semantic_offset_does_not_change_compatibility_modes(
    tmp_path, tpose_frame, left_raised_frame, mode: str
) -> None:
    identity_config = json.loads(
        DEFAULT_NOITOM_IK_CONFIG_PATH.read_text(encoding="utf-8")
    )
    identity_config["ik_match_table"]["left_wrist_yaw_link"][4] = [
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    identity_path = tmp_path / "identity-left-wrist.json"
    identity_path.write_text(json.dumps(identity_config), encoding="utf-8")

    semantic_settings = _config_settings()
    semantic_settings.wrist_orientation_mode = mode
    identity_settings = _config_settings()
    identity_settings.wrist_orientation_mode = mode
    identity_settings.ik_config_path = str(identity_path)
    semantic = NoitomG1Retargeter(semantic_settings)
    identity = NoitomG1Retargeter(identity_settings)
    assert semantic.calibrate(tpose_frame)
    assert identity.calibrate(tpose_frame)

    semantic_targets = semantic.retarget(left_raised_frame)
    identity_targets = identity.retarget(left_raised_frame)
    assert semantic_targets is not None and identity_targets is not None
    for semantic_pose, identity_pose in (
        (semantic_targets.left_wrist, identity_targets.left_wrist),
        (semantic_targets.right_wrist, identity_targets.right_wrist),
    ):
        np.testing.assert_allclose(semantic_pose.position, identity_pose.position)
        assert (
            _rotation_error_deg(
                semantic_pose.quaternion_xyzw, identity_pose.quaternion_xyzw
            )
            < 1.0e-9
        )


def _source_settings(*, rotation_smoothing: float = 1.0) -> NoitomRetargetingSettings:
    settings = _config_settings()
    settings.wrist_orientation_mode = "source"
    settings.rotation_smoothing = rotation_smoothing
    return settings


def _rotation_error_deg(actual: np.ndarray, expected: np.ndarray) -> float:
    error = Rotation.from_quat(expected).inv() * Rotation.from_quat(actual)
    return float(np.rad2deg(error.magnitude()))


def test_source_rest_tpose_applies_side_specific_semantic_offsets(
    bvh_rest_tpose_frame,
) -> None:
    retargeter = NoitomG1Retargeter(_source_settings())
    assert retargeter.calibrate(bvh_rest_tpose_frame)

    references = retargeter.reference_wrist_frames(bvh_rest_tpose_frame)
    targets = retargeter.current_arm_targets
    diagnostics = retargeter.wrist_pose_diagnostics(bvh_rest_tpose_frame)
    assert diagnostics is not None
    config = load_noitom_ik_config(DEFAULT_NOITOM_IK_CONFIG_PATH)
    for side, reference, target in (
        ("left", references["left"], targets.left_wrist),
        ("right", references["right"], targets.right_wrist),
    ):
        raw = diagnostics[side].bvh_aligned_raw_world.quaternion_xyzw
        semantic = diagnostics[side].bvh_semantic_world.quaternion_xyzw
        expected_offset = config.match(side, "wrist").rotation_offset_xyzw
        expected = (
            Rotation.from_quat(raw) * Rotation.from_quat(expected_offset)
        ).as_quat()
        assert _rotation_error_deg(semantic, expected) < 1.0e-9
        assert _rotation_error_deg(reference.quaternion_xyzw, expected) < 1.0e-9
        assert _rotation_error_deg(target.quaternion_xyzw, expected) < 1.0e-9


@pytest.mark.parametrize(
    "rotation",
    [
        Rotation.from_euler("x", 35.0, degrees=True),
        Rotation.from_euler("y", -40.0, degrees=True),
        Rotation.from_euler("z", 55.0, degrees=True),
        Rotation.from_euler("xyz", [25.0, -30.0, 45.0], degrees=True),
    ],
)
@pytest.mark.parametrize("side", ["left", "right"])
def test_source_mode_matches_each_bvh_wrist_axis(
    bvh_rest_tpose_frame,
    bvh_tpose_frame_factory,
    rotation: Rotation,
    side: str,
) -> None:
    retargeter = NoitomG1Retargeter(_source_settings())
    assert retargeter.calibrate(bvh_rest_tpose_frame)
    identity = (0.0, 0.0, 0.0, 1.0)
    source_quat = tuple(rotation.as_quat())
    frame = bvh_tpose_frame_factory(
        source_quat if side == "left" else identity,
        source_quat if side == "right" else identity,
    )

    targets = retargeter.retarget(frame)
    assert targets is not None
    reference = retargeter.reference_wrist_frames(frame)[side]
    diagnostics = retargeter.wrist_pose_diagnostics(frame)
    assert diagnostics is not None
    target = targets.left_wrist if side == "left" else targets.right_wrist
    raw = diagnostics[side].bvh_aligned_raw_world.quaternion_xyzw
    configured_offset = diagnostics[side].local_offset_xyzw
    expected = (
        Rotation.from_quat(raw) * Rotation.from_quat(configured_offset)
    ).as_quat()
    assert _rotation_error_deg(reference.quaternion_xyzw, expected) < 1.0e-9
    assert _rotation_error_deg(target.quaternion_xyzw, expected) < 1.0e-9


def test_semantic_x_axes_follow_both_forearms(bvh_rest_tpose_frame) -> None:
    retargeter = NoitomG1Retargeter(_source_settings())
    assert retargeter.calibrate(bvh_rest_tpose_frame)

    diagnostics = retargeter.wrist_pose_diagnostics(bvh_rest_tpose_frame)
    assert diagnostics is not None
    assert diagnostics["left"].raw_x_dot_forearm is not None
    assert diagnostics["left"].raw_x_dot_forearm < -0.95
    for side in ("left", "right"):
        assert diagnostics[side].semantic_x_dot_forearm is not None
        assert diagnostics[side].semantic_x_dot_forearm > 0.95


def test_source_mode_uses_identity_offset_without_config(bvh_rest_tpose_frame) -> None:
    settings = _source_settings()
    settings.ik_config_path = None
    retargeter = NoitomG1Retargeter(settings)

    assert retargeter.calibrate(bvh_rest_tpose_frame)
    identity = np.array([0.0, 0.0, 0.0, 1.0])
    assert (
        _rotation_error_deg(
            retargeter.current_arm_targets.left_wrist.quaternion_xyzw, identity
        )
        < 1.0e-9
    )
    assert (
        _rotation_error_deg(
            retargeter.current_arm_targets.right_wrist.quaternion_xyzw, identity
        )
        < 1.0e-9
    )


def test_source_target_error_converges_with_rotation_smoothing(
    bvh_rest_tpose_frame, bvh_tpose_frame_factory
) -> None:
    retargeter = NoitomG1Retargeter(_source_settings(rotation_smoothing=0.5))
    assert retargeter.calibrate(bvh_rest_tpose_frame)
    source_quat = tuple(Rotation.from_euler("x", 90.0, degrees=True).as_quat())
    frame = bvh_tpose_frame_factory(source_quat, source_quat)

    assert retargeter.retarget(frame) is not None
    first = retargeter.wrist_orientation_diagnostics(frame)
    assert first is not None
    assert retargeter.retarget(frame) is not None
    second = retargeter.wrist_orientation_diagnostics(frame)
    assert second is not None

    for side in ("left", "right"):
        assert 0.0 < second[side].source_target_error_deg
        assert (
            second[side].source_target_error_deg < first[side].source_target_error_deg
        )


def test_wrist_orientation_defaults_are_source() -> None:
    assert NoitomRetargetingSettings().wrist_orientation_mode == "source"
    assert DEFAULT_NOITOM_G1_SETTINGS.retargeting.wrist_orientation_mode == "source"
