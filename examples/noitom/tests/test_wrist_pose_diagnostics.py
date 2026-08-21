# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Layered 7DoF wrist-pose diagnostic regression tests."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from noitom_retargeting import (
    DEFAULT_NOITOM_IK_CONFIG_PATH,
    NoitomG1Retargeter,
    NoitomRetargetingSettings,
    noitom_position_to_isaac,
    noitom_quaternion_to_isaac,
)
from noitom_tasks import (
    NoitomG1ActionSource,
    _environment_world_pose,
    _pink_solution_fk_pelvis_poses,
    _pose_error,
    _pose_from_local_frame,
    _pose_in_local_frame,
    _pose_to_matrix,
)


def _retargeter() -> NoitomG1Retargeter:
    return NoitomG1Retargeter(
        NoitomRetargetingSettings(
            wrist_orientation_mode="source",
            position_smoothing=1.0,
            rotation_smoothing=1.0,
            motion_scale=1.0,
            ik_config_path=str(DEFAULT_NOITOM_IK_CONFIG_PATH),
        )
    )


def _wrist_action(retargeter: NoitomG1Retargeter) -> np.ndarray:
    targets = retargeter.current_arm_targets
    return np.concatenate(
        [targets.left_wrist.as_action_pose(), targets.right_wrist.as_action_pose()]
    )


def test_raw_semantic_and_reference_wrist_diagnostics_share_paths(
    bvh_rest_tpose_frame,
) -> None:
    retargeter = _retargeter()
    assert retargeter.calibrate(bvh_rest_tpose_frame)

    diagnostics = retargeter.wrist_pose_diagnostics(bvh_rest_tpose_frame)
    assert diagnostics is not None
    expected_raw_position = noitom_position_to_isaac([0.8, 1.25, 0.0])
    expected_raw_rotation = noitom_quaternion_to_isaac([0.0, 0.0, 0.0, 1.0])
    np.testing.assert_allclose(
        diagnostics["left"].bvh_raw_isaac_world.position, expected_raw_position
    )
    np.testing.assert_allclose(
        diagnostics["left"].bvh_raw_isaac_world.quaternion_xyzw,
        expected_raw_rotation,
    )
    assert np.linalg.norm(
        diagnostics["left"].bvh_raw_isaac_world.quaternion_xyzw
    ) == pytest.approx(1.0)

    reference_frames = retargeter.reference_wrist_frames(bvh_rest_tpose_frame)
    reference_positions = retargeter.reference_skeleton_positions(bvh_rest_tpose_frame)
    for side, wrist_index in (("left", 20), ("right", 21)):
        np.testing.assert_allclose(
            diagnostics[side].bvh_aligned_raw_world.position,
            reference_positions[wrist_index],
        )
        np.testing.assert_allclose(
            diagnostics[side].bvh_semantic_world.quaternion_xyzw,
            reference_frames[side].quaternion_xyzw,
        )
        for pose in (
            diagnostics[side].bvh_aligned_raw_world,
            diagnostics[side].bvh_semantic_world,
        ):
            assert np.linalg.norm(pose.quaternion_xyzw) == pytest.approx(1.0)

    left_raw_semantic_error = _pose_error(
        diagnostics["left"].bvh_aligned_raw_world.as_action_pose(),
        diagnostics["left"].bvh_semantic_world.as_action_pose(),
    )
    right_raw_semantic_error = _pose_error(
        diagnostics["right"].bvh_aligned_raw_world.as_action_pose(),
        diagnostics["right"].bvh_semantic_world.as_action_pose(),
    )
    for side, error in (
        ("left", left_raw_semantic_error),
        ("right", right_raw_semantic_error),
    ):
        expected_deg = np.rad2deg(
            Rotation.from_quat(diagnostics[side].local_offset_xyzw).magnitude()
        )
        assert error[1] == pytest.approx(expected_deg)


def test_first_valid_wrist_pose_waits_for_both_wrists_and_resets(
    bvh_rest_tpose_frame, bvh_tpose_frame_factory, capsys
) -> None:
    retargeter = _retargeter()
    assert retargeter.calibrate(bvh_rest_tpose_frame)
    source = object.__new__(NoitomG1ActionSource)
    source._orientation_debug = True
    source._first_valid_wrist_pose_printed = False
    source._frame_count = 17
    source._retargeter = retargeter
    action = _wrist_action(retargeter)

    one_wrist_invalid = bvh_tpose_frame_factory()
    one_wrist_invalid.joints.joints(21).is_valid = False
    source._maybe_print_first_valid_wrist_pose(one_wrist_invalid, action)
    assert not source._first_valid_wrist_pose_printed
    assert capsys.readouterr().out == ""

    source._maybe_print_first_valid_wrist_pose(bvh_rest_tpose_frame, action)
    first_output = capsys.readouterr().out
    assert source._first_valid_wrist_pose_printed
    assert first_output.count("NoitomWristPoseDebug") == 8
    assert first_output.count("NoitomWristAxisDebug") == 2
    assert "source_frame=17" in first_output
    assert "quat_xyzw=" in first_output
    assert "stage=bvh_aligned_raw_world side=left" in first_output
    assert "stage=bvh_semantic_world side=left" in first_output
    assert "raw_to_semantic_rot_deg=180.000000" in first_output
    assert "semantic_to_retarget_rot_deg=0.000000" in first_output
    assert "raw_x_dot_forearm=-1.000000" in first_output
    assert "semantic_x_dot_forearm=+1.000000" in first_output

    source._maybe_print_first_valid_wrist_pose(bvh_rest_tpose_frame, action)
    assert capsys.readouterr().out == ""
    source._reset_wrist_pose_debug_state()
    source._maybe_print_first_valid_wrist_pose(bvh_rest_tpose_frame, action)
    assert capsys.readouterr().out.count("NoitomWristPoseDebug") == 8


def test_disabled_first_pose_diagnostic_has_no_output_or_pose_work(capsys) -> None:
    source = object.__new__(NoitomG1ActionSource)
    source._orientation_debug = False
    source._first_valid_wrist_pose_printed = False
    source._retargeter = SimpleNamespace(
        is_calibrated=True,
        wrist_pose_diagnostics=lambda _frame: pytest.fail(
            "disabled diagnostics must not inspect wrist poses"
        ),
    )

    source._maybe_print_first_valid_wrist_pose(object(), np.zeros(14))
    assert capsys.readouterr().out == ""


def test_pose_error_treats_quaternion_signs_as_equivalent() -> None:
    pose = np.array([0.1, -0.2, 0.3, 0.1, -0.2, 0.3, 0.9])
    opposite = pose.copy()
    opposite[3:7] *= -1.0

    assert _pose_error(pose, opposite) == pytest.approx((0.0, 0.0))


def test_environment_world_pose_subtracts_nonzero_origin() -> None:
    simulator_pose = np.array([11.0, -3.0, 5.5, 0.0, 0.0, 0.0, 1.0])
    environment_pose = _environment_world_pose(
        simulator_pose, np.array([10.0, -4.0, 2.5])
    )

    np.testing.assert_allclose(environment_pose[:3], [1.0, 1.0, 3.0])
    np.testing.assert_allclose(environment_pose[3:7], simulator_pose[3:7])


def test_pelvis_local_pose_round_trip() -> None:
    pelvis_world = np.array(
        [0.4, -0.3, 0.8, *Rotation.from_euler("z", 37.0, degrees=True).as_quat()]
    )
    world_pose = np.array(
        [
            0.7,
            0.2,
            1.1,
            *Rotation.from_euler("xyz", [20, -10, 5], degrees=True).as_quat(),
        ]
    )
    pelvis_matrix = _pose_to_matrix(pelvis_world)

    local_pose = _pose_in_local_frame(world_pose, pelvis_matrix)
    reconstructed = _pose_from_local_frame(local_pose, pelvis_matrix)
    assert _pose_error(world_pose, reconstructed) == pytest.approx((0.0, 0.0))


class _FakeTransform:
    def __init__(self, translation: np.ndarray) -> None:
        self.translation = translation
        self.rotation = np.eye(3)


class _FakePinkConfiguration:
    def __init__(self) -> None:
        self.full_q = np.array([7.0, 8.0, 9.0])
        self.updated: list[np.ndarray] = []

    def update(self, configuration: np.ndarray) -> None:
        self.full_q = np.asarray(configuration, dtype=np.float64).copy()
        self.updated.append(self.full_q.copy())

    def get_transform(self, frame: str, base: str) -> _FakeTransform:
        assert base.endswith("_pelvis")
        side_offset = 1.0 if "left" in frame else -1.0
        return _FakeTransform(np.array([np.sum(self.full_q), side_offset, 0.0]))


def test_solution_fk_helper_restores_pink_configuration() -> None:
    configuration = _FakePinkConfiguration()
    controller = SimpleNamespace(
        pink_configuration=configuration,
        isaac_lab_to_pink_ordering=np.array([2, 0, 1]),
    )
    original = configuration.full_q.copy()

    poses = _pink_solution_fk_pelvis_poses(
        controller,
        current_joint_positions_isaac=np.array([1.0, 2.0, 3.0]),
        controlled_joint_ids=[0],
        final_solution=np.array([10.0]),
        wrist_frame_names={"left": "left_wrist", "right": "right_wrist"},
    )

    assert set(poses) == {"left", "right"}
    np.testing.assert_allclose(configuration.full_q, original)
    np.testing.assert_allclose(configuration.updated[-1], original)
