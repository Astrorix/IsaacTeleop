# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Fixed-lower-body Noitom G1 task configuration and action-layout tests."""

from __future__ import annotations

import numpy as np

from noitom_retargeting import ArmIkTargets, SE3Pose
from noitom_tasks import (
    DEFAULT_NOITOM_G1_SETTINGS,
    G1LocomanipulationAction,
    NoitomLocomanipulationG1EnvCfg,
    _make_action,
    g1_action_dim,
)


def _pose(seed: float) -> SE3Pose:
    return SE3Pose(
        position=np.array([seed, seed + 1.0, seed + 2.0]),
        quaternion_xyzw=np.array([seed + 3.0, seed + 4.0, seed + 5.0, seed + 6.0]),
    )


def _targets() -> ArmIkTargets:
    return ArmIkTargets(
        left_wrist=_pose(0.0),
        right_wrist=_pose(10.0),
        left_elbow=_pose(20.0),
        right_elbow=_pose(30.0),
        left_shoulder=_pose(40.0),
        right_shoulder=_pose(50.0),
    )


def test_fixed_lower_body_action_dimensions_share_one_source() -> None:
    assert g1_action_dim(use_arm_ik_frame_tasks=True) == 56
    assert g1_action_dim(use_arm_ik_frame_tasks=False) == 28
    assert G1LocomanipulationAction(use_arm_ik_frame_tasks=True).types[0].shape == (56,)
    assert G1LocomanipulationAction(use_arm_ik_frame_tasks=False).types[0].shape == (
        28,
    )


def test_arm_frame_action_is_42_pose_plus_14_hand_values() -> None:
    targets = _targets()
    action = _make_action(targets, use_arm_ik_frame_tasks=True)
    expected_poses = np.concatenate(
        [
            targets.left_wrist.as_action_pose(),
            targets.right_wrist.as_action_pose(),
            targets.left_elbow.as_action_pose(),
            targets.right_elbow.as_action_pose(),
            targets.left_shoulder.as_action_pose(),
            targets.right_shoulder.as_action_pose(),
        ]
    )

    assert action.shape == (56,)
    np.testing.assert_allclose(action[:42], expected_poses)
    np.testing.assert_allclose(action[42:56], 0.0)


def test_wrist_only_action_is_14_pose_plus_14_hand_values() -> None:
    targets = _targets()
    action = _make_action(targets, use_arm_ik_frame_tasks=False)
    expected_poses = np.concatenate(
        [targets.left_wrist.as_action_pose(), targets.right_wrist.as_action_pose()]
    )

    assert action.shape == (28,)
    np.testing.assert_allclose(action[:14], expected_poses)
    np.testing.assert_allclose(action[14:28], 0.0)


def test_noitom_env_disables_agile_policy_and_preserves_pink_scope(
    capsys, monkeypatch
) -> None:
    monkeypatch.setenv("NOITOM_MOCAP_AUTO_LAUNCH", "0")
    cfg = NoitomLocomanipulationG1EnvCfg()

    assert cfg.scene.robot.spawn.articulation_props.fix_root_link is True
    assert cfg.actions.lower_body_joint_pos is None
    assert cfg.observations.lower_body_policy is None
    assert "root_fixed=1 leg_action=disabled action_dim=56" in capsys.readouterr().out

    pink_action = cfg.actions.upper_body_ik
    controlled_patterns = pink_action.pink_controlled_joint_names
    assert "waist_.*_joint" in controlled_patterns
    assert not any(
        leg_name in pattern
        for pattern in controlled_patterns
        for leg_name in ("hip", "knee", "ankle")
    )
    pink_tasks = pink_action.controller.variable_input_tasks
    assert len(pink_tasks) == 7
    assert [(task.position_cost, task.orientation_cost) for task in pink_tasks[:6]] == [
        (18.0, 0.75),
        (18.0, 0.75),
        (9.0, 0.0),
        (9.0, 0.0),
        (6.0, 0.0),
        (6.0, 0.0),
    ]
    assert pink_tasks[6].cost == 0.05
    assert pink_action.controller.fail_on_joint_limit_violation is False
    assert DEFAULT_NOITOM_G1_SETTINGS.wrist_pitch_limit_deg == 80.0
    assert DEFAULT_NOITOM_G1_SETTINGS.wrist_yaw_limit_deg == 80.0
    assert not any(
        "agile" in str(value).lower()
        for value in vars(cfg.actions).values()
        if value is not None
    )
