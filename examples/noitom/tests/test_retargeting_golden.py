# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Golden-data regression tests for Noitom G1 retargeting."""

from __future__ import annotations

import numpy as np
import pytest

from noitom_retargeting import (
    NoitomArmIkTargetNode,
    NoitomG1Retargeter,
    NoitomRetargetingSettings,
)

# ---------------------------------------------------------------------------
# Golden IK targets for the canonical mock frames.
# ---------------------------------------------------------------------------

# POST_CALIBRATION — smoothed poses immediately after calibrate(T-pose)
_GOLDEN_POST_CALIB = {
    "left_wrist_pos": [-0.18, 0.1, 0.8],
    "left_wrist_quat": [-0.2706, 0.6533, 0.2706, 0.6533],
    "right_wrist_pos": [0.18, 0.1, 0.8],
    "right_wrist_quat": [-0.7071, 0.0, 0.7071, 0.0],
    "left_elbow_pos": [0.05, 0.19, 0.74],
    "right_elbow_pos": [0.05, -0.19, 0.74],
    "left_shoulder_pos": [0.05, 0.19, 1.02],
    "right_shoulder_pos": [0.05, -0.19, 1.02],
}

# LEFT_ARM_RAISED — smoothed poses after retarget(left-arm-raised)
_GOLDEN_LEFT_RAISED = {
    "left_wrist_pos": [0.45345449413509986, 0.17649999999999996, 1.0178533274937558],
    "left_wrist_quat": [
        -0.2830254113448329,
        0.6218216811821073,
        0.25756152904925506,
        0.6832982307153705,
    ],
    "right_wrist_pos": [-0.372807088, -0.12463581972754674, 0.987],
    "right_wrist_quat": [
        -0.7065417665007234,
        0.028261850435477864,
        0.7065417665007234,
        0.028261850435477864,
    ],
    "left_elbow_pos": [0.28754054741968915, 0.18999999999999997, 0.9835647725752833],
    "right_elbow_pos": [-0.18800000000000003, -0.18999999999999997, 0.978],
    "left_shoulder_pos": [0.05, 0.19, 1.02],
    "right_shoulder_pos": [0.05, -0.19, 1.02],
}

# RIGHT_ARM_RAISED — smoothed poses after retarget(right-arm-raised)
_GOLDEN_RIGHT_RAISED = {
    "left_wrist_pos": [0.5528252621202651, 0.16611081972754674, 1.0196779991240634],
    "left_wrist_quat": [
        -0.24742877244122916,
        0.6558422745341793,
        0.24105777098283188,
        0.6712235578038431,
    ],
    "right_wrist_pos": [-0.4513755573350999, -0.18019537295913196, 1.0459033274937557],
    "right_wrist_quat": [
        -0.6730133965171332,
        0.00707157688935579,
        0.7395626773364568,
        0.007071576889355799,
    ],
    "left_elbow_pos": [0.3236310821129534, 0.18999999999999997, 1.0145347158862925],
    "right_elbow_pos": [-0.22324054741968916, -0.18999999999999997, 1.0192647725752833],
    "left_shoulder_pos": [0.05, 0.19, 1.02],
    "right_shoulder_pos": [0.05, -0.19, 1.02],
}

_TOL = 1e-9


def _build_retargeter() -> NoitomG1Retargeter:
    """Build retargeter with posture-based settings (no aligned skeleton dependency)."""
    settings = NoitomRetargetingSettings(
        wrist_orientation_mode="twist",
        track_aligned_mocap_wrists=False,
        track_elbow_ik_targets=True,
        track_shoulder_ik_targets=True,
        use_posture_based_arms=True,
        sync_nominal_at_calibration=False,
    )
    return NoitomG1Retargeter(settings=settings)


def _assert_se3(
    actual_pos: np.ndarray,
    actual_quat: np.ndarray,
    expected_pos: list[float],
    expected_quat: list[float],
    label: str,
) -> None:
    np.testing.assert_allclose(
        actual_pos,
        np.array(expected_pos),
        atol=_TOL,
        err_msg=f"{label} position mismatch",
    )
    np.testing.assert_allclose(
        actual_quat,
        np.array(expected_quat),
        atol=_TOL,
        err_msg=f"{label} quaternion mismatch",
    )


# ---------------------------------------------------------------------------
# Test: calibration produces correct initial smoothed poses
# ---------------------------------------------------------------------------


class TestCalibration:
    def test_calibrate_succeeds(self, tpose_frame):
        retargeter = _build_retargeter()
        assert retargeter.calibrate(tpose_frame) is True
        assert retargeter.is_calibrated

    def test_awaiting_calibration_before_calibrate(self):
        retargeter = _build_retargeter()
        assert retargeter.awaiting_calibration

    def test_post_calibration_wrist_positions(self, tpose_frame):
        retargeter = _build_retargeter()
        retargeter.calibrate(tpose_frame)
        targets = retargeter.current_arm_targets
        g = _GOLDEN_POST_CALIB
        _assert_se3(
            targets.left_wrist.position,
            targets.left_wrist.quaternion_xyzw,
            g["left_wrist_pos"],
            g["left_wrist_quat"],
            "post-calib left_wrist",
        )
        _assert_se3(
            targets.right_wrist.position,
            targets.right_wrist.quaternion_xyzw,
            g["right_wrist_pos"],
            g["right_wrist_quat"],
            "post-calib right_wrist",
        )

    def test_post_calibration_elbow_positions(self, tpose_frame):
        retargeter = _build_retargeter()
        retargeter.calibrate(tpose_frame)
        targets = retargeter.current_arm_targets
        g = _GOLDEN_POST_CALIB
        np.testing.assert_allclose(
            targets.left_elbow.position, g["left_elbow_pos"], atol=_TOL
        )
        np.testing.assert_allclose(
            targets.right_elbow.position, g["right_elbow_pos"], atol=_TOL
        )

    def test_post_calibration_shoulder_positions(self, tpose_frame):
        retargeter = _build_retargeter()
        retargeter.calibrate(tpose_frame)
        targets = retargeter.current_arm_targets
        g = _GOLDEN_POST_CALIB
        np.testing.assert_allclose(
            targets.left_shoulder.position, g["left_shoulder_pos"], atol=_TOL
        )
        np.testing.assert_allclose(
            targets.right_shoulder.position, g["right_shoulder_pos"], atol=_TOL
        )


# ---------------------------------------------------------------------------
# Test: retarget produces correct smoothed poses for moved arms
# ---------------------------------------------------------------------------


class TestRetargetGolden:
    def _calibrated_retargeter(self, tpose_frame) -> NoitomG1Retargeter:
        retargeter = _build_retargeter()
        ok = retargeter.calibrate(tpose_frame)
        assert ok, "calibration failed — test frames may be invalid"
        return retargeter

    def test_left_arm_raised_wrist(self, tpose_frame, left_raised_frame):
        r = self._calibrated_retargeter(tpose_frame)
        r.retarget(left_raised_frame)
        targets = r.current_arm_targets
        g = _GOLDEN_LEFT_RAISED
        _assert_se3(
            targets.left_wrist.position,
            targets.left_wrist.quaternion_xyzw,
            g["left_wrist_pos"],
            g["left_wrist_quat"],
            "left-raised left_wrist",
        )
        _assert_se3(
            targets.right_wrist.position,
            targets.right_wrist.quaternion_xyzw,
            g["right_wrist_pos"],
            g["right_wrist_quat"],
            "left-raised right_wrist",
        )

    def test_left_arm_raised_elbow(self, tpose_frame, left_raised_frame):
        r = self._calibrated_retargeter(tpose_frame)
        r.retarget(left_raised_frame)
        targets = r.current_arm_targets
        g = _GOLDEN_LEFT_RAISED
        np.testing.assert_allclose(
            targets.left_elbow.position, g["left_elbow_pos"], atol=_TOL
        )
        np.testing.assert_allclose(
            targets.right_elbow.position, g["right_elbow_pos"], atol=_TOL
        )

    def test_right_arm_raised_wrist(
        self, tpose_frame, left_raised_frame, right_raised_frame
    ):
        r = self._calibrated_retargeter(tpose_frame)
        r.retarget(left_raised_frame)  # advance state as in the golden-data run
        r.retarget(right_raised_frame)
        targets = r.current_arm_targets
        g = _GOLDEN_RIGHT_RAISED
        _assert_se3(
            targets.left_wrist.position,
            targets.left_wrist.quaternion_xyzw,
            g["left_wrist_pos"],
            g["left_wrist_quat"],
            "right-raised left_wrist",
        )
        _assert_se3(
            targets.right_wrist.position,
            targets.right_wrist.quaternion_xyzw,
            g["right_wrist_pos"],
            g["right_wrist_quat"],
            "right-raised right_wrist",
        )

    def test_right_arm_raised_elbow(
        self, tpose_frame, left_raised_frame, right_raised_frame
    ):
        r = self._calibrated_retargeter(tpose_frame)
        r.retarget(left_raised_frame)
        r.retarget(right_raised_frame)
        targets = r.current_arm_targets
        g = _GOLDEN_RIGHT_RAISED
        np.testing.assert_allclose(
            targets.left_elbow.position, g["left_elbow_pos"], atol=_TOL
        )
        np.testing.assert_allclose(
            targets.right_elbow.position, g["right_elbow_pos"], atol=_TOL
        )

    def test_body_yaw_delta_zero_for_static_torso(self, tpose_frame, left_raised_frame):
        r = self._calibrated_retargeter(tpose_frame)
        r.retarget(left_raised_frame)
        assert r.body_yaw_delta == pytest.approx(0.0, abs=_TOL)


# ---------------------------------------------------------------------------
# Test: clear_calibration resets state correctly
# ---------------------------------------------------------------------------


class TestClearCalibration:
    def test_clear_resets_is_calibrated(self, tpose_frame):
        r = _build_retargeter()
        r.calibrate(tpose_frame)
        assert r.is_calibrated
        r.clear_calibration()
        assert not r.is_calibrated
        assert r.awaiting_calibration

    def test_clear_resets_wrist_to_nominal(self, tpose_frame):
        r = _build_retargeter()
        r.calibrate(tpose_frame)
        r.clear_calibration()
        # After clearing, wrist should be back to factory defaults.
        np.testing.assert_allclose(
            r.current_left.position,
            [-0.18, 0.1, 0.8],
            atol=_TOL,
        )
        np.testing.assert_allclose(
            r.current_right.position,
            [0.18, 0.1, 0.8],
            atol=_TOL,
        )

    def test_recalibrate_after_clear(self, tpose_frame):
        r = _build_retargeter()
        r.calibrate(tpose_frame)
        r.clear_calibration()
        ok = r.calibrate(tpose_frame)
        assert ok
        assert r.is_calibrated


# ---------------------------------------------------------------------------
# Test: NoitomArmIkTargetNode unit tests
# ---------------------------------------------------------------------------


class TestArmIkTargetNode:
    """Exercise per-arm target state independently."""

    def _make_node(self, is_left: bool) -> NoitomArmIkTargetNode:
        settings = NoitomRetargetingSettings()
        pos = (
            settings.nominal_left_wrist_pos
            if is_left
            else settings.nominal_right_wrist_pos
        )
        quat = (
            settings.nominal_left_wrist_quat_xyzw
            if is_left
            else settings.nominal_right_wrist_quat_xyzw
        )
        return NoitomArmIkTargetNode(
            is_left=is_left,
            settings=settings,
            nominal_wrist_pos=pos,
            nominal_wrist_quat_xyzw=quat,
        )

    def test_initial_wrist_is_nominal(self):
        settings = NoitomRetargetingSettings()
        node = NoitomArmIkTargetNode(
            is_left=True,
            settings=settings,
            nominal_wrist_pos=settings.nominal_left_wrist_pos,
            nominal_wrist_quat_xyzw=settings.nominal_left_wrist_quat_xyzw,
        )
        np.testing.assert_allclose(
            node.current_wrist.position,
            settings.nominal_left_wrist_pos,
            atol=_TOL,
        )

    def test_bounded_twist_initially_none(self):
        node = self._make_node(is_left=True)
        assert node.bounded_twist_rad is None

    def test_apply_bound_twist_clamps(self):
        node = self._make_node(is_left=True)
        # With default limit 60°, input 90° should be clamped to 60°.
        limit_rad = float(np.deg2rad(60.0))
        result = node.apply_bound_twist(float(np.deg2rad(90.0)))
        assert abs(result) <= limit_rad + 1e-9

    def test_apply_bound_twist_accumulates(self):
        node = self._make_node(is_left=True)
        _ = node.apply_bound_twist(0.1)
        second = node.apply_bound_twist(0.2)
        assert node.bounded_twist_rad is not None
        assert node.bounded_twist_rad == pytest.approx(second, abs=_TOL)

    def test_reset_clears_bounded_twist(self):
        node = self._make_node(is_left=True)
        node.apply_bound_twist(0.3)
        assert node.bounded_twist_rad is not None
        node.reset()
        assert node.bounded_twist_rad is None

    def test_reset_to_nominal_updates_poses(self):
        from noitom_retargeting import SE3Pose

        node = self._make_node(is_left=True)
        new_pos = np.array([0.1, 0.2, 0.3])
        new_quat = np.array([0.0, 0.0, 0.0, 1.0])
        nominal_wrist = SE3Pose(new_pos.copy(), new_quat.copy())
        nominal_elbow = SE3Pose(new_pos.copy() + 0.1, new_quat.copy())
        nominal_shoulder = SE3Pose(new_pos.copy() + 0.2, new_quat.copy())
        node.reset_to_nominal(nominal_wrist, nominal_elbow, nominal_shoulder)
        np.testing.assert_allclose(node.current_wrist.position, new_pos, atol=_TOL)
        np.testing.assert_allclose(
            node.current_elbow.position, new_pos + 0.1, atol=_TOL
        )
        np.testing.assert_allclose(
            node.current_shoulder.position, new_pos + 0.2, atol=_TOL
        )
        assert node.bounded_twist_rad is None

    def test_importable_from_noitom_retargeter_nodes(self):
        from noitom_retargeter_nodes import NoitomArmIkTargetNode as NodeFromShim
        from noitom_retargeting import NoitomArmIkTargetNode as NodeFromRetargeting

        assert NodeFromShim is NodeFromRetargeting
