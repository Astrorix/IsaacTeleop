# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Pytest configuration and shared fixtures for Noitom retargeting tests.

Tests run without Isaac Lab / Isaac Sim; they use a lightweight mock frame
that satisfies the same duck-typed interface that ``noitom_retargeting``
reads (``frame.joints.joints(i).is_valid``, ``.pose.position.{x,y,z}``,
``.pose.orientation.{x,y,z,w}``).
"""

from __future__ import annotations

import sys
import pathlib

# Make the examples/noitom directory importable without installation.
_noitom_dir = pathlib.Path(__file__).parent.parent
if str(_noitom_dir) not in sys.path:
    sys.path.insert(0, str(_noitom_dir))

import pytest  # noqa: E402


# ---------------------------------------------------------------------------
# BodyJoint integer constants (avoids importing isaacteleop in fixtures)
# Values verified against isaacteleop.schema.BodyJoint.
# ---------------------------------------------------------------------------
PELVIS = 0
SPINE1 = 3
SPINE2 = 6
SPINE3 = 9
NECK = 12
HEAD = 15
LEFT_COLLAR = 13
RIGHT_COLLAR = 14
LEFT_SHOULDER = 16
RIGHT_SHOULDER = 17
LEFT_ELBOW = 18
RIGHT_ELBOW = 19
LEFT_WRIST = 20
RIGHT_WRIST = 21
LEFT_HAND = 22
RIGHT_HAND = 23
NUM_JOINTS = 24


# ---------------------------------------------------------------------------
# Minimal mock-frame classes (no flatbuffer dependency)
# ---------------------------------------------------------------------------


class _MockPoint:
    __slots__ = ("x", "y", "z")

    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)


class _MockOrientation:
    __slots__ = ("x", "y", "z", "w")

    def __init__(
        self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0
    ) -> None:
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.w = float(w)


class _MockPose:
    __slots__ = ("position", "orientation")

    def __init__(
        self,
        pos: tuple[float, float, float],
        quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    ) -> None:
        self.position = _MockPoint(*pos)
        self.orientation = _MockOrientation(*quat)


class _MockJoint:
    __slots__ = ("is_valid", "pose")

    def __init__(
        self,
        pos: tuple[float, float, float],
        quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
        valid: bool = True,
    ) -> None:
        self.is_valid = valid
        self.pose = _MockPose(pos, quat)


class _MockJoints:
    def __init__(self, joint_map: dict[int, _MockJoint]) -> None:
        self._joints = joint_map
        self._invalid = _MockJoint((0.0, 0.0, 0.0), valid=False)

    def joints(self, index: int) -> _MockJoint:
        return self._joints.get(int(index), self._invalid)


class MockFrame:
    """Minimal FullBodyPose stand-in for tests (no flatbuffer required).

    Joints are specified in Noitom Y-up coordinates (same convention as the
    actual device data). The retargeting code converts them internally via
    ``_NOITOM_TO_ISAAC``.
    """

    def __init__(self, joint_map: dict[int, _MockJoint]) -> None:
        self.joints = _MockJoints(joint_map)


# ---------------------------------------------------------------------------
# Canonical test frames (Noitom Y-up coordinates)
# ---------------------------------------------------------------------------


def _make_tpose_frame() -> MockFrame:
    """T-pose neutral: arms extended horizontally, operator standing upright.

    Used as the calibration frame in regression tests.
    """
    quat_left = (0.0, 0.0, 0.707, 0.707)
    quat_right = (0.0, 0.0, -0.707, 0.707)
    return MockFrame(
        {
            PELVIS: _MockJoint((0.0, 0.85, 0.0)),
            SPINE1: _MockJoint((0.0, 0.95, 0.0)),
            SPINE2: _MockJoint((0.0, 1.05, 0.0)),
            SPINE3: _MockJoint((0.0, 1.25, 0.0)),
            NECK: _MockJoint((0.0, 1.35, 0.0)),
            HEAD: _MockJoint((0.0, 1.60, 0.0)),
            LEFT_COLLAR: _MockJoint((0.1, 1.25, 0.0)),
            RIGHT_COLLAR: _MockJoint((-0.1, 1.25, 0.0)),
            LEFT_SHOULDER: _MockJoint((0.20, 1.25, 0.0)),
            RIGHT_SHOULDER: _MockJoint((-0.20, 1.25, 0.0)),
            LEFT_ELBOW: _MockJoint((0.50, 1.25, 0.0)),
            RIGHT_ELBOW: _MockJoint((-0.50, 1.25, 0.0)),
            LEFT_WRIST: _MockJoint((0.80, 1.25, 0.0), quat_left),
            RIGHT_WRIST: _MockJoint((-0.80, 1.25, 0.0), quat_right),
            LEFT_HAND: _MockJoint((0.90, 1.25, 0.0)),
            RIGHT_HAND: _MockJoint((-0.90, 1.25, 0.0)),
        }
    )


def _make_bvh_rest_tpose_frame(
    left_wrist_quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    right_wrist_quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> MockFrame:
    """Reference-BVH rest frame with zero local rotations on both arm chains.

    The reference file's first frame has zero Hips, Arm, ForeArm, and Hand local
    rotations on both sides, so both wrist world rotations are identity.
    """
    return MockFrame(
        {
            PELVIS: _MockJoint((0.0, 0.85, 0.0)),
            SPINE1: _MockJoint((0.0, 0.95, 0.0)),
            SPINE2: _MockJoint((0.0, 1.05, 0.0)),
            SPINE3: _MockJoint((0.0, 1.25, 0.0)),
            NECK: _MockJoint((0.0, 1.35, 0.0)),
            HEAD: _MockJoint((0.0, 1.60, 0.0)),
            LEFT_COLLAR: _MockJoint((0.1, 1.25, 0.0)),
            RIGHT_COLLAR: _MockJoint((-0.1, 1.25, 0.0)),
            LEFT_SHOULDER: _MockJoint((0.20, 1.25, 0.0)),
            RIGHT_SHOULDER: _MockJoint((-0.20, 1.25, 0.0)),
            LEFT_ELBOW: _MockJoint((0.50, 1.25, 0.0)),
            RIGHT_ELBOW: _MockJoint((-0.50, 1.25, 0.0)),
            LEFT_WRIST: _MockJoint((0.80, 1.25, 0.0), left_wrist_quat),
            RIGHT_WRIST: _MockJoint((-0.80, 1.25, 0.0), right_wrist_quat),
            LEFT_HAND: _MockJoint((0.90, 1.25, 0.0)),
            RIGHT_HAND: _MockJoint((-0.90, 1.25, 0.0)),
        }
    )


def _make_left_arm_raised_frame() -> MockFrame:
    """Left arm raised ~90° (vertical), right arm still in T-pose."""
    quat_left = (0.0, 0.0, 0.707, 0.707)
    quat_right = (0.0, 0.0, -0.707, 0.707)
    return MockFrame(
        {
            PELVIS: _MockJoint((0.0, 0.85, 0.0)),
            SPINE1: _MockJoint((0.0, 0.95, 0.0)),
            SPINE2: _MockJoint((0.0, 1.05, 0.0)),
            SPINE3: _MockJoint((0.0, 1.25, 0.0)),
            NECK: _MockJoint((0.0, 1.35, 0.0)),
            HEAD: _MockJoint((0.0, 1.60, 0.0)),
            LEFT_COLLAR: _MockJoint((0.1, 1.25, 0.0)),
            RIGHT_COLLAR: _MockJoint((-0.1, 1.25, 0.0)),
            LEFT_SHOULDER: _MockJoint((0.20, 1.25, 0.0)),
            RIGHT_SHOULDER: _MockJoint((-0.20, 1.25, 0.0)),
            LEFT_ELBOW: _MockJoint((0.20, 1.55, 0.0)),  # elbow raised
            RIGHT_ELBOW: _MockJoint((-0.50, 1.25, 0.0)),
            LEFT_WRIST: _MockJoint((0.20, 1.85, 0.0), quat_left),  # wrist up
            RIGHT_WRIST: _MockJoint((-0.80, 1.25, 0.0), quat_right),
            LEFT_HAND: _MockJoint((0.20, 1.95, 0.0)),
            RIGHT_HAND: _MockJoint((-0.90, 1.25, 0.0)),
        }
    )


def _make_right_arm_raised_frame() -> MockFrame:
    """Right arm raised ~90° (vertical), left arm still in T-pose."""
    quat_left = (0.0, 0.0, 0.707, 0.707)
    quat_right = (0.0, 0.0, -0.707, 0.707)
    return MockFrame(
        {
            PELVIS: _MockJoint((0.0, 0.85, 0.0)),
            SPINE1: _MockJoint((0.0, 0.95, 0.0)),
            SPINE2: _MockJoint((0.0, 1.05, 0.0)),
            SPINE3: _MockJoint((0.0, 1.25, 0.0)),
            NECK: _MockJoint((0.0, 1.35, 0.0)),
            HEAD: _MockJoint((0.0, 1.60, 0.0)),
            LEFT_COLLAR: _MockJoint((0.1, 1.25, 0.0)),
            RIGHT_COLLAR: _MockJoint((-0.1, 1.25, 0.0)),
            LEFT_SHOULDER: _MockJoint((0.20, 1.25, 0.0)),
            RIGHT_SHOULDER: _MockJoint((-0.20, 1.25, 0.0)),
            LEFT_ELBOW: _MockJoint((0.50, 1.25, 0.0)),
            RIGHT_ELBOW: _MockJoint((-0.20, 1.55, 0.0)),  # elbow raised
            LEFT_WRIST: _MockJoint((0.80, 1.25, 0.0), quat_left),
            RIGHT_WRIST: _MockJoint((-0.20, 1.85, 0.0), quat_right),  # wrist up
            LEFT_HAND: _MockJoint((0.90, 1.25, 0.0)),
            RIGHT_HAND: _MockJoint((-0.20, 1.95, 0.0)),
        }
    )


# ---------------------------------------------------------------------------
# Pytest fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def tpose_frame() -> MockFrame:
    return _make_tpose_frame()


@pytest.fixture
def bvh_rest_tpose_frame() -> MockFrame:
    return _make_bvh_rest_tpose_frame()


@pytest.fixture
def bvh_tpose_frame_factory():
    return _make_bvh_rest_tpose_frame


@pytest.fixture
def left_raised_frame() -> MockFrame:
    return _make_left_arm_raised_frame()


@pytest.fixture
def right_raised_frame() -> MockFrame:
    return _make_right_arm_raised_frame()
