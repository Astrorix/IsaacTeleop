# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Public re-export shim for Noitom G1 arm IK target node classes.

Phase A refactoring: ``NoitomArmIkTargetNode`` lives in ``noitom_retargeting``
(its natural home, since it uses the private helpers defined there) and is
re-exported here so downstream code can import from this module without
creating a circular-import between ``noitom_retargeting`` and a separate
nodes file.

Usage::

    from noitom_retargeter_nodes import NoitomArmIkTargetNode

Or equivalently::

    from noitom_retargeting import NoitomArmIkTargetNode
"""

from noitom_retargeting import NoitomArmIkTargetNode  # noqa: F401

__all__ = ["NoitomArmIkTargetNode"]
