<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Noitom example — agent notes

- Pure-Python tests run after `source ~/env_isaaclab/bin/activate` in the Isaac Lab
  development workspace. Set `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` when invoking pytest;
  otherwise ROS `launch_testing` entry points from the system Python path may be loaded
  into the Isaac Lab interpreter.
- Run tests against Python bindings built from the current worktree; an older installed
  `isaacteleop` package may not expose the generic full-body tracker API used by main.
- Tests that instantiate the task config without exercising plugin launch must set
  `NOITOM_MOCAP_AUTO_LAUNCH=0` so they do not depend on an installed plugin tree.
- When `tests/conftest.py` inserts the example directory into `sys.path`, mark only
  the necessarily delayed imports with `# noqa: E402`. Import names used by test
  type annotations at module scope so Ruff can resolve them.
- Keep `README.md` and `README_CH.md` behavior, configuration, and command examples
  synchronized in the same change.
- When planning/documentation and implementation are explicitly assigned to separate
  agents, the documentation agent must not modify implementation or configuration files.
- BVH wrist-frame targets and their debug visualization must use the same coordinate
  transform. Do not reuse a VR-controller target offset from a pipeline that disables
  source wrist rotation as a BVH local-frame offset.
- Treat zero error between a generated wrist reference and its Pink input as a pipeline
  consistency check, not proof of anatomical axis alignment. Before tuning IK weights or
  limits for a left/right asymmetry, verify each side's three basis vectors against the
  forearm and palm semantics; mirrored BVH hand bones may require a side-specific proper
  rotation even when both source joint rotations are numerically identical.
- If the sandbox mounts the default pre-commit cache read-only, copy that cache to a
  task-specific directory under `/tmp`, rewrite the copied `db.db` repository paths to
  that directory, and set `PRE_COMMIT_HOME` for the required run; copying alone leaves
  absolute paths to the read-only cache. Do not skip the repository check.
- After adding multiline diagnostic or test code, run the repository Ruff-format hook
  and rerun the full pre-commit suite if it rewrites a file; a formatter rewrite is not
  a completed validation pass.
