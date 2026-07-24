<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Noitom G1 Teleop

This example registers an external Isaac Lab task based on
`Isaac-PickPlace-Locomanipulation-G1-Abs-v0` and drives its upper-body action
pipeline from Noitom full-body mocap.

The `noitom_mocap` plugin reads Noitom Hybrid Data Server data through
MocapApi, converts each avatar update to the existing IsaacTeleop
`FullBodyPose` layout, and publishes it on tensor identifier `full_body`
inside the `noitom_mocap` OpenXR tensor collection. The Python task consumes
that stream through the generic `FullBodyTracker()` with the `body.noitom`
vendor selected. Plugin launch defaults, including the HDS endpoint, live in
[`plugin.yaml`](../../src/plugins/noitom_mocap/plugin.yaml). Before installing,
set its `--host` and `--port` arguments to the Hybrid Data Server TCP endpoint.

## Build

The Noitom SDK is not vendored in this repository. Build the optional plugin
when you need Noitom support:

```bash
cmake -B build -DBUILD_PLUGIN_NOITOM_MOCAP=ON
cmake --build build --target python_package noitom_mocap_plugin --parallel
cmake --install build
uv pip install --find-links=install/wheels "isaacteleop[cloudxr]"
```

Run `cmake --install build` again whenever `plugin.yaml` changes so the updated
configuration is copied to `install/plugins/noitom_mocap/plugin.yaml`.

By default CMake fetches MocapApi from `https://github.com/pnmocap/MocapApi`.
For offline builds, pass `-DNOITOM_MOCAP_API_ROOT=/path/to/MocapApi`.

## Example 1: Record And Replay

Record uses a Noitom wrapper around `TeleopSession` so the Noitom plugin can be
launched and consumed through the `noitom_mocap` tensor collection without
changing the shared MCAP examples. Replay uses the existing generic full-body
MCAP script because the recording uses the standard `full_body` channel.

![Noitom full-body recording](assets/record.gif)

Record the Noitom full-body stream:

```bash
uv run python examples/noitom/record_noitom_full_body.py \
  10 examples/noitom/recordings/noitom_full_body.mcap
```

The resulting MCAP uses the standard `core.FullBodyPoseRecord` schema and
the standard `full_body` channel, so the generic replay script can play it back:

![Noitom MCAP replay](assets/replay.gif)

```bash
cd examples/mcap_record_replay/python
uv sync
uv run python replay_full_body.py ../../noitom/recordings/noitom_full_body.mcap
```

## Example 2: Teleop

![Noitom G1 live teleoperation](assets/teleop.gif)

Run Isaac Lab with the external task registration callback. The task launches
`noitom_mocap_plugin` through IsaacTeleop's plugin manager by default.

```bash
cd ~/dependence/IsaacLab3-0
PYTHONPATH=~/IsaacTeleop/examples/noitom:$PYTHONPATH \
  ./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
  --task Isaac-PickPlace-Locomanipulation-G1-Noitom-Abs-v0 \
  --visualizer kit \
  --xr \
  --external_callback noitom_tasks.register_tasks
```

For advanced manual plugin control, start the plugin yourself and disable
auto-launch in the Isaac Lab terminal:

```bash
./install/plugins/noitom_mocap/noitom_mocap_plugin

NOITOM_MOCAP_AUTO_LAUNCH=0 \
PYTHONPATH=~/IsaacTeleop/examples/noitom:$PYTHONPATH \
  ./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
  --task Isaac-PickPlace-Locomanipulation-G1-Noitom-Abs-v0 \
  --visualizer kit \
  --xr \
  --external_callback noitom_tasks.register_tasks
```

If you run a dedicated CloudXR runtime yourself, source its environment before
starting Isaac Lab and pass Isaac Lab's flags for using the existing runtime
instead of auto-launching another one.

## Behavior

Retargeting lives in `noitom_retargeting.py` and is wired by
`noitom_tasks.py`. `NoitomG1Retargeter` orchestrates the per-arm target-state
nodes introduced by the architecture refactor. It maps Noitom shoulder, elbow,
and wrist bones into G1 Pink IK frame targets, with elbow and shoulder frame
tasks enabled by default.

The default motion path is driven by
[`ik_config/noitom_to_g1.json`](ik_config/noitom_to_g1.json). The validated
configuration declares the left/right human-joint to robot-link mappings, Pink
position/orientation weights, local target offsets, neutral wrist
post-rotations, and per-segment human-to-robot scale ratios. It intentionally
contains no leg or foot mappings because the Noitom task fixes the lower body.

The task is fixed-base upper-body teleoperation: the pelvis/root is physically
fixed, and the 12 hip, knee, and ankle joints retain the G1 articulation's
initial standing targets. The inherited Agile lower-body policy and its
observation group are disabled and are not loaded. The three waist joints are
not part of the fixed lower body and remain controlled by Pink IK.

Pipeline:

```text
FullBodyPose
  -> torso frame (pelvis, SPINE3, shoulders)
  -> calibrated human arm segments
  -> declarative mapping + shortest-arc segment solve
  -> robot-aligned BVH wrist axes + local offset + pose smoothing
  -> Pink IK frame-task action [wrists, elbows, shoulders, hands]
```

The default arm-frame action is 56D: six 7DoF frame poses followed by 14 hand
values. Wrist-only compatibility mode is 28D: two 7DoF wrist poses followed by
14 hand values. Neither layout has a locomotion suffix.

Calibration:

1. After teleop reset, retargeting clears its neutral reference.
2. Hold a stable upper-body pose; the next valid frame becomes the neutral
   reference.
3. Arm positions are applied relative to that neutral pose; `source` wrist
   orientation follows the current side-normalized semantic BVH wrist frame.

At calibration, each measured upper-arm and forearm length is multiplied by
the matching `human_scale_table` entry and clamped to the configured physical
range. Subsequent frames rotate each neutral segment toward its current
direction along the shortest arc and rebuild the robot target chain with those
calibrated lengths. Each JSON wrist rotation offset is a local xyzw
post-rotation in the BVH wrist frame, not a world pose or a VR-controller target
offset. This BVH mirrors its hand bones: the left raw X axis points from fingers
to palm while the right raw X axis points from palm to fingers. The release
proper rotation, not a reflection.
configuration normalizes both to G1 anatomy with the field-selected left
local-Z 180-degree rotation `[0, 0, 1, 0]` and a right identity rotation
`[0, 0, 0, 1]`. The left offset flips X and Y together while preserving Z and
determinant `+1`; it is a proper rotation, not a reflection.
proper rotation, not a reflection.

Override the mapping file for tuning with `NOITOM_IK_CONFIG=/path/to/config.json`.
The file is validated at startup, so missing links, joints, scales, malformed
offsets, or negative weights fail with a direct error instead of silently
falling back to constants.

Wrist orientation defaults to `source`: the raw BVH wrist world frame is placed
in the same G1-facing alignment used by the cyan reference skeleton, then the
side-specific local offset is post-multiplied to form the semantic wrist frame
and the pose is smoothed. The semantic target and reference axes correspond as
`X=red`, `Y=green`, and `Z=blue`. Pink's orientation cost defaults to 0.75.
Compatibility and diagnostic modes retain the previous `forearm`, `twist`, and
`full` mappings.

The viewport also shows left/right wrist coordinate frames by default. The
axes follow `X=red`, `Y=green`, `Z=blue`:

- Thin line axes at the cyan wrists are the side-normalized semantic BVH/Noitom
  wrist frames that Pink should match, after placing the source skeleton in the
  G1-facing world frame.
- Frame-marker axes attached to the robot are the actual G1
  `left_wrist_yaw_link` and `right_wrist_yaw_link` frames.

Disable both wrist-frame displays with `NOITOM_DRAW_WRIST_FRAMES=0`.

### Wrist-orientation diagnostics

Enable low-rate source/target and Pink target/actual wrist rotation logs:

```bash
NOITOM_ORIENTATION_DEBUG=1 \
PYTHONPATH=~/IsaacTeleop/examples/noitom:$PYTHONPATH \
  ./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
  --task Isaac-PickPlace-Locomanipulation-G1-Noitom-Abs-v0 \
  --visualizer kit --xr --external_callback noitom_tasks.register_tasks
```

`NoitomOrientationDebug` compares world-frame and torso-relative Noitom wrist
rotation deltas and prints `forearm_swing_deg` plus raw and bounded residual
twist for compatibility-mode diagnosis. `source_target_error_deg` is the
rotation error from the semantic BVH wrist frame to the smoothed wrist target;
it may lag during motion and should converge below one degree after the source
stops. A small source-target error with a large `NoitomPinkOrientationDebug`
error isolates the problem to Pink IK, smoothing, or joint limits.
`NoitomPinkOrientationDebug` compares the Pink target with the simulated G1
wrist-link orientation. `NoitomPinkJointDebug` prints G1 wrist roll/pitch/yaw
actual and IK-target positions, distance to the active safety boundary, and
`solver_held=1` when Pink returned the current positions unchanged. With debug
enabled, Pink also prints the underlying QP exception. `reference_q` is the
quaternion used by the thin BVH viewport axes. Quaternions use `(x, y, z, w)`.

The same switch also emits synchronized 7DoF `NoitomWristPoseDebug` and
`NoitomWristPoseError` records. The first valid source sample records the raw
BVH pose after Y-up-to-Z-up axis conversion, the robot-aligned raw wrist frame,
the side-normalized semantic frame, and the retargeting world target. Its
`NoitomWristAxisDebug` record reports raw/semantic X-axis alignment with the
elbow-to-wrist direction, all three semantic world axes, and the local offset.
Periodic `solve_cycle` records then show
the exact Pink world input, the pelvis-local `LocalFrameTask` target, FK of the
final safety-clipped Pink joint solution, and the simulated wrist pose before
that solution is applied. Positions use meters, rotations use normalized
`quat_xyzw`, and arrays print six decimal places.

`bvh_raw_isaac_world` still uses the BVH origin and has not received the
operator-facing or robot-pelvis placement, so its position must not be directly
subtracted from a G1 world pose. `bvh_aligned_raw_world` preserves the source
frame fact, while `bvh_semantic_world` applies the configured anatomical
normalization once and is the viewport/Pink reference. `bvh_semantic_world`,
`pink_input_world`, `pink_solution_fk_world`, and
`robot_actual_pre_apply_world` share the Isaac environment-world frame and can
be compared directly. The corresponding `*_pelvis` stages audit Pink's
local-frame conversion.

Select the orientation mapping without editing Python:

```bash
# Recommended/default: match side-normalized semantic BVH wrist axes.
NOITOM_WRIST_ORIENTATION_MODE=source \
NOITOM_WRIST_ORIENTATION_COST=0.75 ...

# Compatibility: shortest-arc forearm transport plus residual twist.
NOITOM_WRIST_ORIENTATION_MODE=twist \
NOITOM_WRIST_TWIST_LIMIT_DEG=60 \
NOITOM_WRIST_TWIST_MAX_STEP_DEG=4 \
NOITOM_WRIST_ORIENTATION_COST=0.75 ...

# Diagnostic: forearm transport only; its fixed roll can conflict with G1 IK.
NOITOM_WRIST_ORIENTATION_MODE=forearm ...

# Diagnostic only: old full 3-axis Noitom wrist delta.
NOITOM_WRIST_ORIENTATION_MODE=full ...
```

For an A/B run that keeps orientation generation and logging enabled but removes
its influence on Pink IK, add `NOITOM_WRIST_ORIENTATION_COST=0`. Any nonnegative
value can be supplied to sweep the cost without editing Python.

The Noitom task alone constrains wrist pitch/yaw inside Pink and clips its final
joint targets to the same boundary. Other Isaac Lab tasks are unchanged. The
defaults are `+/-80` degrees and can be adjusted together with the Noitom Pink
response tuning:

```bash
NOITOM_WRIST_PITCH_LIMIT_DEG=80 \
NOITOM_WRIST_YAW_LIMIT_DEG=80 \
NOITOM_PINK_TASK_GAIN=0.20 \
NOITOM_PINK_LM_DAMPING=50 ...
```

For collision-free arm-swing diagnosis, remove the packing table and park its
required task object out of the workspace with gravity and object termination
conditions disabled:

```bash
NOITOM_CLEAR_WORKSPACE=1 ...
```

Alternatively, shift the fixed G1 robot and all Noitom reference/IK anchors by a
world-frame XYZ offset in meters:

```bash
NOITOM_ROBOT_OFFSET=0,-0.5,0 ...
```

Default settings live in `NoitomG1Settings` and
`NoitomRetargetingSettings`. When Kit visualization is enabled, the incoming
Noitom pose is shown as a cyan stick figure anchored to the robot pelvis.
