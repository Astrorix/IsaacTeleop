<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# 站立垂臂腕部同步 7DoF 诊断实施计划

**状态**：实现和自动化验证完成，现场站立验收待执行

**执行者**：另一个 coding agent

**背景决策**：
[`../decisions/2026-07-24-standing-left-wrist-ik-limit-diagnosis.md`](../decisions/2026-07-24-standing-left-wrist-ik-limit-diagnosis.md)

## 1. 目标

在不改变 Noitom 重定向和 Pink 控制行为的前提下，为站立垂臂测试增加可同步比较的
左右腕 7DoF 位姿日志，明确区分：

```text
第一有效 BVH 腕位姿
  -> 视口 robot-aligned BVH 腕位姿
  -> Pink world action
  -> Pink pelvis-local task target
  -> Pink 最终关节解的 FK 腕位姿
  -> 仿真机器人实际 wrist_yaw_link 位姿
```

最终日志必须能够判断左腕约 90 度外观来自源映射、Pink 受限解还是仿真实际跟随。

## 2. 范围与禁止项

预计修改：

- `examples/noitom/noitom_retargeting.py`
- `examples/noitom/noitom_tasks.py`
- `examples/noitom/tests/` 下相关纯 Python 单元测试
- 只有在需要公开新诊断行为时才更新 `README.md` 和 `README_CH.md`；若更新其中
  一个，必须在同一变更中同步另一个。

本阶段禁止：

- 修改 `ik_config/noitom_to_g1.json` 的任何 offset、cost 或映射；
- 修改 `source` 腕朝向算法、平滑参数、Pink task cost/gain/damping；
- 修改 `NOITOM_WRIST_PITCH_LIMIT_DEG` 或 `NOITOM_WRIST_YAW_LIMIT_DEG` 默认值；
- 为左手增加临时 90 度补偿或左右镜像；
- 修改 Isaac Lab 上游源码、`src/core/**` 或 Noitom C++ 插件；
- 提交 `/tmp` 日志、截图或完整 BVH 数据。

## 3. 执行前检查

1. 从 `/home/lenovo/IsaacTeleop` 列出所有 `AGENTS.md`。
2. 阅读根 `AGENTS.md` 和 `examples/noitom/AGENTS.md`。
3. 检查 dirty worktree，保留其他 agent 和用户的已有改动；禁止用 checkout/reset 或
   整文件替换清理工作树。
4. 阅读本计划、背景决策、现有
   [`BVH_WRIST_AXIS_ALIGNMENT_PLAN.md`](BVH_WRIST_AXIS_ALIGNMENT_PLAN.md)、相关代码和
   测试后再编辑。

## 4. 坐标系和数据定义

全部日志位置单位为米，四元数统一使用 `(x, y, z, w)`，字段名必须写成
`quat_xyzw`，不能只写含糊的 `quat`。

### 4.1 `bvh_raw_isaac_world`

- 来源：左右 `BodyJoint.LEFT_WRIST`/`RIGHT_WRIST` 的有效 joint pose。
- 位置使用 `noitom_position_to_isaac()`；旋转使用
  `noitom_quaternion_to_isaac()`。
- 这是 Noitom/BVH 原点下、轴已经转换为 Isaac Z-up 的源世界位姿。
- 它没有应用 operator-facing 旋转、robot pelvis anchor、人体到机器人长度映射，
  因此不能直接与 G1 世界位置数值相减。

### 4.2 `bvh_aligned_world`

- 必须表示视口青色腕坐标轴实际使用的位姿。
- 方向使用 `reference_wrist_frames()` 的共享 `_aligned_source_wrist_rotation()` 结果。
- 在启用当前 config-driven reference skeleton 时，视口腕轴位置来自
  `reference_skeleton_positions()` 覆盖后的腕目标位置；日志必须复用同一路径，不能
  另写一套近似位置转换。
- 这是可以与 `pink_input_world` 逐轴、逐位置比较的 environment-world 位姿。

### 4.3 `pink_input_world`

- 使用 `PinkInverseKinematicsAction.process_actions()` 实际收到的 action：左腕
  `actions[:, 0:7]`，右腕 `actions[:, 7:14]`。
- 这是 environment-world 位姿，不能用 retargeter 的预期值替代。

### 4.4 `pink_target_pelvis`

- 使用 `_transform_poses_to_base_link_frame()` 的真实输出，或从已经写入对应
  `LocalFrameTask.transform_target_to_base` 的 `pin.SE3` 读取。
- 必须记录 Pink 本轮求解真正使用的 pelvis-local target，而不是重新用另一套数学
  函数估算。

### 4.5 `pink_solution_fk_world`

- 使用本轮 Pink 返回并经过 Noitom wrist/waist 安全裁剪后的最终 joint target。
- 将当前完整 Isaac Lab joint vector 复制一份，只替换
  `_isaaclab_controlled_joint_ids` 对应的最终解；不要把只含受控关节的数组直接当成
  Pinocchio 完整配置。
- 按 `ik_controller.isaac_lab_to_pink_ordering` 转为 Pinocchio 顺序。
- 在 Pink kinematics configuration 上对该临时配置执行 FK，读取左右完整 URDF frame
  名称相对于 `_PINK_PELVIS_LINK` 的变换，再与本轮
  `base_link_frame_in_world_rf` 复合为 environment-world 位姿。
- 诊断 FK 不得改变下一轮求解状态：在 `try/finally` 中保存当前完整配置并恢复，或
  使用独立数据/配置副本。不能因打印日志改变控制结果。

完整 Pink frame 名称必须取 controller task 配置中的 frame，不要把 Isaac asset 的
短 link 名直接传给 Pinocchio。

### 4.6 `robot_actual_world` 和 `robot_actual_pelvis`

- 来源：Isaac asset `body_link_pose_w` 中实际的
  `left_wrist_yaw_link`/`right_wrist_yaw_link`。
- 世界位置减去对应 `env_origin`，得到与 action 一致的 environment-world frame；
  单环境原点通常为零，但实现不能依赖这个巧合。
- 使用与 Pink 相同的 `base_link_frame_in_world_rf` 转换出 pelvis-local 实际位姿。
- 这是物理仿真的当前 link pose，不是 Pink task target，也不是 IK joint target 的 FK。

## 5. 实现步骤

### 5.1 源层第一有效帧

在 `NoitomG1ActionSource` 增加独立状态，例如
`_first_valid_wrist_pose_printed`：

- 仅在 `frame is not None` 且左右腕 pose 都有效时置位；
- `context.execution_events.reset` 时恢复为 `False`；
- 不复用当前只说明 tracker 是否有数据的 `_first_frame_printed`，因为该标志可能在
  `frame=None` 时已经置位；
- 在校准/retarget 已为同一有效帧生成目标后打印，确保 raw、aligned 和 action target
  来自同一 source frame；
- 左右手都打印 `bvh_raw_isaac_world`、`bvh_aligned_world` 和
  `retarget_target_world`。

建议在 `NoitomG1Retargeter` 增加只读的公共诊断方法返回 raw 和 aligned 腕
`SE3Pose`，避免 `noitom_tasks.py` 导入 `_joint_pose` 等私有实现。该方法只能复用现有
转换函数，不能引入第二套坐标公式。

### 5.2 捕获 Pink 本轮真实目标

在 Noitom 专用 `NoitomPinkInverseKinematicsAction` 中：

1. `process_actions()` 调用父类后，保存左右腕的 `pink_input_world` 副本。
2. 从父类刚完成的 transformed poses/LocalFrameTask target 保存
   `pink_target_pelvis` 副本。
3. 增加单调递增的 `solve_cycle`。目标、FK 解和实际 link pose 必须使用同一个
   `solve_cycle` 标签。
4. 不要在 `process_actions()` 中把上一轮 `_last_ik_solution` 当成本轮结果打印。
   求解结果日志应在本轮 `_compute_ik_solutions()` 完成安全裁剪之后生成。

### 5.3 解后 FK 和实际位姿

扩展当前 Noitom action 的 `_compute_ik_solutions()`：

1. 保留现有 current joint、Pink solution、安全裁剪和 joint debug 行为。
2. 在最终 `sol` 确定后，对每个 environment 计算 `pink_solution_fk_world`。
3. 同一打印周期读取 `robot_actual_world`，并将 FK 与实际位姿都转换到 pelvis-local。
4. 当前 action hook 中的实际 pose 是本次 joint target 应用前的物理状态，日志中可
   明确命名为 `robot_actual_pre_apply_world`；稳定站立时它应与低速
   `robot_actual_world` 等价。不要宣称它是同一物理 step 应用后的状态。
5. 默认继续使用现有 `print_period_s` 节流，避免逐物理帧刷屏；第一有效 BVH 帧
   不受节流影响，只打印一次。

如果不适合在 `_compute_ik_solutions()` 内做打印，可抽取只读 helper，但必须保持
“目标已经设置、当前轮求解和安全裁剪已经完成、尚未被下一轮覆盖”的时序。

### 5.4 误差

每侧至少计算：

- 第一有效 source frame：`aligned_to_retarget_pos_m`、
  `aligned_to_retarget_rot_deg`
- `input_to_solution_pos_m`、`input_to_solution_rot_deg`
- `solution_to_actual_pos_m`、`solution_to_actual_rot_deg`
- `input_to_actual_pos_m`、`input_to_actual_rot_deg`

旋转距离使用归一化四元数和 `abs(dot(q1, q2))`，保证 `q` 与 `-q` 等价。世界系
误差只比较同为 environment-world 的位姿；pelvis-local 值用于核对 Pink frame
转换，禁止跨坐标系直接求差。

## 6. 固定日志格式

第一有效源帧：

```text
NoitomWristPoseDebug sample=first_valid source_frame=123 stage=bvh_raw_isaac_world side=left pos_m=[...] quat_xyzw=[...]
NoitomWristPoseDebug sample=first_valid source_frame=123 stage=bvh_aligned_world side=left pos_m=[...] quat_xyzw=[...]
NoitomWristPoseDebug sample=first_valid source_frame=123 stage=retarget_target_world side=left pos_m=[...] quat_xyzw=[...]
NoitomWristPoseError sample=first_valid source_frame=123 side=left aligned_to_retarget_pos_m=... aligned_to_retarget_rot_deg=...
```

Pink 稳态周期：

```text
NoitomWristPoseDebug solve_cycle=456 stage=pink_input_world side=left pos_m=[...] quat_xyzw=[...]
NoitomWristPoseDebug solve_cycle=456 stage=pink_target_pelvis side=left pos_m=[...] quat_xyzw=[...]
NoitomWristPoseDebug solve_cycle=456 stage=pink_solution_fk_world side=left pos_m=[...] quat_xyzw=[...]
NoitomWristPoseDebug solve_cycle=456 stage=pink_solution_fk_pelvis side=left pos_m=[...] quat_xyzw=[...]
NoitomWristPoseDebug solve_cycle=456 stage=robot_actual_pre_apply_world side=left pos_m=[...] quat_xyzw=[...]
NoitomWristPoseDebug solve_cycle=456 stage=robot_actual_pelvis side=left pos_m=[...] quat_xyzw=[...]
NoitomWristPoseError solve_cycle=456 side=left input_to_solution_pos_m=... input_to_solution_rot_deg=... solution_to_actual_pos_m=... solution_to_actual_rot_deg=... input_to_actual_pos_m=... input_to_actual_rot_deg=...
```

右手格式完全相同。数组至少打印 6 位小数，避免毫米级位置误差和小角度误差被
当前 3 位格式吞掉。

若 source 和 Pink 对象之间不适合共享 `source_frame`，不要用全局可变状态把它们
强行关联。第一帧用 `source_frame`，Pink 用 `solve_cycle`；静态站立时通过
`retarget_target_world` 与 `pink_input_world` 的相同 7DoF 数值建立对应。

## 7. 自动化测试

至少增加以下回归测试：

1. raw wrist diagnostic 使用现有 Y-up 到 Z-up 位置和旋转转换，输出归一化 xyzw。
2. aligned wrist diagnostic 与 `reference_wrist_frames()` 使用完全相同的旋转；在
   config-driven reference 下，位置与实际视口腕轴位置相同。
3. 第一有效帧只有左右腕均有效时才打印；无数据或单腕无效不能消耗一次性标志。
4. reset 后第一有效帧可以再次打印。
5. pose/rotation error 对 `q` 和 `-q` 返回零。
6. environment origin 非零时，actual world position 正确减去对应 origin。
7. Pink target 从 world 转为 pelvis-local 后，再复合 pelvis world 能恢复原 pose。
8. 解后 FK helper 恢复 Pink configuration，不改变后续求解使用的当前配置。
9. 诊断关闭时不新增输出、不执行额外 FK，默认控制数值与修改前一致。

不要为测试引入完整参考 BVH、Isaac GUI 或真实 Noitom 数据流。优先测试抽出的纯函数
和已有轻量 fixture。

## 8. README 和代码注释

- 如果新增日志只扩展已有 `NOITOM_ORIENTATION_DEBUG=1`，可以在中英文 README 的
  现有腕部诊断章节各补充同等内容；只要修改一个 README，就必须同步另一个。
- README 必须明确 raw BVH Isaac-axis world 与 robot-aligned environment world 的
  原点/朝向不同，只有后者能直接和 Pink world action 比较。
- 对 Pinocchio 临时 FK 的保存/恢复写一条短源码注释，说明诊断不得改变控制器状态。
- 不把本次具体四元数或 `/tmp` 日志数值写成永久源码注释；这些留在背景 decision。

## 9. 实施者测试命令

从仓库根运行：

```bash
cd /home/lenovo/IsaacTeleop
source /home/lenovo/env_isaaclab/bin/activate

VALIDATION_DIR=/tmp/noitom-wrist-axis-validation
mkdir -p "$VALIDATION_DIR"
set -o pipefail

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python -m pytest examples/noitom/tests -vv \
  2>&1 | tee "$VALIDATION_DIR/standing-pose-debug-pytest.log"

git diff --check \
  2>&1 | tee "$VALIDATION_DIR/standing-pose-debug-diff-check.log"

SKIP=check-copyright-year \
pre-commit run --all-files \
  2>&1 | tee "$VALIDATION_DIR/standing-pose-debug-pre-commit.log"
```

预期：全部 Noitom 测试、diff check 和完整 pre-commit 通过。若 README 有变化，中
英文内容、命令、环境变量和日志字段必须一致。

## 10. 实施后站立测试命令

继续使用当前 `source` 模式、0.75 orientation cost 和默认腕限位，不在本轮做参数
A/B：

```bash
VALIDATION_DIR=/tmp/noitom-wrist-axis-validation
mkdir -p "$VALIDATION_DIR"
cd /home/lenovo/IsaacLab
set -o pipefail

PYTHONUNBUFFERED=1 \
NOITOM_ORIENTATION_DEBUG=1 \
NOITOM_CLEAR_WORKSPACE=1 \
NOITOM_WRIST_ORIENTATION_MODE=source \
NOITOM_WRIST_ORIENTATION_COST=0.75 \
PYTHONPATH=/home/lenovo/IsaacTeleop/examples/noitom:${PYTHONPATH:-} \
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
  --task Isaac-PickPlace-Locomanipulation-G1-Noitom-Abs-v0 \
  --visualizer kit \
  --xr \
  --external_callback noitom_tasks.register_tasks \
  2>&1 | tee "$VALIDATION_DIR/standing-pose-trace.log"
```

若广播器支持冻结或重复单帧，重复第一有效站立帧至少 5 秒；否则完整广播站立动作，
在动作停止变化后继续保持至少 5 秒再退出。

提取摘要：

```bash
rg \
  'NoitomWristPoseDebug|NoitomWristPoseError|NoitomOrientationDebug|NoitomPinkOrientationDebug|NoitomPinkJointDebug|QP|Traceback' \
  /tmp/noitom-wrist-axis-validation/standing-pose-trace.log \
  | tee /tmp/noitom-wrist-axis-validation/standing-pose-trace-summary.log
```

## 11. 预期结果和判读

### 正常的数据关系

- 第一帧 `bvh_raw_isaac_world` 与 `bvh_aligned_world` 可以不同，这是预期的对齐和
  锚定结果。
- 使用单位 wrist local offset 且平滑稳定后，`bvh_aligned_world`、
  `retarget_target_world` 和 `pink_input_world` 的旋转误差应小于 1 度。
- `pink_input_world` 与 `pink_target_pelvis` 经 pelvis transform 往返后的误差应接近
  数值精度。
- 右手 `pink_input_world` 到 `robot_actual_world` 旋转误差应继续保持约 1 度量级，
  且不触及 wrist pitch/yaw 边界。

### 左手重点

1. 若 `input_to_solution_rot_deg` 仍约 69 度，且 pitch/yaw margin 仍接近零：确认问题
   在 Pink 受限解、局部求解分支或任务可达性，不修改 BVH offset。
2. 若 solution FK 接近目标而 `solution_to_actual_rot_deg` 很大：检查关节 target 应用、
   actuator 和物理跟随。
3. 若 `aligned_to_retarget_rot_deg` 已经很大，或稳定段
   `retarget_target_world` 与 `pink_input_world` 数值不一致：回查 action 构造、姿态
   平滑和坐标顺序；此时才是 Pink 之前的问题。
4. 若第一帧可达、随后约 35 度源腕变化后才触限：记录首次触限 cycle，下一阶段围绕
   轨迹、局部解和腕奇异区设计 A/B，不把动态问题误写成固定 offset。

## 12. 完成条件

- 所有要求的 stage 都同时覆盖 left/right，并明确 frame 和 `quat_xyzw`。
- 第一有效 source frame 和周期性 Pink solve 日志满足固定格式。
- 诊断关闭时不产生额外计算或控制行为变化。
- 新旧自动化测试、`git diff --check` 和完整 pre-commit 全部通过。
- 若修改 README，`README.md` 与 `README_CH.md` 已同步。
- 实施者在交付中明确列出修改文件、测试结果，并把第 10 节命令留给用户运行；不能
  把未执行的现场站立验收写成已通过。

## 13. 执行结果（2026-07-24）

- 已增加第一有效源帧的 raw、robot-aligned 和 retarget target 左右腕 7DoF 日志；
  只有双腕均有效且本帧已经完成校准/重定向时才消耗一次性标志，reset 后重新启用。
- Pink 日志已改为在同一 `solve_cycle` 完成安全裁剪后采集真实 world input、真实
  pelvis-local task target、最终解 FK，以及减去对应 `env_origin` 的 apply 前实际腕
  位姿。
- 解后 FK 使用 controller 配置中的完整 URDF frame 名称，并通过 `try/finally` 恢复
  Pink configuration；诊断关闭或未到打印周期时不执行额外 FK。
- 已增加 `q/-q` 等价误差、坐标往返、非零环境原点、双腕有效门控/reset、raw/aligned
  路径复用和 FK 状态恢复测试；中英文 README 已同步。
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest examples/noitom/tests -vv`：
  `46 passed`。
- `git diff --check` 和 `SKIP=check-copyright-year pre-commit run --all-files`：
  通过。日志保存在 `/tmp/noitom-wrist-axis-validation/standing-pose-debug-*.log`。
- 第 10 节 Noitom HDS + Isaac Lab 现场站立测试尚未执行。
