<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Noitom G1 固定下半身实施计划

**状态**：实现和自动化验证完成，现场站立验收待执行

**范围**：只改 `examples/noitom/`

**背景决策**：

- [`../decisions/2026-07-24-noitom-fixed-lower-body.md`](../decisions/2026-07-24-noitom-fixed-lower-body.md)
- [`../decisions/2026-07-24-left-bvh-wrist-semantic-frame-correction.md`](../decisions/2026-07-24-left-bvh-wrist-semantic-frame-correction.md)

## 1. 目标与边界

把 Noitom G1 任务改为稳定的固定下半身站立模式：

- pelvis/root 继续固定在初始世界位姿；
- 左右共 12 个髋、膝、踝关节保持 G1 初始关节姿态，不再运行 Agile locomotion
  policy，也不再接收速度和 hip-height action；
- 腰关节不属于本计划的“下半身固定”范围，继续由当前 Pink IK 使用；
- 双臂、双腕和双手控制保持现状；
- 不改变已经现场基本对齐的 BVH 腕坐标系，不调整 Pink 权重、腕限位、骨长、平滑
  或世界锚点。

固定腿关节集合必须精确使用现有 lower-body action 的三组表达式：

```text
.*_hip_.*_joint
.*_knee_joint
.*_ankle_.*_joint
```

不要把 `waist_*`、手臂、手腕、手指或头部关节加入该集合。

## 2. 当前问题

`NoitomLocomanipulationG1EnvCfg.__post_init__()` 已经设置：

```python
self.scene.robot.spawn.articulation_props.fix_root_link = True
```

所以骨盆不会自由移动。但它继承的 `LocomanipulationG1EnvCfg` 仍包含
`AgileBasedLowerBodyActionCfg`：

- 控制髋、膝和踝；
- 加载 `agile_locomotion.pt`；
- 消费 `lower_body_policy` observation group；
- 当前 Noitom action 最后 4 维仍发送 `[vx, vy, wz, hip_height]`，站立时为
  `[0, 0, 0, 0.72]`。

因此当前实现只是“固定 root + 仍由 Agile policy 调腿”，并没有固定腿部。这也是
已有注释所说 policy 会下蹲、破坏腕部 IK 可达性的来源。

Isaac Lab 已有
`fixed_base_upper_body_ik_g1_env_cfg.py` 作为直接设计先例：固定 root，只保留
`upper_body_ik` action，不创建 lower-body policy action 或 observation。Noitom 实现
应遵循这一最小模式，但不要整体更换父 EnvCfg，因为那会连带改变场景、终止项和当前
Noitom 定制行为。

## 3. 强制执行前检查

1. 从仓库根运行 `find . -name AGENTS.md ! -path '*/.git/*' | sort`。
2. 阅读仓库根 `AGENTS.md` 和 `examples/noitom/AGENTS.md`。
3. 检查 dirty worktree，保留用户和其他 agent 的修改，禁止 checkout/reset 或整文件
   覆盖。
4. 阅读本计划、背景决策、`noitom_tasks.py`、相关测试，以及 Isaac Lab 的：
   - `locomanipulation_g1_env_cfg.py`；
   - `fixed_base_upper_body_ik_g1_env_cfg.py`；
   - `isaaclab/managers/action_manager.py`。
5. 当前 `ik_config/noitom_to_g1.json` 左腕 offset 是现场选定的
   `[0.0, 0, 1.0, 0.0]`。先把仍断言 `[0,1,0,0]` 的测试和旧 JSON 注释同步为
   Z 轴 180 度语义；不得恢复早期 Y 轴候选。至少检查
   `tests/test_retargeting_config.py` 中的配置和三轴参数化断言。

## 4. 推荐实现：删除 Noitom 任务中的 Agile 腿部 action

在 `NoitomLocomanipulationG1EnvCfg.__post_init__()` 中，在环境 manager 实例化前明确：

```python
self.scene.robot.spawn.articulation_props.fix_root_link = True
self.actions.lower_body_joint_pos = None
self.observations.lower_body_policy = None
```

实现语义：

1. root 使用现有的物理 fixed-root 配置。
2. `ActionManager` 会跳过值为 `None` 的 action term，因此不加载 Agile policy，也不再
   控制 12 个腿关节。
3. observation manager 同样跳过 `None` group，避免计算仅供 Agile policy 使用的
   observation。
4. 腿部保持 G1 articulation 的初始姿态：hip pitch `-0.10 rad`、knee
   `+0.30 rad`、ankle pitch `-0.20 rad`，其余匹配的 hip/ankle 轴为默认 `0`。
5. 腰关节仍留在 Pink controlled joint 集合，不得锁定或从 Pink 移除。

优先采用上述 Isaac Lab 固定基座任务已经使用的“无 lower-body action”方案，不新增
本地 locomotion policy、不修改 Isaac Lab 上游文件，也不要用更高 Pink cost 抵消腿部
运动。

如果运行时证据显示删除 action 后腿关节没有维持 articulation 初始 target，而是在
重力下持续漂移，才允许采用后备方案：把 inherited term 替换为只覆盖上述 12 个关节、
`scale=0.0`、`use_default_offset=True` 的 `JointPositionActionCfg`，并由 Noitom source
持续输出 12 个零 action。采用后备方案会改变下述 action 维度和测试，必须在 README
与执行结果中说明原因和实测证据；不得未经实测直接增加这层复杂度。

## 5. 同步修改 action layout

主方案删除 Agile term 后，teleop action 不能继续保留最后 4 个 locomotion 值。统一
修改 `noitom_tasks.py` 中所有 action 维度和构造路径：

| 模式 | 当前维度 | 固定下半身维度 | 内容 |
| --- | ---: | ---: | --- |
| wrist + elbow + shoulder frame tasks | 60 | 56 | 42 pose + 14 hand |
| wrist-only compatibility mode | 32 | 28 | 14 wrist pose + 14 hand |

具体要求：

- 把 `_G1_ACTION_DIM_WITH_ARM_IK` 改为 `56`，`_G1_ACTION_DIM_WRIST_ONLY` 改为
  `28`，名称/注释应体现已无 locomotion suffix；
- `g1_action_dim()`、`G1LocomanipulationAction()`、`NoitomG1ActionSource.output_spec()`
  和 pipeline 输出必须继续共享同一个维度来源；
- `_make_action()` 只填写腕、可选肘肩和 14 维 hand action，删除对 `action[-4:]` 的
  locomotion 写入；
- 删除不再使用的 `_LOCOMOTION_DEFAULT_HIP_HEIGHT`；
- 不改变 `_ROBOT_PELVIS_ANCHOR=(0,0,0.72)`。它是 retargeting/可视化锚点，不是
  已删除的 lower-body hip-height action；本计划不要顺带调整。

不要保留“未被 manager 消费的 4 个尾部元素”。pipeline 输出 tensor 与 Isaac Lab
ActionManager 维度必须严格相同，否则运行时会在 action 分配阶段失败。

## 6. 启动日志和可选诊断

环境配置完成时打印一次清晰摘要，例如：

```text
NoitomG1Env: lower_body=fixed root_fixed=1 leg_action=disabled action_dim=56
```

日志必须足以确认当前运行没有使用 Agile policy。不要每帧打印完整 12 关节向量。

如果已有调试结构允许以很小改动低频读取机器人状态，可在
`NOITOM_ORIENTATION_DEBUG=1` 时按现有 `NOITOM_PRINT_PERIOD_S` 输出：

```text
NoitomLowerBodyDebug max_leg_pos_error_rad=... max_leg_vel_rad_s=...
```

误差相对于 articulation 初始腿部关节位置计算。该低频诊断是推荐项，不得为此引入
新的 manager/action 依赖或把腿关节加入 Pink。

## 7. 测试要求

扩展现有 `examples/noitom/tests/`，至少覆盖：

1. `g1_action_dim(use_arm_ik_frame_tasks=True) == 56`。
2. `g1_action_dim(use_arm_ik_frame_tasks=False) == 28`。
3. arm-frame 模式 `_make_action()` 的布局精确为 42 pose + 14 hand，无 4 维
   locomotion suffix。
4. wrist-only 模式布局精确为 14 wrist pose + 14 hand。
5. `NoitomLocomanipulationG1EnvCfg` 配置中 root fixed、
   `lower_body_joint_pos is None`、`lower_body_policy is None`。
6. Pink controlled joints 不包含任何 hip/knee/ankle，仍包含当前 waist joints；腿部固定
   不能改变 Pink task 数量、权重或关节限位。
7. current wrist semantic config 测试统一断言左腕 `[0,0,1,0]`、右腕 identity，且
   proper rotation determinant 为 `+1`；所有基于旧 Y 轴候选的期望值同步更新。
8. 没有访问 Agile policy asset 的配置路径，纯配置构造不应因 lower-body policy
   下载失败。

若 EnvCfg 纯单元测试因 Isaac/Kit 导入边界无法在轻量 pytest 中运行，应把“固定模式
配置变换”抽成最小 helper 并测试该 helper；不能因此只留下视觉测试。

## 8. 双语 README

在同一变更中同步更新 `README.md` 和 `README_CH.md`：

- Noitom G1 task 现在是固定 root、固定髋/膝/踝的上半身 teleoperation；
- waist 仍由 Pink IK 控制；
- Agile lower-body policy 不再加载；
- action layout 从 60/32 改为 56/28；
- 左腕现场选定的 semantic offset 是局部 Z 轴 180 度 `[0,0,1,0]`；
- 更新所有包含旧 action 维度、4D locomotion suffix 或 Y 轴腕 offset 的描述。

英文和中文内容、默认值、命令、维度表必须一致。

## 9. 自动化验证命令

从仓库根执行并保存日志：

```bash
cd /home/lenovo/IsaacTeleop
source /home/lenovo/env_isaaclab/bin/activate

VALIDATION_DIR=/tmp/noitom-fixed-lower-body-validation
mkdir -p "$VALIDATION_DIR"
set -o pipefail

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python -m pytest examples/noitom/tests -vv \
  2>&1 | tee "$VALIDATION_DIR/pytest.log"

git diff --check \
  2>&1 | tee "$VALIDATION_DIR/diff-check.log"

SKIP=check-copyright-year \
pre-commit run --all-files \
  2>&1 | tee "$VALIDATION_DIR/pre-commit.log"
```

若默认 pre-commit cache 在当前沙箱只读，复制现有 cache 到任务专用 `/tmp` 目录并设置
`PRE_COMMIT_HOME` 后重跑。必须保留最终无修改且全部通过的一轮日志。

## 10. 现场站立测试命令

继续使用已经确定的顺序：先持续离线广播站立动作，再启动 Isaac。广播流不含 T-pose
不会影响固定下半身验证；本轮比较的是机器人 root/腿是否稳定，以及手腕修正是否无
回退。

```bash
VALIDATION_DIR=/tmp/noitom-fixed-lower-body-validation
mkdir -p "$VALIDATION_DIR"
cd /home/lenovo/IsaacLab
set -o pipefail

PYTHONUNBUFFERED=1 \
NOITOM_ORIENTATION_DEBUG=1 \
NOITOM_CLEAR_WORKSPACE=1 \
NOITOM_WRIST_ORIENTATION_MODE=source \
NOITOM_WRIST_ORIENTATION_COST=0.75 \
NOITOM_WRIST_PITCH_LIMIT_DEG=80 \
NOITOM_WRIST_YAW_LIMIT_DEG=80 \
PYTHONPATH=/home/lenovo/IsaacTeleop/examples/noitom:${PYTHONPATH:-} \
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
  --task Isaac-PickPlace-Locomanipulation-G1-Noitom-Abs-v0 \
  --visualizer kit \
  --xr \
  --external_callback noitom_tasks.register_tasks \
  2>&1 | tee "$VALIDATION_DIR/standing-fixed-lower-body.log"
```

测试至少持续 30 秒。提取摘要：

```bash
rg \
  'NoitomG1Env|NoitomLowerBodyDebug|NoitomWristAxisDebug|NoitomWristPoseError|Agile|QP|solver_held|NaN|Traceback' \
  /tmp/noitom-fixed-lower-body-validation/standing-fixed-lower-body.log \
  | tee /tmp/noitom-fixed-lower-body-validation/standing-fixed-lower-body-summary.log
```

## 11. 预期效果和重点关注

必须达到：

- 启动日志明确 `root_fixed=1`、`leg_action=disabled`、arm-frame action dim `56`；
- 日志不再加载或运行 Agile locomotion policy；
- pelvis 世界位姿不漂移，髋、膝、踝保持启动初始弯曲姿态，不再下蹲、迈步或响应
  BVH 下半身动作；
- waist 仍可随 Pink IK 合理调整，上肢和手指控制不回退；
- 左腕保持当前三轴基本对齐，配置打印为 `[0,0,1,0]`，右腕保持 identity；
- 无 action dimension mismatch、QP failure、`solver_held=1`、NaN 或 Traceback。

重点观察：

- 去掉腿部下蹲后，腕目标的位置残差和关节贴限是否改善；这属于可达性效果，不能再用
  新的 wrist offset 补偿；
- 若腿关节在固定 root 下仍明显漂移，记录实际关节误差和速度，再按第 4 节后备方案
  增加显式默认姿态 hold；
- 若手腕仍有小量残差，先区分 `semantic target -> Pink solution FK` 的 IK 残差与
  `Pink solution FK -> robot actual` 的执行残差，不要重新混淆源坐标系。

## 12. 非目标与停止条件

- 不修改 `/home/lenovo/IsaacLab` 上游源码。
- 不更换整个 EnvCfg 父类或任务 ID。
- 不固定 waist，不删除 Pink waist DOF。
- 不修改已现场选择的左腕 `[0,0,1,0]` 数值。
- 不重新调 Pink cost/gain/damping、腕限位或 retargeting anchor。
- 不为“以后可能恢复行走”预先增加模式开关；当前任务明确固定下半身。若未来需要移动
  模式，另立计划定义两个 action schema，避免一个 tensor 同时兼容 56D 和 60D。

若执行者发现删除 Agile term 会影响 Noitom 之外的任务，说明修改落错层：变更必须只
发生在 `NoitomLocomanipulationG1EnvCfg` 和 Noitom pipeline，立即停止扩大范围。

## 13. 执行结果（2026-07-24）

- `NoitomLocomanipulationG1EnvCfg` 继续固定 root，并将 inherited
  `lower_body_joint_pos` action 和 `lower_body_policy` observation 设为 `None`；没有
  修改 Isaac Lab 上游或增加后备腿部 action。
- Noitom action schema 已改为 arm-frame `56D` 和 wrist-only `28D`，删除 4D
  locomotion suffix；启动摘要会打印 `lower_body=fixed`、`root_fixed=1`、
  `leg_action=disabled` 和当前 action dimension。
- Pink 的 controlled-joint pattern 仍排除 hip/knee/ankle 并包含 `waist_.*_joint`；
  既有任务数量、权重、腕限位和 retargeting anchor 未调整。
- 左腕配置和自动化断言已同步为现场选定的局部 Z180 `[0,0,1,0]`，右腕仍为
  identity；中英文 README 已同步固定下半身行为和 action layout。
- `/home/lenovo/env_isaaclab/bin/activate` 环境下 Noitom 测试 `54 passed`，
  `git diff --check` 和完整 pre-commit hook set 均通过。验证日志保存在
  `/tmp/noitom-fixed-lower-body-validation/`。
- 第 10 节要求的至少 30 秒 Isaac/Noitom 现场站立测试尚未执行；是否需要显式默认
  姿态 hold 必须由该现场测试的腿部漂移证据决定。
