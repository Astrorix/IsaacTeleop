<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# BVH 手腕坐标轴到 G1 手掌坐标轴直接映射实施计划

**状态**：实现和自动化验证完成，实机验收待执行

**执行者**：另一个 coding agent

**背景决策**：
[`../decisions/2026-07-24-bvh-wrist-axis-alignment.md`](../decisions/2026-07-24-bvh-wrist-axis-alignment.md)

## 1. 目标与边界

让 G1 `left_wrist_yaw_link`/`right_wrist_yaw_link` 的目标三轴逐轴匹配视口中
对应的 BVH/Noitom 腕三轴，消除当前固定约 90 度偏转。

本次只修改 `examples/noitom/`：

- `noitom_retargeting.py`：源腕帧映射和诊断数据。
- `noitom_tasks.py`：默认模式和日志输出。
- `ik_config/noitom_to_g1.json`：腕部 local offset。
- `tests/`：T-pose 和逐轴回归测试。
- `README.md`、`README_CH.md`：同步更新行为和命令。
- `AGENTS.md`：规则已经由文档 agent 补充，执行 agent 不应删除。

不得修改 `src/core/**` 或 Noitom C++ 插件。不要把完整 BVH、截图或运行日志提交到
仓库。

## 2. 执行前检查

1. 从仓库根列出全部 `AGENTS.md`。
2. 阅读根 `AGENTS.md` 和 `examples/noitom/AGENTS.md`。
3. 检查工作树并保留现有未提交改动；这些文件已有另一轮重构内容，禁止用
   `git checkout`、`git reset` 或整文件替换清除它们。
4. 读取本计划、背景决策、当前中英文 README、配置和测试后再编辑。

## 3. 实现设计

### 3.1 新增 `source` 模式

- 将 `source` 加入 `_WRIST_ORIENTATION_MODES`。
- `NoitomRetargetingSettings.wrist_orientation_mode` 和
  `NoitomG1Settings.retargeting` 的发布默认值都改为 `source`。
- 保持 `forearm`、`twist`、`full` 的现有数值路径不变，避免兼容模式发生无关
  回归。

### 3.2 共享坐标转换

在 `noitom_retargeting.py` 中抽取一个只负责旋转的共享函数，语义为“将 BVH 腕
世界帧放到青色参考骨架使用的机器人对齐世界帧”：

```text
bvh_wrist_world = torso.rotation * arm.wrist_rot_torso
aligned_source = _reference_alignment_rotation(torso) * bvh_wrist_world
```

下列路径必须调用同一函数：

- `reference_wrist_frames()` 绘制青色细线轴。
- `source` 模式的 Pink 腕目标生成。
- `wrist_orientation_diagnostics()` 的 `reference_q` 和误差计算。

不要在这三条路径中分别复制公式。

### 3.3 配置 offset 语义

`source` 模式使用：

```text
target_rotation = aligned_source * local_offset
```

- `local_offset` 使用 xyzw 四元数，在 BVH 腕局部帧中后乘。
- `left_wrist_yaw_link` 和 `right_wrist_yaw_link` 的 offset 都改为
  `[0.0, 0.0, 0.0, 1.0]`。
- 只修改两个腕条目；肩和肘的 orientation cost 为 0，本次不扩大范围处理其
  rotation offset。
- 更新 JSON 注释，明确这些值不是世界姿态，也不是 VR controller target offset。
- 配置缺失的 legacy 路径在 `source` 模式下也使用单位 local offset，不能退回旧
  `_DEFAULT_*_WRIST_QUAT`。

### 3.4 朝向计算

- `source` 分支直接返回 `aligned_source * local_offset`，不使用标定时固定世界
  四元数，也不做 calibration-relative delta。
- 仍通过现有 `_smooth_pose()`/arm-node 更新路径执行四元数最短路 SLERP。
- 保留 Pink wrist pitch/yaw 边界和最终 joint-target clip。
- `source` 模式不调用 twist accumulator；reset/clear calibration 仍按现有方式清理
  状态，避免切换模式时遗留状态。

### 3.5 诊断

在 `WristOrientationDiagnostics` 增加 `source_target_error_deg`：

```text
degrees(magnitude(inverse(aligned_source) * smoothed_target))
```

`NoitomOrientationDebug` 同行打印该值。它用于区分：

- BVH→目标映射或平滑问题。
- Pink 目标→实际机器人求解问题。

现有 `NoitomPinkOrientationDebug error_deg` 继续表示目标和实际
`wrist_yaw_link` 的误差。

## 4. 自动化测试

### 4.1 Fixture

不要提交完整的
`/home/lenovo/Downloads/LC_02_run_01_001_processed_Liujiahao_first1000.bvh`。
在测试中增加精简 `bvh_rest_tpose_frame`：

- 复用现有 T-pose 上身位置。
- 左右腕世界四元数都设为 `[0, 0, 0, 1]`。
- 注释注明它来自参考 BVH 第一帧；该帧 Hips、双侧 Arm/ForeArm/Hand 局部旋转
  都为 0。

### 4.2 用例

1. 配置左右腕 local offset 都是单位四元数。
2. `source` 模式在 BVH rest T-pose 标定后：
   - 两个 `reference_wrist_frames()` 旋转为单位旋转。
   - 两个腕目标也为单位旋转。
3. 分别向左右腕输入 X、Y、Z 单轴和组合旋转；保持关节位置不变，断言腕目标和
   reference wrist frame 旋转完全一致。
4. 比较旋转矩阵或旋转距离，不直接逐元素比较四元数，允许 `q` 与 `-q` 等价。
5. 设置 `rotation_smoothing=1.0` 测纯映射；另保留一个小于 1 的用例验证停止输入后
   误差收敛。
6. 现有 `twist` golden 测试继续显式选择 `twist`，保证兼容模式数值不变。
7. 增加发布默认值为 `source` 的测试。

## 5. README 双语同步

同一提交内同步更新 `README.md` 与 `README_CH.md`：

- Pipeline 中腕部步骤改为“robot-aligned BVH wrist axes + local offset + smoothing”。
- 默认模式改为 `source`，说明红/绿/蓝轴逐轴对应。
- 明确两个腕 offset 为单位 local post-rotation；删除“侧特定 VR 中立四元数作为
  BVH offset”的旧描述。
- 保留并解释 `twist`、`forearm`、`full` 诊断模式。
- 增加 `source_target_error_deg` 的判读方式。
- 英文和中文的环境变量、默认值、命令和行为必须一致。

## 6. 测试命令与日志

### 6.1 自动化测试

```bash
cd /home/lenovo/IsaacTeleop
source /home/lenovo/env_isaaclab/bin/activate

VALIDATION_DIR=/tmp/noitom-wrist-axis-validation
mkdir -p "$VALIDATION_DIR"
set -o pipefail

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python -m pytest examples/noitom/tests -vv \
  2>&1 | tee "$VALIDATION_DIR/pytest.log"

git diff --check \
  2>&1 | tee "$VALIDATION_DIR/git-diff-check.log"

SKIP=check-copyright-year \
pre-commit run --all-files \
  2>&1 | tee "$VALIDATION_DIR/pre-commit.log"
```

预期：pytest、`git diff --check` 和完整 pre-commit 全部通过。若 pre-commit 暴露可
复用的仓库规则，按根 `AGENTS.md` 的 learning loop 在同一轮补充最局部文档。

### 6.2 实机主测试

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
  2>&1 | tee "$VALIDATION_DIR/source-cost-075.log"
```

按顺序测试：第一帧 T-pose 掌心向下、站立垂臂、双手向前平举、左右腕分别绕三个
局部轴缓慢旋转、跨越四元数正负表示边界、接近物理极限后返回中立。

### 6.3 orientation cost 隔离测试

若主测试仍有偏差，运行：

```bash
VALIDATION_DIR=/tmp/noitom-wrist-axis-validation
mkdir -p "$VALIDATION_DIR"
cd /home/lenovo/IsaacLab
set -o pipefail

PYTHONUNBUFFERED=1 \
NOITOM_ORIENTATION_DEBUG=1 \
NOITOM_CLEAR_WORKSPACE=1 \
NOITOM_WRIST_ORIENTATION_MODE=source \
NOITOM_WRIST_ORIENTATION_COST=0 \
PYTHONPATH=/home/lenovo/IsaacTeleop/examples/noitom:${PYTHONPATH:-} \
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
  --task Isaac-PickPlace-Locomanipulation-G1-Noitom-Abs-v0 \
  --visualizer kit \
  --xr \
  --external_callback noitom_tasks.register_tasks \
  2>&1 | tee "$VALIDATION_DIR/source-cost-0.log"
```

cost 为 0 时机器人不跟随朝向属于预期；此运行只验证目标生成与 Pink 执行层边界。

### 6.4 提取关键日志

```bash
VALIDATION_DIR=/tmp/noitom-wrist-axis-validation

rg \
  'NoitomOrientationDebug|NoitomPinkOrientationDebug|NoitomPinkJointDebug|QP|Traceback' \
  "$VALIDATION_DIR/source-cost-075.log" \
  | tee "$VALIDATION_DIR/source-cost-075-summary.log"

rg \
  'NoitomOrientationDebug|NoitomPinkOrientationDebug|NoitomPinkJointDebug|QP|Traceback' \
  "$VALIDATION_DIR/source-cost-0.log" \
  | tee "$VALIDATION_DIR/source-cost-0-summary.log"
```

## 7. 预期效果与重点问题

### 预期效果

- T-pose 稳定后，BVH 细线轴和实际 G1 腕轴红对红、绿对绿、蓝对蓝。
- 第一帧 `reference_q` 和未平滑目标是单位四元数或其等价负四元数。
- 稳定可达姿态下 `source_target_error_deg < 1` 度。
- 稳定可达姿态下 `NoitomPinkOrientationDebug error_deg < 10` 度。
- 不再出现由旧 VR offset 引入的固定约 90 度偏差。

### 重点关注

1. **映射层**：若 `source_target_error_deg` 在停止运动后仍大于 1 度，检查共享对齐
   函数、offset 乘法顺序或是否错误使用了 calibration-relative nominal。
2. **平滑层**：运动中允许短暂误差；停止后必须收敛。不要把正常平滑滞后误判为
   固定帧偏差。
3. **IK 层**：若源到目标误差接近 0、但目标到实际误差大，检查 Pink QP、任务权重
   和 `solver_held`。
4. **物理限位**：若 `safe_margin_rad` 接近 0，实际腕轴无法完全跟随可以接受，但
   目标轴仍必须和 BVH 一致。
5. **左右手对称性**：不要因为右手指骨沿 BVH 局部 `-X` 延伸而额外镜像
   `RightHand` 腕帧；逐轴验收基于腕坐标系本身。
6. **默认行为**：无环境变量时必须进入 `source` 模式；显式选择旧模式才使用旧
   算法。

## 8. 完成条件

- 自动化、diff check 和完整 pre-commit 全部通过。
- 中英文 README 同步。
- 实机主测试日志已保存，T-pose、垂臂、前平举和三轴转腕均完成。
- 若尚未获得实机结果，提交说明必须明确标注“自动化完成，实机验收待执行”，不能
  把计划状态写成完全完成。

## 9. 执行结果（2026-07-24）

- 已新增并默认启用 `source` 模式；目标生成、青色参考腕轴和诊断共用同一源腕对齐
  函数。
- 左右腕 local offset 已改为单位四元数；config-free `source` 路径也明确使用单位
  local offset，不继承旧 VR controller nominal 四元数。
- 已增加参考 BVH 第一帧、左右腕 X/Y/Z 与组合旋转、平滑收敛、配置和发布默认值
  回归测试；旧 golden 测试继续显式使用 `twist`。
- `source_target_error_deg` 已加入诊断日志，中英文 README 已同步。
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest examples/noitom/tests -vv`：
  `39 passed`。
- `git diff --check` 和 `SKIP=check-copyright-year pre-commit run --all-files`：
  通过。日志保存在 `/tmp/noitom-wrist-axis-validation/`。
- Noitom HDS + Isaac Lab 实机姿态验收尚未执行，需按第 6.2 至 6.4 节完成。
