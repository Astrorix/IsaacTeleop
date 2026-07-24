<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# 左侧 BVH 腕部语义坐标系修正实施计划

**状态**：实现和自动化验证完成；现场后续选择局部 Z 轴 180 度，坐标系已基本对齐

**执行者**：另一个 coding agent

**背景决策**：
[`../decisions/2026-07-24-left-bvh-wrist-semantic-frame-correction.md`](../decisions/2026-07-24-left-bvh-wrist-semantic-frame-correction.md)

## 1. 目标

把 Noitom/BVH 左腕的镜像局部轴约定归一化为 G1 `left_wrist_yaw_link` 的解剖语义：

```text
X: 掌心 -> 指尖
Y/Z: 保持合法右手系，并与机器人同色轴逐轴对应
```

当前日志已证明修正前 `dot(left_target_X, elbow_to_wrist)=-0.9763`，右侧为
`+0.9644`。完成后，左右 semantic target 的 X 轴都必须朝向手掌/指尖方向，Pink
不得再接收左侧反向 X 轴。

本计划只修坐标系语义。不要借机调整 Pink 默认 cost、gain、damping、腕限位、位置
权重、骨长或平滑参数。

## 2. 强制执行前检查

1. 从 `/home/lenovo/IsaacTeleop` 运行：

   ```bash
   find . -name AGENTS.md ! -path '*/.git/*' | sort
   ```

2. 阅读根 `AGENTS.md` 和 `examples/noitom/AGENTS.md`。
3. 检查 dirty worktree，保留用户及其他 agent 的已有改动；禁止 checkout/reset 或
   整文件覆盖。
4. 阅读本计划、背景决策、现有实现和相关测试。

## 3. 预计修改范围

- `examples/noitom/ik_config/noitom_to_g1.json`
- `examples/noitom/noitom_retargeting.py`
- `examples/noitom/noitom_tasks.py`（只改诊断字段/打印所需部分）
- `examples/noitom/tests/test_retargeting_config.py`
- `examples/noitom/tests/test_wrist_pose_diagnostics.py`
- 必要时更新 `examples/noitom/tests/conftest.py`
- `examples/noitom/README.md`
- `examples/noitom/README_CH.md`

禁止修改 Isaac Lab 上游源码、Noitom C++ 插件、`src/core/**` 或默认 Pink/腕限位
参数。若修改 README，英文和中文必须在同一变更中同步。

## 4. 配置变更

在 `ik_config/noitom_to_g1.json` 中只修改左腕 local rotation offset：

```json
"left_wrist_yaw_link": [
  "LEFT_WRIST",
  18.0,
  0.75,
  [0.0, 0.0, 0.0],
  [0.0, 1.0, 0.0, 0.0]
]
```

右腕继续使用：

```json
[0.0, 0.0, 0.0, 1.0]
```

更新 JSON `_comment`：offset 是把 BVH 镜像手骨的局部 frame 归一化为 G1 腕解剖
frame 的 local post-rotation；左侧绕局部 Y 轴 180 度，右侧 identity。不要写成
世界姿态、VR controller offset 或单轴镜像。

基线候选 `[0,1,0,0]` 的含义是保留当前绿色 Y 轴并同时翻转红色 X、蓝色 Z，保持
determinant `+1`。如果现场三轴核对明确证明应保留 Z 而不是 Y，停止并把配置候选
统一改为 `[0,0,1,0]`；配置、代码测试和双语文档必须使用同一个候选，禁止隐藏
硬编码第二份补偿。

## 5. 统一 raw 与 semantic frame 路径

当前 `_aligned_source_wrist_rotation(torso, arm)` 返回 raw robot-aligned BVH frame，
而 `_tracked_wrist_quaternion()` 在 `source` 模式中随后乘配置 offset。重构成两个
语义清楚的阶段，例如：

```python
aligned_raw = _aligned_source_wrist_rotation(torso, arm)
semantic = aligned_raw * Rotation.from_quat(local_offset_xyzw)
```

实现要求：

1. 增加一个小型共享 helper（名称可按现有风格调整），输入 raw aligned rotation 和
   normalized local offset，返回 semantic rotation。
2. `source` 模式的 `_tracked_wrist_quaternion()` 调用该 helper。
3. 用于与 G1 同色轴对比的 `reference_wrist_frames()` 也调用同一 helper，并从当前
   `ik_config` 按 side 获取 wrist offset。无 config 的 legacy 路径继续使用 identity。
4. offset 只乘一次，顺序必须是右乘 local post-rotation：
   `aligned_raw * local_offset`。禁止改成左乘世界旋转。
5. `twist`、`forearm`、`full` 模式行为保持不变；若这些模式继续显示 raw frame，
   用明确命名区分，不能静默混用 source semantic frame。
6. 不要在 C++ 插件修正。插件应继续发布真实 Noitom/BVH 世界旋转；语义归一化属于
   Noitom-to-G1 retargeting 配置边界。

## 6. 诊断模型与日志

现有 `bvh_aligned_world`/`reference_q` 会在 offset 非 identity 后产生歧义。保留
raw 事实并新增 semantic 阶段：

```text
bvh_aligned_raw_world
bvh_semantic_world
retarget_target_world
pink_input_world
pink_solution_fk_world
robot_actual_pre_apply_world
```

要求：

- `bvh_aligned_raw_world` 不应用 side-specific offset；
- `bvh_semantic_world` 应用一次配置 offset，并与视口用于对比 G1 的 BVH 腕轴一致；
- `retarget_target_world` 在关闭平滑或平滑收敛后与 semantic frame 误差接近零；
- 第一有效帧打印 `raw_to_semantic_rot_deg` 和
  `semantic_to_retarget_rot_deg`，不能再把 raw 到 target 的 180 度当成错误；
- 低频增加或扩展轴诊断，至少包含：

  ```text
  side
  raw_x_dot_forearm
  semantic_x_dot_forearm
  semantic_x_world
  semantic_y_world
  semantic_z_world
  local_offset_xyzw
  ```

  forearm 方向使用与目标位置相同的 robot-aligned `elbow -> wrist` 方向。退化长度时
  跳过该字段，禁止产生 NaN。

预期 standing 首帧近似：

```text
left  raw_x_dot_forearm     ~= -0.98
left  semantic_x_dot_forearm ~= +0.98
right raw_x_dot_forearm     ~= +0.96
right semantic_x_dot_forearm ~= +0.96
```

视口只把 `bvh_semantic_world` 作为“应与机器人逐轴重合”的 BVH 腕 frame。若仍需
显示 raw frame，必须使用不同尺寸或样式并在 README 解释，避免用户误认为两个 frame
都是 Pink 目标。

## 7. 自动化测试

### 7.1 配置测试

把 `test_config_wrist_offsets_are_identity_local_post_rotations` 改为语义明确的测试：

- left wrist offset 等于 `[0,1,0,0]`；
- right wrist offset 等于 `[0,0,0,1]`；
- 两者归一化且对应 rotation matrix determinant 为 `+1`。

### 7.2 source 模式测试

替换“左右 T-pose target 都是 identity”的错误假设：

- raw aligned frame 可以保留源 BVH 数值；
- semantic reference 和 source target 必须逐侧一致；
- rest frame 左侧 semantic target 相对 raw frame为绕 local Y 的 180 度，右侧为 0；
- 对左右源腕分别施加 X/Y/Z 和组合旋转后，目标始终等于
  `aligned_raw * configured_side_offset`；
- 四元数正负号等价。

### 7.3 解剖轴回归

增加一个最小回归，使用本次现场第一有效帧的已记录方向/四元数，或等价的确定性
fixture，验证：

```text
dot(left_semantic_X, left_elbow_to_wrist) > 0.95
dot(right_semantic_X, right_elbow_to_wrist) > 0.95
```

同时验证修正前左侧点积 `< -0.95`，确保测试真的覆盖本次错误而不是恒真断言。

### 7.4 诊断测试

- raw/semantic/target 三阶段均使用规范化 `xyzw` 四元数；
- semantic frame 与 `reference_wrist_frames()`、视口 frame 使用同一路径；
- 左侧 `raw_to_semantic_rot_deg` 约 180，右侧约 0；
- 左右 `semantic_to_retarget_rot_deg` 在无平滑时接近 0；
- 更新第一有效帧日志条数和字段断言；
- 禁用诊断时仍不得执行额外 pose/axis 计算。

## 8. 双语 README

同步更新 `README.md` 和 `README_CH.md`：

- 解释 raw BVH 左右镜像手骨的纵向局部轴语义不同；
- source 模式跟踪的是 side-normalized semantic wrist frame；
- 左腕 local Y 180 度 offset 是 proper rotation，不是反射或 VR offset；
- 说明 raw、semantic、Pink target、solution FK、robot actual 日志阶段；
- 不把 cost=4 或 90 度限位写成新默认值。

## 9. 自动化验证命令

从仓库根目录运行：

```bash
cd /home/lenovo/IsaacTeleop
source /home/lenovo/env_isaaclab/bin/activate

VALIDATION_DIR=/tmp/noitom-wrist-axis-validation
mkdir -p "$VALIDATION_DIR"
set -o pipefail

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python -m pytest examples/noitom/tests -vv \
  2>&1 | tee "$VALIDATION_DIR/left-semantic-frame-pytest.log"

git diff --check \
  2>&1 | tee "$VALIDATION_DIR/left-semantic-frame-diff-check.log"

SKIP=check-copyright-year \
pre-commit run --all-files \
  2>&1 | tee "$VALIDATION_DIR/left-semantic-frame-pre-commit.log"
```

必须修复所有失败。若 Ruff format 改写文件，重新运行完整 pre-commit，直到最终一轮
无修改通过。

## 10. 实施后现场测试

保持“先持续广播站立动作，再启动 Isaac”的顺序。先用诊断宽松参数确认三轴映射：

```bash
VALIDATION_DIR=/tmp/noitom-wrist-axis-validation
mkdir -p "$VALIDATION_DIR"
cd /home/lenovo/IsaacLab
set -o pipefail

PYTHONUNBUFFERED=1 \
NOITOM_ORIENTATION_DEBUG=1 \
NOITOM_CLEAR_WORKSPACE=1 \
NOITOM_WRIST_ORIENTATION_MODE=source \
NOITOM_WRIST_ORIENTATION_COST=4.0 \
NOITOM_WRIST_PITCH_LIMIT_DEG=90 \
NOITOM_WRIST_YAW_LIMIT_DEG=90 \
PYTHONPATH=/home/lenovo/IsaacTeleop/examples/noitom:${PYTHONPATH:-} \
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
  --task Isaac-PickPlace-Locomanipulation-G1-Noitom-Abs-v0 \
  --visualizer kit \
  --xr \
  --external_callback noitom_tasks.register_tasks \
  2>&1 | tee "$VALIDATION_DIR/standing-left-semantic-cost-4-limit-90.log"
```

确认三轴正确后，关闭 Isaac，保持广播器运行，用生产默认参数复测：

```bash
VALIDATION_DIR=/tmp/noitom-wrist-axis-validation
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
  2>&1 | tee "$VALIDATION_DIR/standing-left-semantic-defaults.log"
```

提取摘要：

```bash
rg \
  'NoitomWristAxisDebug|NoitomWristPoseError|NoitomPinkJointDebug|QP|Traceback' \
  /tmp/noitom-wrist-axis-validation/standing-left-semantic-cost-4-limit-90.log \
  /tmp/noitom-wrist-axis-validation/standing-left-semantic-defaults.log \
  | tee /tmp/noitom-wrist-axis-validation/standing-left-semantic-summary.log
```

## 11. 预期效果与停止条件

### 必须达到

- 视口中的 semantic BVH 左腕红色 X 轴由掌心指向指尖，与 G1 左腕红轴同向；
- 左腕另外两轴也按颜色与 G1 对应，三个轴构成右手系；
- 右腕视觉与数值行为不回退；
- `left semantic_x_dot_forearm > 0.95`，不再为负；
- 无平滑或收敛后 `semantic_to_retarget_rot_deg` 接近 0；
- Pink solution FK 与 robot actual 仍保持小误差；
- 没有 QP failure、`solver_held=1`、NaN 或 Traceback。

### 重点观察但不与坐标系修复混为一谈

- `input_to_solution_rot_deg` 是否降至与右腕同量级；
- 左腕是否离开 pitch/yaw 双边界；
- cost=4 时左腕位置误差是否从此前约 `75 mm` 显著下降；
- 回到默认 cost=0.75、limit=80 后是否仍有独立的可达性或任务权重问题。

如果 semantic X 已正确但绿色/蓝色轴不匹配，停止 IK 参数调试，先在
`[0,1,0,0]` 与 `[0,0,1,0]` 两个合法 180 度候选中根据 T-pose 掌心向下语义选择
正确轴并同步测试/文档。如果三轴已经正确而 Pink 仍有大残差，再开启独立 IK
可达性调查；不得再次用固定 offset 掩盖 IK 问题。

## 12. 执行结果（2026-07-24）

- 左腕 wrist local offset 已改为 `[0,1,0,0]`，右腕保持 identity；JSON 注释明确
  这是把镜像 BVH 手骨 frame 归一化为 G1 解剖 frame 的局部 Y 轴 180 度 proper
  rotation。
- 已抽取统一 semantic helper，并确保 `source` 目标与视口腕轴只按
  `aligned_raw * local_offset` 应用一次 offset；无配置路径继续使用 identity。
- 诊断已拆分为 `bvh_aligned_raw_world` 和 `bvh_semantic_world`，第一有效帧增加
  `raw_to_semantic_rot_deg`、`semantic_to_retarget_*` 以及 `NoitomWristAxisDebug`。
- 合成镜像 T-pose 回归确认左侧 raw X/前臂点积小于 `-0.95`，语义修正后左右均大于
  `+0.95`；左右 X/Y/Z 与组合旋转继续按各自配置 offset 匹配。
- `forearm`、`twist`、`full` 三种兼容模式已用 identity/semantic 双配置参数化测试
  验证数值不变；Pink 默认参数、腕限位和插件未修改。
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest examples/noitom/tests -vv`：
  `50 passed`。
- 第 10 节的现场三轴核对与宽松/默认参数 A/B 尚未执行。

## 13. 现场后续结果（2026-07-24）

- 先使用局部 Y 轴 180 度 `[0,1,0,0]` 后，左腕 X 轴方向可以对齐；继续核对其余
  两轴，最终在配置中选择 `[0,0,1,0]`，即局部 Z 轴 180 度。
- `[0,0,1,0]` 也等价于先绕 Y 轴 180 度、再绕旋转后的 X 轴 180 度；它翻转
  X/Y、保留 Z，仍为 determinant `+1` 的 proper rotation。
- 用户确认当前 BVH 与机器人腕坐标系已经基本对齐。后续 agent 必须保留当前配置，
  并把本计划第 4、7、8、12 节中仍记录的早期 Y 轴候选视为历史实施基线；实际
  配置注释、测试断言和双语 README 应统一更新为 Z 轴候选。
