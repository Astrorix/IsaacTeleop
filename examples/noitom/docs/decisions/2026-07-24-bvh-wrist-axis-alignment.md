<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# BVH 手腕坐标轴与 G1 手掌坐标轴逐轴对齐决策

**日期**：2026-07-24

**状态**：已被后续现场轴语义证据部分修正

**范围**：`examples/noitom/` 的腕部朝向重定向、诊断、测试和双语文档

> **后续修正**：左右腕使用单位 local offset 的决定不能保证解剖语义一致。现场日志
> 已证明左侧 robot-aligned BVH X 轴与肘到腕方向相反，而右侧同向。以
> `aligned_to_retarget_rot_deg=0` 作为“逐轴正确”的验收也不充分，因为它只能证明
> 同一个坐标系被原样传入 Pink。现行决定见
> [`2026-07-24-left-bvh-wrist-semantic-frame-correction.md`](2026-07-24-left-bvh-wrist-semantic-frame-correction.md)。

## 对话背景

1. `RETARGETING_REFACTOR_PLAN.md` 执行完成后，用户要求提供自动化和实机测试命令。
   纯 Python 测试在 Isaac Lab Python 环境中得到 `26 passed`。
2. 用户分别运行了主 teleop 命令和 `NOITOM_WRIST_ORIENTATION_COST=0` 的 A/B 命令，
   两者都观察到同一现象：BVH 手掌近似沿前臂、手腕相对父关节旋转为 0 时，G1
   手掌在 `wrist_yaw` 附近存在约 90 度固定偏转。
3. 用户提供了两张现场截图：
   - `/home/lenovo/Pictures/isaacteleop_g1_arms_outstretched.png`：BVH 双手向前平举。
   - `/home/lenovo/Pictures/isaacteleop_g1_stand.png`：BVH 站立、双手放在身体两侧。
4. 初步分析确认位置重定向基本正常，偏差集中在 BVH/Noitom 腕局部零姿态到 G1
   `wrist_yaw_link` 目标帧的旋转定义。
5. 曾讨论“修正帧语义”“只调固定四元数”“禁用腕部朝向”三个方向；用户没有接受
   这些笼统选项，而是明确最终需求：**机器人手掌方向应逐轴匹配 BVH 手腕坐标轴**。
6. 用户提供参考 BVH：
   `/home/lenovo/Downloads/LC_02_run_01_001_processed_Liujiahao_first1000.bvh`。
   第一帧是标准 rest pose/T-pose，掌心指向地面。
7. 本窗口仅负责记录背景和执行计划；代码实现交给另一个 agent，避免两个 agent
   同时改动相同实现文件。

## 已核实证据

### BVH 第一帧

文件包含 180 个通道、1000 帧，旋转通道顺序为
`Yrotation Xrotation Zrotation`。第一帧中：

- `Hips` 旋转为 `(0, 0, 0)`。
- `RightArm`、`RightForeArm`、`RightHand` 的旋转均为 `(0, 0, 0)`。
- `LeftArm`、`LeftForeArm`、`LeftHand` 的旋转均为 `(0, 0, 0)`。

插件通过父姿态乘局部姿态构造世界姿态，因此第一帧左右手腕的 Noitom 世界旋转都是
单位旋转。单位旋转经过 `noitom_quaternion_to_isaac()` 的基变换后仍是单位旋转。
对于这份 T-pose，当前青色参考腕坐标轴的机器人对齐旋转也为单位旋转。

### 当前固定四元数的来源

`ik_config/noitom_to_g1.json` 当前腕部条目使用：

- 左腕：`[-0.2706, 0.6533, 0.2706, 0.6533]`
- 右腕：`[-0.7071, 0.0, 0.7071, 0.0]`

它们来自 Isaac Lab 的 VR 控制器管线。原管线设置了 `use_wrist_rotation=False`，这些
值是控制器到 G1 的固定目标姿态偏置，不是 BVH 腕局部帧 offset。把它们直接作为
BVH 标定时的腕目标，会在 BVH 相对腕旋转为 0 时仍固定注入大约 90 度旋转。

### G1 手掌帧

参考 G1 URDF 中，左右 `wrist_yaw_link` 到手掌几何体没有固定旋转，手掌使用与
`wrist_yaw_link` 相同的方向基。因此目标是让 `wrist_yaw_link` 的三个世界轴直接
匹配机器人对齐后的 BVH 腕三个世界轴，而不是再叠加 VR 控制器偏置。

## 决策

1. 新增直接源坐标轴模式 `source`，并作为 Noitom G1 示例默认腕部朝向模式。
2. `source` 模式目标旋转定义为：

   ```text
   aligned_source = reference_alignment(torso) * bvh_wrist_world
   wrist_target = aligned_source * configured_local_offset
   ```

3. ~~左右腕的 `configured_local_offset` 默认为单位四元数 `[0, 0, 0, 1]`。~~
   此项已被现场三轴证据推翻；保留在此仅记录历史决定。
4. 目标生成和青色 BVH 腕坐标轴可视化必须调用同一个对齐函数，防止两条路径再次
   漂移。
5. “逐轴匹配”以旋转坐标系为准，不以 `wrist_yaw_joint` 数值必须等于 0 为准；IK
   可以通过肩、肘和三个腕关节共同实现目标帧。
6. ~~BVH 右手子骨骼沿局部 `-X` 延伸是该 BVH 的层级定义，不据此额外镜像右腕坐标
   系。需求是匹配 `RightHand` 腕帧本身的三个轴。~~ 此项遗漏了解剖轴语义验证，
   已由后续决定替代。
7. 保留姿态平滑、Pink IK、安全限位以及现有 `twist`、`forearm`、`full` 模式，供
   兼容和诊断使用。

## 未采用方案

- **只调两个固定世界四元数**：只能修正特定姿势，无法保证站立垂臂、前平举和任意
  三轴转腕时都逐轴一致。
- **继续用 swing + twist 近似完整手腕姿态**：腕部屈伸或侧偏但前臂方向不变时，
  不能完整恢复 BVH 三轴。
- **将 orientation cost 设为 0**：只能隔离 IK 层，机器人不会跟随掌心方向，不是
  产品行为。
- **直接复制 GMR 腕 offset**：GMR 的 BVH/MuJoCo 坐标约定和四元数顺序不同，且
  当前需求已有可观测的本地 BVH 参考轴，不需要引入另一套帧假设。

## 验收原则

- T-pose 稳定后，青色 BVH 腕轴与 G1 `wrist_yaw_link` 轴按颜色逐一重合。
- 站立垂臂、向前平举以及 X/Y/Z 三轴转腕时保持同样的轴对应关系。
- 目标生成层的源帧到目标帧误差应接近 0；实际机器人误差单独由 Pink IK、平滑和
  关节限位解释。
- 修改英文 `README.md` 时必须在同一变更中同步 `README_CH.md`。

详细实现和测试步骤见
[`../architectures/BVH_WRIST_AXIS_ALIGNMENT_PLAN.md`](../architectures/BVH_WRIST_AXIS_ALIGNMENT_PLAN.md)。
