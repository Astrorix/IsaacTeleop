<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# 站立垂臂时左腕约 90 度偏转的分层诊断决策

**日期**：2026-07-24

**状态**：诊断已完成；“IK 限位为主因”的判断已被后续轴语义证据替代

**范围**：`examples/noitom/` 的 BVH 腕姿态、Pink 腕任务、IK 结果和仿真实际腕姿态诊断

> **后续修正**：90 度限位和 orientation cost A/B 已确认限位与权重只影响 Pink
> 对错误目标的追踪程度。左腕根因是进入 Pink 前的 BVH 腕坐标系解剖语义未归一化。
> 详见
> [`2026-07-24-left-bvh-wrist-semantic-frame-correction.md`](2026-07-24-left-bvh-wrist-semantic-frame-correction.md)。

## 对话背景

1. [`BVH_WRIST_AXIS_ALIGNMENT_PLAN.md`](../architectures/BVH_WRIST_AXIS_ALIGNMENT_PLAN.md)
   已由另一个 agent 实施。默认 `source` 模式直接让机器人对齐后的 BVH 腕三轴成为
   G1 `wrist_yaw_link` 的目标三轴，左右腕 JSON local offset 均为单位四元数。
2. 用户当前只能离线广播动作数据，数据从双臂自然下垂开始，不包含 T-pose。用户先
   只测试了站立动作。
3. 现场观察是右手视觉效果基本正确，但 G1 左手的 `wrist_yaw` 看起来仍多出约
   90 度旋转。
4. 本次运行日志位于
   `/tmp/noitom-wrist-axis-validation/source-cost-075.log`。用户希望下一轮继续以站立
   动作为唯一基准，并把打印的 7DoF 位姿与视口坐标轴逐一对应。
5. 当前对话窗口只负责分析和文档；代码由另一个 agent 按实施计划修改。

## 已确认的日志证据

校准后的源层日志显示：

- 左右手 `source_target_error_deg=0.00`。经机器人朝向对齐的 BVH 腕旋转与平滑后
  腕目标一致，没有在 BVH 到 Pink action 的方向映射中额外注入 90 度。
- 稳定段左腕目标四元数约为
  `[+0.0012, +0.5668, +0.3374, -0.7516]`，实际 G1 左腕四元数约为
  `[-0.2947, +0.0676, +0.4093, -0.8609]`，目标到实际旋转误差稳定在
  `69.20` 度。
- 同一稳定段右腕目标到实际旋转误差约为 `0.81` 度。
- 左腕实际关节约为：
  - roll：`-0.9048 rad`
  - pitch：`-1.3959 rad`，约 `-79.98` 度
  - yaw：`+1.3961 rad`，约 `+79.99` 度
- 左腕 pitch/yaw 的 `safe_margin_rad` 分别约为 `0.0003` 和 `0.0002`，已经
  实质触及当前 `+/-80` 度 Noitom 安全边界。
- 左腕 `solver_held=0`，Pink 并非因为 QP 异常而返回原位置；它仍在输出受限的
  局部解。
- 右腕 pitch/yaw 仍有约 `1 rad` 的安全余量，没有触限。
- 退出末尾出现的 `XR_ERROR_VALIDATION_FAILURE` 与稳定段腕姿态误差无直接证据链，
  不作为本次腕偏转原因。

启动早期、校准完成前后的巨大 Pink 误差来自机器人从默认 hold pose 向源腕目标
收敛，不能用来判断最终固定偏差。应以校准后且源姿态速度接近零的稳定段为准。

## 当前判断

当前可确定的故障边界是：

```text
BVH 腕轴 -> robot-aligned source -> Pink world action：一致
Pink world action -> Pink/机器人实际左腕：存在约 69.2 度残差
左腕 pitch/yaw：同时触及 +/-80 度安全边界
```

因此，视觉上的“左 wrist_yaw 多约 90 度”不是已知的 JSON offset 或源腕对齐函数
再次增加了固定 90 度，而是左腕在当前 Pink 解中接近 `+80` 度 yaw 边界的外观。
由于 pitch 同时接近 `-80` 度，剩余旋转并不是纯 yaw；腕部串联轴耦合会让视口外观
看起来接近单一 yaw 偏转。

站立垂臂会把腕链带到强耦合、接近欧拉奇异位形的区域。当前证据仍不能区分：

1. 目标在当前肩、肘、腕任务和安全边界下确实不可达；
2. Pink 从校准后的目标跳变进入了受限局部求解分支，但存在另一条可达分支；
3. Pink IK 解可以达到目标，但仿真关节控制或物理状态没有跟上。

第 3 点需要增加“Pink 解后 FK”与“仿真实际 link pose”的独立打印后才能排除。

## 不包含 T-pose 的影响

- `source` 模式的腕朝向直接使用当前 robot-aligned BVH 腕世界旋转，不使用 T-pose
  中立旋转做 calibration-relative delta。因此，首帧是站立垂臂而不是 T-pose，
  不会单独造成固定 90 度 source offset。
- 首帧校准仍影响人体骨长、目标位置、后续动作增量和 Pink 的初始收敛路径，所以
  可能影响求解分支与是否触限。
- 日志尾部的左右源腕相对校准帧旋转变化都约为 35 度，说明“站立动作流”不是完全
  静止的单帧。下一轮最好冻结或重复第一有效站立帧数秒；若广播器不能冻结，日志
  必须带 sample/cycle 标识，并在源停止变化后的稳定段比较。
- 缺少在线 T-pose 不妨碍本轮站立问题定位，但不能替代参考 BVH 第一帧已经承担的
  T-pose 轴语义自动化验证。

## 决策

1. 下一轮仍只使用站立垂臂动作，先建立清晰的分层 7DoF 证据，不改变测试变量。
2. 在获得同步位姿前，不修改左右腕 JSON offset、不降低/提高 orientation cost、
   不改变 wrist pitch/yaw 限位，也不增加左右侧特例。
3. 下一阶段必须区分并打印：
   - 第一有效 BVH 帧的原始 Isaac 轴系腕位姿；
   - 视口青色轴使用的 robot-aligned BVH 腕位姿；
   - Pink 收到的 world action 和实际写入 LocalFrameTask 的 pelvis-local target；
   - Pink 输出关节解做 FK 后的腕位姿；
   - 仿真机器人实际 `wrist_yaw_link` 位姿。
4. 世界位姿统一使用单环境的 Isaac environment-world frame；pelvis 局部位姿必须
   显式标注。四元数统一标注为 `xyzw`，比较旋转时允许 `q` 与 `-q` 等价。
5. Pink 解后 FK 必须基于最终安全裁剪后的关节目标，并与仿真实际腕位姿分开，
   不能把 action target 当成 IK result。
6. 代码修改仅增加诊断和测试，不改变运行行为。具体实现见
   [`../architectures/STANDING_WRIST_POSE_DIAGNOSTICS_PLAN.md`](../architectures/STANDING_WRIST_POSE_DIAGNOSTICS_PLAN.md)。

## 下一轮判读原则

- `bvh_aligned_world` 到 `pink_input_world` 稳定后误差大：源映射、offset 或平滑层。
- 两者一致，但 `pink_solution_fk_world` 误差大且关节边界余量接近零：Pink 局部解、
  任务约束或可达性问题。
- Pink solution FK 接近目标，但 `robot_actual_world` 误差大：关节目标执行或物理
  跟随问题。
- 只有在前三层证据确认左侧目标本身需要固定局部修正后，才能讨论 side-specific
  offset；不能仅凭 `wrist_yaw_joint` 数值或截图增加一个 90 度补偿。
