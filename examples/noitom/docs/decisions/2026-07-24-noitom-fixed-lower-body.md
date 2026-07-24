<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Noitom G1 固定下半身决策

**日期**：2026-07-24

**状态**：已实现并通过自动化验证，现场站立验收待执行

**范围**：Noitom G1 站立 teleoperation 的 root、腿部 action schema 和 Pink 边界

## 背景

1. 经过左腕 semantic frame 修正和现场逐轴核对，当前左腕 local offset 已选择
   `[0,0,1,0]`，右腕为 identity；用户确认 BVH 与机器人腕坐标系目前基本对齐。
2. 当前 `NoitomLocomanipulationG1EnvCfg` 已设置 `fix_root_link=True`，原因是继承的
   Agile lower-body policy 会让机器人下蹲，改变上肢 IK 的可达空间。
3. 仅固定 root 仍不等于固定下半身：继承的 `AgileBasedLowerBodyActionCfg` 继续控制
   12 个髋、膝、踝关节，Noitom action 尾部也仍发送三维速度和 `0.72` hip height。
4. Isaac Lab 已有 fixed-base upper-body IK G1 任务作为设计先例：固定 root，只保留
   upper-body IK，不加载 lower-body policy action/observation。
5. 本窗口只写文档；代码、配置和测试由另一个 agent 按架构计划实施。

## 决策

1. Noitom G1 默认改为固定下半身站立任务，不保留同一任务内的移动模式开关。
2. pelvis/root 继续使用 `fix_root_link=True`；髋、膝、踝不再由 Agile policy 控制，
   保持 G1 articulation 的初始腿部姿态。
3. 固定范围精确为 `.*_hip_.*_joint`、`.*_knee_joint` 和
   `.*_ankle_.*_joint`。腰关节继续由 Pink IK 控制，不属于本次固定范围。
4. 在 Noitom EnvCfg 层把 inherited lower-body action 和对应 observation 设为
   `None`，不修改 Isaac Lab 上游文件，也不整体更换 EnvCfg 父类。
5. 删除不再被 ActionManager 消费的 4 维 locomotion action suffix：arm-frame 模式
   从 60D 改为 56D，wrist-only 模式从 32D 改为 28D。
6. 如果现场日志证明无 lower-body action 时腿部无法维持 articulation 初始 target，
   才使用显式零输入、default-offset 的 joint-position hold 作为后备方案。采用后备
   方案必须记录证据，并同步重新定义 action schema。
7. 本次实现不得调整 Pink cost/gain/damping、腕限位、retargeting anchor，或改变
   当前已接受的左腕 `[0,0,1,0]` semantic offset。

## 预期结果

- 机器人 pelvis 世界位姿固定，髋、膝和踝不再下蹲、迈步或响应 BVH 下肢动作。
- Agile locomotion policy 不再加载，相关 observation 不再计算。
- waist 仍能为腕部可达性参与 Pink IK；双臂、双腕和双手行为保持现状。
- 腿部姿态稳定后，腕部位置/姿态残差可重新解释为上肢 IK 可达性问题，不再混入
  lower-body policy 的动态变化。

具体实施、测试命令和停止条件见
[`../architectures/NOITOM_FIXED_LOWER_BODY_PLAN.md`](../architectures/NOITOM_FIXED_LOWER_BODY_PLAN.md)。
