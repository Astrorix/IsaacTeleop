<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# 左侧 BVH 腕部解剖坐标系归一化决策

**日期**：2026-07-24

**状态**：语义修正已实现；现场选择局部 Z 轴 180 度后，BVH 与机器人腕坐标系已
基本对齐

**范围**：`examples/noitom/` 的 source 腕姿态、配置 offset、调试坐标轴和回归测试

## 对话和测试背景

1. 用户在站立垂臂测试中观察到：G1 右手视觉上正常，左手 `wrist_yaw` 近似多出
   90 度。同步 7DoF 日志证明 `pink_solution_fk_world` 与
   `robot_actual_pre_apply_world` 高度一致，排除了仿真实际执行层。
2. 将腕 pitch/yaw 诊断限位从 80 度提高到 90 度后，左腕仍立即贴住新的有效 yaw
   边界 `-1.5644 rad`。在近似相同的源姿态上，左腕
   `input_to_solution_rot_deg` 仅从 `66.33` 度降到 `62.62` 度，因此 80 度安全限位
   不是固定偏转根因。
3. 将 `NOITOM_WRIST_ORIENTATION_COST` 从 `0.75` 提高到 `4.0` 后，近似相同站立
   姿态的左腕方向误差降到 `12.30` 度，但位置误差从约 `9.9 mm` 增大到
   `74.6 mm`，且左腕 pitch/yaw 同时贴近 `-89.63` 度。右腕同期方向误差约
   `2.93` 度、位置误差约 `15.3 mm`。这说明 Pink 正在更强地追踪输入方向，并以
   位置和关节余量为代价；提高 cost 不能修正输入坐标系语义。
4. 用户在视口逐轴核对后指出：
   - G1 左右腕红色 X 轴均由掌心指向指尖；
   - BVH 右腕 X 轴同样由掌心指向指尖；
   - BVH 左腕 X 轴却由指尖指向掌心；
   - Pink 的目标语义是三轴严格对齐，因此错误的左腕目标会让机器人左手朝后。
5. 本窗口只允许分析和文档修改；实现由另一个 agent 完成。

## 日志中的独立数值证据

日志：
`/tmp/noitom-wrist-axis-validation/standing-limit-90-cost-4.log`。

第一有效帧记录了语义修正前的 robot-aligned BVH 腕姿态、腕目标和肘/腕位置。把
目标旋转矩阵的第一列记为腕坐标系 X 轴，把 `wrist_position - elbow_position`
归一化为前臂到手掌方向，得到：

```text
left:  dot(target_X, elbow_to_wrist) = -0.9763
right: dot(target_X, elbow_to_wrist) = +0.9644
```

这与用户的视觉判断一致：右侧 X 轴沿掌心到指尖，左侧 X 轴几乎精确反向。

同一帧还显示：

```text
left  bvh_aligned_world quaternion = [+0.050041, +0.620278, +0.216174, -0.752343]
left  retarget_target_world quaternion = [+0.050041, +0.620278, +0.216174, -0.752343]
right bvh_aligned_world quaternion = [+0.019208, -0.617500, -0.226957, -0.752871]
right retarget_target_world quaternion = [+0.019208, -0.617500, -0.226957, -0.752871]
```

因此 `aligned_to_retarget_rot_deg=0` 并没有排除源坐标系错误；它只证明错误的左侧
robot-aligned frame 被不加修正地送入了 Pink。

## BVH 层级证据

参考文件：
`/home/lenovo/Downloads/LC_02_run_01_001_processed_Liujiahao_first1000.bvh`。

第一帧是所有手臂局部旋转为零的 T-pose。层级中：

- `LeftForeArm -> LeftHand` 的 offset 为 `[+25, 0, 0]`；左侧手指子骨骼继续沿
  局部 `+X` 延伸；
- `RightForeArm -> RightHand` 的 offset 为 `[-25, 0, 0]`；右侧手指子骨骼继续沿
  局部 `-X` 延伸；
- 左右手的局部旋转数值都为零，不代表两个局部 frame 具有相同的“掌心到指尖”
  解剖语义。

这是镜像骨骼常见的轴约定差异。必须在 retargeting 边界把源 frame 归一化为机器人
腕 frame 语义，不能用 IK 权重、关节限位或单轴反射掩盖。

## 决策

1. 右腕保持当前 source local offset 单位四元数 `[0, 0, 0, 1]`。
2. 左腕 source frame 使用一次局部 180 度合法后旋转，把 X 轴从指尖到掌心翻转为
   掌心到指尖。最初实施并自动化验证的候选是绕局部 Y 轴 180 度
   `[0, 1, 0, 0]`；随后现场逐轴核对选择了绕局部 Z 轴 180 度：

   ```text
   left source local offset xyzw = [0, 0, 1, 0]
   right source local offset xyzw = [0, 0, 0, 1]
   semantic_left = aligned_raw_left * Rotation_z(pi)
   ```

   当前左腕四元数也可表述为“先绕局部 Y 轴 180 度，再绕旋转后的局部 X 轴
   180 度”；组合旋转与绕局部 Z 轴 180 度等价（四元数整体正负号等价）。该旋转
   同时翻转 X、Y 并保留 Z，保持右手系。禁止只把矩阵 X 列乘以 `-1`，因为
   那会产生 determinant 为 `-1` 的反射，不能表示合法腕姿态。
3. 三轴验收是该 offset 生效的最终条件。现场已经确认 `[0, 1, 0, 0]` 能修正左腕
   X 轴，继续逐轴核对后采用 `[0, 0, 1, 0]`，目前 BVH 与机器人腕坐标系基本
   对齐。后续实现和测试不得把数值回退为早期 Y 轴候选；配置注释、自动化测试和
   双语 README 必须同步为当前 Z 轴候选。
4. “raw BVH frame”和“semantic-normalized BVH frame”必须在命名和日志中分开：
   raw frame 保留源数据事实，semantic frame 才是 Pink 和用于与 G1 对比的视口轴。
5. Pink target、视口语义 BVH 腕轴和 `aligned_to_retarget` 验证必须复用同一个
   side-specific offset 路径，且 offset 只能应用一次。
6. 先修 frame 语义，再回到默认 80 度限位和 `0.75` orientation cost 评估 IK。
   cost=4、limit=90 只保留为诊断 A/B，不作为默认修复。

## 当前 Pink 权重基线

之前测试命令中的 `NOITOM_WRIST_ORIENTATION_COST=4.0` 只覆盖腕姿态权重。当前各项
基线为：

| 任务 | position cost | orientation cost |
| --- | ---: | ---: |
| 左右腕 | `18.0` | 默认 `0.75`；环境变量可覆盖为 `4.0` |
| 左右肘 | `9.0` | `0.0` |
| 左右肩 | `6.0` | `0.0` |
| Null-space posture | `0.05` | 不适用 |

所有 Pink frame task 当前使用 `gain=0.20`、`lm_damping=50.0`。因此 cost=4 的日志
不能代表发布默认值；坐标系现场确认完成后应使用 orientation cost `0.75`、腕
pitch/yaw 限位 `80` 度复测。

## 现场确认和后续边界

- 当前配置文件 `ik_config/noitom_to_g1.json` 的左腕 offset 已手动设为
  `[0.0, 0, 1.0, 0.0]`，右腕仍为 identity。
- 当前自动化测试仍含早期 `[0, 1, 0, 0]` 断言，JSON 注释也可能保留早期 Y 轴
  描述。下一位实现者必须同步这些预期，不得用测试失败为由恢复旧候选。
- 下一项独立工作是固定机器人下半身。骨盆/root 当前已固定，但继承的 Agile
  lower-body action 仍会控制髋、膝和踝，因此“骨盆不移动”不等于“下半身已
  固定”。腰关节继续属于 Pink 上半身 IK，不纳入腿部固定范围。

下半身边界和实施要求见
[`2026-07-24-noitom-fixed-lower-body.md`](2026-07-24-noitom-fixed-lower-body.md) 与
[`../architectures/NOITOM_FIXED_LOWER_BODY_PLAN.md`](../architectures/NOITOM_FIXED_LOWER_BODY_PLAN.md)。

## 被否定的判断

- `aligned_to_retarget_rot_deg=0` 不能证明解剖轴正确。
- 左腕贴住 pitch/yaw 边界是错误目标的后果之一，不是固定偏转的充分根因。
- 提高 orientation cost 会让 Pink 更努力追踪错误目标，并可能显著牺牲位置，不是
  坐标系修复。
- 左右腕 local offset 都为 identity 与这份镜像 BVH 的解剖轴语义不相容。

具体实施见
[`../architectures/LEFT_BVH_WRIST_SEMANTIC_FRAME_PLAN.md`](../architectures/LEFT_BVH_WRIST_SEMANTIC_FRAME_PLAN.md)。
