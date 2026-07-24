<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Noitom → G1 重定向重构计划（供执行 agent 使用）

**状态**：已执行（自动化验证完成；真实 Noitom/Isaac Lab 手感验证仍需硬件会话）。本文档本身不是产品文档，只是一份给另一个 coding agent
的可执行操作手册，完成重构后可以删除或归档。

**目标读者**：负责实际改代码的 agent。你不需要重新做架构决策——决策已经在下面写
清楚了；你的工作是照此实现、验证、提交。如果发现某个假设与代码实际情况不符，
先在本文件对应小节旁补一句"已核实：与假设不同，实际是 ……"，再继续，不要静默
改变方案范围。

---

## 0. 执行前必读（不可跳过）

1. 仓库根 [`AGENTS.md`](../../AGENTS.md) 的 "CRITICAL — mandatory preflight"：
   在改动或新建本次涉及目录下的任何文件之前，必须先用文件读取工具读完该目录及其
   所有祖先目录里的 `AGENTS.md`。本次改动只涉及 `examples/noitom/`（无自己的
   `AGENTS.md`），所以只需要读根 `AGENTS.md` 即可；如果后续扩大范围碰到
   `src/core/**`，必须额外读 `src/core/AGENTS.md` 以及对应子包的 `AGENTS.md`
   （例如 `src/core/live_trackers/AGENTS.md`、`src/core/deviceio_trackers/AGENTS.md`）。
2. 现有代码（改之前先看一遍，不要凭空脑补接口）：
   - `examples/noitom/README.md`
   - `examples/noitom/noitom_retargeting.py`（当前的单体重定向实现，本次重构的
     主要拆除对象）
   - `examples/noitom/noitom_tasks.py`（任务注册 + Pink IK 配置，消费
     `noitom_retargeting.py` 的输出）
   - `examples/noitom/noitom_reference_draw.py`（调试可视化，依赖
     `noitom_retargeting.py` 的一些内部函数，重构时要同步更新其 import）
   - `src/core/retargeting_engine/python/interface/base_retargeter.py`
     （`BaseRetargeter`/`connect()`/`RetargeterSubgraph` 的真实签名）
   - `src/core/retargeting_engine/python/interface/output_combiner.py`
     （`OutputCombiner` 用法）
   - `examples/retargeting/python/example_retargeters.py`、
     `examples/retargeting/python/sources_example.py`（仓库里"标准"小型
     Retargeter 节点写法的参考范例）
   - `/home/lenovo/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomanipulation/pick_place/locomanipulation_g1_env_cfg.py`
     （PICO 基线任务 `_build_g1_locomanipulation_pipeline`，本次要模仿的组织方式）
3. 外部参考（只读，不要复制代码，只借鉴设计）：
   - `/home/lenovo/GitRepo/GMR/general_motion_retargeting/motion_retarget.py`
     （声明式映射表 + 通用加权 IK 的参考实现）
   - `/home/lenovo/GitRepo/GMR/general_motion_retargeting/ik_configs/noi_to_g1.json`
     （已验证过的 Noitom→G1 全身关节映射表，四元数 offset 和权重可作为初始参数
     来源）
4. 提交前必须跑通仓库根 `AGENTS.md` 里的 "Pre-commit — match CI before you stop"：
   `SKIP=check-copyright-year pre-commit run --all-files`，本次改动是纯 Python，
   不涉及 C++，可以不跑 clang-format 步骤。所有 AI 起草的 commit 需要
   `git commit -s`（DCO sign-off）。

---

## 1. 背景（一句话版）

`examples/noitom` 现在用一个 ~2000 行的单体类 `NoitomG1Retargeter`
手写全部运动学（躯干系提取、骨长标定、姿态混合、前臂摆动/扭转分解等），
而仓库里作为对照的 PICO 基线任务
（`Isaac-PickPlace-Locomanipulation-G1-Abs-v0`）用的是标准的
"小 Retargeter 节点 + `.connect()` + `TensorReorderer`/`OutputCombiner`"
声明式组合方式；同时 GMR 项目已经有一份验证过的 Noitom→G1 全身关节声明式映射表
（`noi_to_g1.json`），用的是"人体关节→机器人 link 的位置/旋转权重表 + 自动骨长
比例 + 通用加权 IK"模式，比 Noitom 现在的手写几何公式更通用、更少参数。

本次重构分两个独立、可分别提交的阶段：

- **阶段 A（架构）**：把 `NoitomG1Retargeter` 拆成符合仓库惯例的多个小
  `BaseRetargeter` 节点，用 `.connect()` 组合，替换掉单体状态机类。
  **不改变任何数值行为**，纯重构，可以先做、先提交、先验证。
- **阶段 B（运动学）**：把手写几何公式（骨长标定、姿态混合、扭转分解等）替换
  为声明式关节映射表 + 更通用的缩放/IK 逻辑，参考 GMR 的 `ik_match_table`/
  `human_scale_table`/`auto_limb_scale` 设计。**这一步会改变数值行为**，需要
  仿真里实际跑一遍确认手感一致或更好。

两阶段务必分开提交（至少分两个 commit，建议分两个 PR），不要合并成一次大改动。

---

## 2. 范围声明

**本次改动范围（in scope）**：
- `examples/noitom/noitom_retargeting.py`（拆分/重写）
- `examples/noitom/noitom_tasks.py`（更新 import 和管线搭建代码以适配新节点）
- `examples/noitom/noitom_reference_draw.py`（同步更新对内部函数的依赖）
- `examples/noitom/README.md`（"Behavior" 一节要跟着改，描述新的管线结构）
- 新增：`examples/noitom/noitom_retargeter_nodes.py`（阶段 A 产出的新节点文件，
  命名可自行调整，但要在 README 里同步）
- 新增：`examples/noitom/ik_config/noitom_to_g1.json` 或等价的声明式配置
  （阶段 B 产出）
- 新增：`examples/noitom/tests/`（单元测试，纯 Python + numpy，不依赖 Isaac Lab）
- 新增：`examples/noitom/pyproject.toml`（给上面的测试提供隔离的 `uv` 环境，
  参考 `examples/retargeting/python/pyproject.toml`）

**明确不在本次范围内（out of scope，不要动）**：
- `src/plugins/noitom_mocap/**`（C++ 插件本身，数据格式转换逻辑不变）
- `src/core/**` 下任何文件（本次改动完全在 example 层完成；如果发现
  `BaseRetargeter`/`connect()` 缺某个能力导致做不到，先在本文件"未决问题"一节
  记录，不要顺手改 core）
- Pink IK 的 waist/wrist 安全限位逻辑（`noitom_tasks.py` 里
  `_build_noitom_pink_ik_action_class`），除非该函数因为阶段 A 的接口变化必须
  跟着改签名——如果需要改，只改调用方式，不改限位算法本身
- 下半身/移动策略（`AgileBasedLowerBodyActionCfg` 相关），本次重构只覆盖
  上肢 IK 目标计算

---

## 3. 阶段 A：架构重构（拆成可组合的 Retargeter 节点）

### 3.1 目标结构

参照 PICO 基线 `_build_g1_locomanipulation_pipeline` 的组织方式，把当前
`NoitomG1Retargeter._compute_fn` 里做的事情拆成若干个独立、职责单一、可单独
测试的 `BaseRetargeter` 子类，通过 `.connect()` 串起来，最终用
`OutputCombiner`（或继续用现有的 `NoitomG1ActionSource`，见 3.3）拼出
`noitom_tasks.py` 需要的 action tensor。

建议的节点划分（每个节点对应 `noitom_retargeting.py` 里现有的一段逻辑，
拆分时把对应的私有函数原样搬过去，先不改算法）：

1. **`NoitomTorsoFrameRetargeter`**
   - 输入：`full_body_tracked`（`DeviceIOFullBodyPoseTracked`）
   - 输出：躯干系（origin + 四元数）、pelvis world 位置、body yaw
   - 对应现有 `_build_torso_frame` / `_compute_torso_yaw` / `_parse_upper_body`
     中提取躯干部分的逻辑

2. **`NoitomArmCalibrationRetargeter`**（每侧一个实例，`side="left"/"right"`）
   - 输入：躯干系输出 + `full_body_tracked`
   - 输出：该侧手臂的标定状态（肩/肘/腕世界坐标、骨长、`wrist_rot_torso` 等）
   - 内部维护"是否已标定"状态和"清除标定"方法（对应现在
     `NoitomG1Retargeter.calibrate` / `clear_calibration` 里按侧处理的部分）
   - 对应现有 `_parse_arm` / `_ArmCalibration`

3. **`NoitomArmIkTargetRetargeter`**（每侧一个实例）
   - 输入：躯干系 + 标定状态（自己维护的，见上）+ `full_body_tracked`
   - 输出：该侧的 `wrist` / `elbow` / `shoulder` 三个 SE3 目标（7D pose 张量）
   - 对应现有 `_solve_wrist_target` / `_solve_elbow_target` / `_solve_shoulder_target`
     / `_arm_fk_robot_blended` / `_tracked_wrist_quaternion` 等一整条链路
   - 保留现有的平滑（`_smooth_pose`）、扭转限幅/展开（`_bound_wrist_twist`）逻辑，
     原样搬迁，不改数值

4. **`NoitomReferenceSkeletonRetargeter`**（可选，仅在调试可视化打开时用）
   - 输入：躯干系 + 标定状态
   - 输出：机器人比例下的参考骨架关节位置字典（给 `noitom_reference_draw.py`
     画青色骨架用）
   - 对应现有 `compute_robot_reference_positions` / `_aligned_skeleton_positions`

5. **`NoitomG1PipelineBuilder`**（替代 `build_noitom_g1_locomanipulation_pipeline`，
   函数名可以保持不变，内部实现改为组装上面 4 类节点）
   - 用 `.connect()` 把 1→2→3 串起来（每侧各一条链），必要时再接
     `TensorReorderer`（如果决定把输出打平成一个 tensor）或者继续沿用现有
     `NoitomG1ActionSource._compute_fn` 里手工拼 `np.ndarray` 的方式（见 3.3
     的取舍说明）。

> 拆分原则：**先只做"把函数搬进新的类方法里"级别的重构**，函数体内部逐行保持
> 不变（除了变量传递方式从"一个大 `_CalibrationState` dataclass"变成"节点间的
> tensor 输出/输入"）。不要在这一步顺手"优化"或"简化"算法——那是阶段 B 的事，
> 混在一起会让 diff 无法 review、也无法定位回归。

### 3.2 标定（calibration）状态怎么处理

当前 `NoitomG1Retargeter` 用 `self._calibration: _CalibrationState | None` 来
表示"是否已标定"，标定后的状态贯穿整个 retarget 调用。拆成多节点后：

- `NoitomArmCalibrationRetargeter` 自己持有 `_ArmCalibration | None` 成员变量，
  并暴露 `is_calibrated` / `awaiting_calibration` / `clear_calibration()` 方法
  （方法名对齐现有 API，方便 `noitom_tasks.py` 少改）。
- 每个 `BaseRetargeter._compute_fn` 都能拿到 `context.execution_events.reset`
  （参考现有 `noitom_retargeting.py` 里 `if context.execution_events.reset:
  self.clear_calibration()` 的用法），所以 reset 事件的处理可以原样下放到每个
  持有状态的节点里，不需要一个上层节点去广播 reset。
- **躯干标定和左右手臂标定是否需要同步完成**（现有实现是整体一次成功/失败）：
  拆分后如果分成独立节点，理论上会出现"左臂标定成功但右臂还没成功"的中间态。
  为了保持现有行为（整体校准，要么都成功要么都不成功），建议：
  - 方案 1（推荐，改动最小）：不拆"标定"和"IK 目标计算"到两个独立可外部触发
    的节点，而是把 `NoitomArmCalibrationRetargeter` 设计成
    **`NoitomArmIkTargetRetargeter` 的内部辅助类/方法**，而不是独立注册进管线
    的 `BaseRetargeter`。也就是说，"3 个手臂目标节点"（左手臂、右手臂）各自内部
    做自己的标定判断，标定成功的判定标准（8 个必需关节全部有效）保持和现有
    `NoitomG1Retargeter.calibrate` 一致，只是逻辑挪到了新类里而不是共享一个大
    state。这样可以避免引入新的"部分标定"边界情况。
  - 方案 2（更彻底但风险更高）：保留独立标定节点，但补一个"标定门"节点，仅当
    左右两侧都标定成功后才把 IK 目标节点从"输出上一帧 hold pose"切换到"输出新
    目标"。如果时间有限，先用方案 1。

### 3.3 `noitom_tasks.py` 里 `NoitomG1ActionSource` 怎么接新节点

现有 `NoitomG1ActionSource`（`examples/noitom/noitom_tasks.py` 约 717-978 行）
本身已经是一个 `IDeviceIOSource`，内部直接持有一个 `NoitomG1Retargeter` 实例并
手动调用 `.calibrate()` / `.retarget()`。拆分后两种接法都可以，按优先级：

1. **推荐**：`NoitomG1ActionSource` 内部改为持有拆分后的多个节点实例
   （躯干节点 + 左臂节点 + 右臂节点 + 可选参考骨架节点），在自己的
   `_compute_fn` 里按顺序调用 `.compute(...)`（不是走 `.connect()` 图，因为
   `NoitomG1ActionSource` 本身已经是手写的 `IDeviceIOSource`，混合"手写调用"
   和"图连接"两种风格在一个类内部是可以接受的，因为它对外仍然只暴露一个
   `action` tensor）。这样改动面最小：`NoitomG1ActionSource` 的
   `input_spec`/`output_spec`/整体行为完全不变，只是内部实现从"一个大
   retargeter"变成"几个小 retargeter 顺序调用"。
2. **可选，如果想要更彻底地贯彻"PICO 方式"**：把
   `NoitomG1ActionSource` 也拆掉，改成用 `.connect()` 把躯干/左臂/右臂节点接到
   一个 `TensorReorderer`（参照 `locomanipulation_g1_env_cfg.py` 里
   `_build_g1_locomanipulation_pipeline` 对 `TensorReorderer` 的用法），彻底
   走声明式管线。**这个选项工作量明显更大**（需要处理"没有数据时输出 hold
   pose"、"打印调试信息"、"驱动 `_NoitomReferenceVisualizer`"这些目前写在
   `NoitomG1ActionSource._compute_fn` 里的旁路逻辑，`TensorReorderer` 风格的图
   节点不适合直接做这些副作用），**除非用户明确要求，否则先做选项 1**。

### 3.4 阶段 A 的验证方法

> 已核实（2026-07-24）：本工作区的测试解释器来自
> `source ~/env_isaaclab/bin/activate`；不要使用未同步完整依赖的
> `examples/noitom/.venv`。运行 pytest 时还要设置
> `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`，避免加载系统 Python 3.10 下的 ROS
> `launch_testing` 插件。阶段 A golden 与阶段 B 配置测试均在该环境运行。

因为阶段 A 不改数值，验证目标是"重构前后行为完全一致"：

1. 新建 `examples/noitom/pyproject.toml`（参照
   `examples/retargeting/python/pyproject.toml`，依赖
   `isaacteleop[retargeters-lite]` 或等价 extra + `numpy` + `pytest` +
   `scipy`），以及 `examples/noitom/tests/conftest.py`。
2. 写一组**不依赖 Isaac Lab / Isaac Sim** 的纯 Python 单元测试：
   - 构造一个假的 `FullBodyPoseT`/`FullBodyPoseTrackedT`（或者直接
     用 `isaacteleop.schema` 里的 flatbuffer builder，参考
     `src/core/schema_tests/python/test_full_body.py` 怎么构造测试数据）
     模拟几个手臂姿态帧（比如：中立站姿、抬左臂 90 度、双臂交叉等）。
   - 用**重构前**的 `NoitomG1Retargeter`（先在 git 里 `git stash` 或者切到重构
     前的 commit 跑一遍，记录每帧的 `left_wrist`/`right_wrist`/`left_elbow`/
     `right_elbow`/`left_shoulder`/`right_shoulder` 七元数输出，存成一份
     "golden" 数据，可以直接写成 Python 字面量或 `.npz` 文件）。
   - 重构后跑同样的输入帧序列（含标定帧），断言输出与 golden 数据在数值上
     完全一致（`np.testing.assert_allclose`，绝对误差可以放到 `1e-9`，因为
     阶段 A 不应该引入任何浮点路径变化）。
3. 如果时间允许，额外跑一次真实的 Isaac Lab teleop（按 README 里 Example 2
   的命令），确认 UI 里的青色骨架、腕部坐标轴显示、Pink IK 目标位置和重构前
   目视一致（这一步无法自动化，跑一次、截图对比即可，不要求逐帧数值验证）。

---

## 4. 阶段 B：运动学重构（声明式关节映射表 + 通用缩放）

### 4.1 目标

把 `noitom_retargeting.py` 里下列手写几何逻辑：

- 骨长标定与缩放（`arm_scale`、`body_height_scale`、
  `robot_upper_arm_length`/`robot_forearm_length` 常量、
  `_human_robot_reach_scale`）
- 姿态混合与元素级的 FK（`_arm_fk_robot_blended`、`_slerp_unit_direction`、
  `_forearm_dir_from_elbow_angle`、`_bend_forearm_direction`、
  `_elbow_interior_angle`）
- 腕部朝向的摆动/扭转分解（`_tracked_wrist_quaternion`、
  `_wrist_twist_delta_rad`、`_source_forearm_swing_rotation`、
  `_shortest_arc_rotation`、`_signed_twist_rad`、`_unwrap_angle_near`）

替换/收敛为一份**声明式配置 + 少量通用求解代码**，参考
`/home/lenovo/GitRepo/GMR/general_motion_retargeting/ik_configs/noi_to_g1.json`
和 `general_motion_retargeting/motion_retarget.py` 的设计：

- 一份 `noitom_to_g1.json`（或 `.py` 字典，看哪种在 `isaacteleop` 里更方便打包），
  内容至少包含：
  - `human_root_name` / `robot_root_name`
  - 每个需要跟踪的关节对：`{ "robot_link": ["human_joint", pos_weight,
    rot_weight, pos_offset, rot_offset] }`，覆盖肩/肘/腕（这次不扩展到腿/脚，
    因为下半身仍由学习到的行走策略接管——这是和 GMR 全身重定向场景的**关键
    差异，不要照抄 GMR 的腿部/接触/自碰撞部分**）
  - 每个关节对的 `human_scale_table` 条目（人体骨段长度→机器人骨段长度的
    比例，替代现在硬编码的 `robot_upper_arm_length = 0.28` 等常量）
  - 四元数 offset 可以先直接抄 `noi_to_g1.json` 里对应肩/肘/腕的数值作为初始
    值，再结合实机测试微调（GMR 的坐标约定和本仓库的 Noitom→Isaac 坐标转换
    `_NOITOM_TO_ISAAC` 不完全一样，**不能直接假设数值可以 1:1 复用，必须先把
    两边的坐标系约定对齐后再迁移数值**，见 4.3）。
- 一段通用求解逻辑：给定标定帧和当前帧，对配置表里的每一条 `(human_joint,
  robot_link)` 计算「标定时的人体骨段方向 → 当前帧人体骨段方向」的最短旋转
  （即保留现有 `_shortest_arc_rotation` 的思路，这部分本来就是通用的，不需要
  重写），乘以按配置表算出的骨长比例，得到机器人 link 的目标位置；旋转目标用
  配置表里的四元数 offset 加残余扭转（复用现有 `_wrist_twist_delta_rad` /
  `_clamp_wrist_twist` 逻辑，这部分算法本身是对的，只是输入的 offset 常量要从
  硬编码改成配置表驱动）。

### 4.2 明确保留、不要删除的部分

- **在线标定（calibration-to-neutral-pose）流程本身**：GMR 面向离线 BVH、
  已知 `actual_human_height`，Noitom 面向实时遥操、身高未知，所以"等一个稳定
  姿态作为中立参考"这一步是本仓库场景特有的，必须保留（对应现有
  `NoitomG1Retargeter.calibrate` / `awaiting_calibration` 语义），只是标定
  产出的"骨长比例"和"关节映射"要改成读配置表而不是散落的常量。
- **腕部扭转限幅与展开**（`wrist_twist_limit_deg`、
  `wrist_twist_max_step_deg`、`_bound_wrist_twist` 里"选最近的等价角度再限幅
  再限步长"的逻辑）：这是 G1 硬件/Pink IK 特有的安全约束，GMR 的配置表里没有
  对应概念，不要因为"简化"而删掉。
- **平滑（position_smoothing / rotation_smoothing）**：同上，保留。
- **下半身**：继续由 `AgileBasedLowerBodyActionCfg` 学习策略驱动，配置表不要
  引入腿部/脚部映射（即使 GMR 的 `noi_to_g1.json` 里有，也不要照搬）。

### 4.3 坐标系对齐——阶段 B 最容易出错的地方

在直接复用 `noi_to_g1.json` 里的四元数 offset 数值之前，必须先确认两套坐标系
约定一致，否则数值会静默错误（不会报错，只会让手臂姿态"看起来差不多但有偏差"）：

- 本仓库：Noitom 原始数据是 Y-up，通过 `noitom_retargeting.py` 里的
  `_NOITOM_TO_ISAAC`（`[[-1,0,0],[0,0,1],[0,1,0]]`）转成 Isaac Z-up，再用
  `operator_faces_robot`（绕 Z 转 180°）让操作员面向机器人。
- GMR：用的是 mink/MuJoCo 的世界系约定，`ik_match_table` 里的四元数是
  "human 骨段局部系 → robot link 局部系"的旋转 offset，和 `_NOITOM_TO_ISAAC`
  不是同一层概念（GMR 的 offset 已经把 BVH 的骨骼局部坐标系差异吸收掉了，
  而本仓库现在的实现是先把位置整体转到 Isaac 系，再在 Isaac 系里做方向计算，
  没有 GMR 意义上的"每个关节一个局部 offset"）。
- **建议做法**：不要试图把 GMR 的 `rot_offset` 数值直接塞进本仓库的公式。
  而是：先按本仓库现有坐标约定（Isaac Z-up + `operator_faces_robot`）搭好配置
  表结构和求解逻辑框架，四元数 offset 先继续用
  `noitom_retargeting.py` 里现有的 `_DEFAULT_LEFT_WRIST_QUAT` /
  `_DEFAULT_RIGHT_WRIST_QUAT`（这两个已经在本仓库坐标系下验证过），只把"骨长
  比例"和"关节映射表结构"这两点借鉴 GMR，四元数 offset 留给实机调试阶段用
  `NOITOM_ORIENTATION_DEBUG=1`（参考 README "Wrist-orientation diagnostics"
  一节）逐个手臂对比调整。**如果调试后发现 GMR 的某个 offset 数值确实更准，
  再把它转换到本仓库坐标系后写回配置表，并在配置文件里用注释记录"来源:
  GMR noi_to_g1.json，已转换坐标系"，不要不加说明地直接抄数值。**

### 4.4 阶段 B 的验证方法

阶段 B 会改变数值行为，不能用阶段 A 的"golden 数据完全一致"标准，改为：

1. 单元测试层面：对几个典型姿态（中立站姿、单臂平举、双臂交叉、肘部弯曲）
   断言新公式给出的机器人手臂目标 **在合理误差范围内**（比如末端位置误差
   < 2cm，姿态误差 < 5°）与重构前的结果一致——如果某个姿态下两者差异明显，
   要能解释为什么新公式更合理（比如骨长比例来自配置表还是硬编码常量），不能
   只是"数值变了但不知道为什么"。
2. 实机/仿真验证：按 README Example 2 跑一遍 Isaac Lab teleop，覆盖
   README 里列的几个环境变量组合（默认 `twist` 模式、`forearm` 模式、
   `NOITOM_ORIENTATION_DEBUG=1`），确认：
   - 标定（举起手臂保持不动）能正常触发中立姿势采集；
   - 手臂能达到与真实操作幅度成比例的机器人姿态，没有明显的"够不到"或
     "过冲"；
   - `NoitomPinkJointDebug` 打印的 `solver_held` 不应长期卡在 1（说明目标
     没有被限位卡死）；
   - 肩/肘/腕三级 frame task 没有互相打架导致抖动。
3. 更新 `examples/noitom/README.md` 的 "Behavior" 一节，描述新的配置表驱动
   流程，并在文档里注明配置文件路径（`ik_config/noitom_to_g1.json` 之类），
   方便后续调参的人知道去哪改。

---

## 5. 提交计划（建议的 commit/PR 切分）

1. **PR/commit 1（阶段 A，架构重构，无数值变化）**
   - 新增 `noitom_retargeter_nodes.py`（或直接在 `noitom_retargeting.py` 内部
     重新组织，看拆分后文件大小决定要不要拆文件）
   - 更新 `noitom_tasks.py`、`noitom_reference_draw.py` 的调用方式
   - 新增 `examples/noitom/pyproject.toml` + `tests/`，带 golden-data 回归测试
   - 更新 `README.md` "Behavior" 一节的管线示意图（如果结构变了）
   - pre-commit 通过，golden 测试通过，附一次真实 Isaac Lab teleop 的截图/说明
     作为人工验证记录（可以放在 PR 描述里，不用进 commit）
2. **PR/commit 2（阶段 B，运动学重构，数值可能变化）**
   - 新增配置表文件
   - 重写目标求解逻辑消费配置表
   - 更新/新增相应的容差测试
   - PR 描述里明确写清楚"哪些数值会变、为什么变、怎么验证过没有变差"

不要把两个阶段合并成一个 PR——数值变化和纯重构混在一起会让 review 和之后的
`git bisect` 都很难做。

---

## 6. 未决问题 / 需要执行 agent 自行确认的点

- `BaseRetargeter.connect()` 要求所有非 optional 输入都在图连接时提供
  （见 `base_retargeter.py` 的 `connect()` 实现），如果某个新节点的某个输入
  在"未标定"阶段暂时没有意义（比如 IK 目标节点在标定完成前不需要目标),
  需要确认 `TensorGroupType` 是否支持声明成 optional，以及
  `OptionalTensorGroup.is_none` 的检查方式，用之前先读
  `src/core/retargeting_engine/python/interface/tensor_group_type.py` 和
  `tensor_group.py` 确认 API，不要凭本文档猜测的用法直接写代码。
- 3.3 节选项 1 和选项 2 的取舍：本文档默认建议选项 1（`NoitomG1ActionSource`
  内部手动调用小节点），因为改动面小；如果执行 agent 判断彻底转成
  `.connect()` 图更符合长期维护目标，需要先跟用户确认再做选项 2，不要默认
  升级范围。
- 阶段 B 的配置文件格式（JSON vs Python dict）：本文档建议先用 JSON（方便直接
  对照 GMR 的 `noi_to_g1.json` 做数值迁移和 diff），但要确认
  `isaacteleop` 的打包/CMake install 流程是否会自动把 `examples/noitom/` 下的
  非 `.py` 文件一起安装/复制到运行目录（参考 `plugin.yaml` 是怎么被
  `cmake --install` 处理的），否则配置文件可能在实际运行时找不到路径。

---

## 7. 执行核对（2026-07-24）

- 阶段 A 已在 `cdf4b7c` 提交：采用 3.2/3.3 推荐的低风险方案，将每臂平滑、
  扭转和目标状态抽到内部 `NoitomArmIkTargetNode`，保留
  `NoitomG1Retargeter`/`NoitomG1ActionSource` 外部接口；20 个 golden 测试验证
  配置关闭时的数值路径不变。
- 已核实：阶段 A 没有选择 3.3 的可选 `.connect()` 全图改造。该选项原计划要求
  先得到用户确认，因此不作为缺失项补做。
- 阶段 B 已接通默认任务：`NoitomG1Settings` 默认加载
  `ik_config/noitom_to_g1.json`，配置校验覆盖根节点、双臂链、六个映射、权重、
  offset、缩放表和物理限幅。
- 配置模式使用校准骨长、最短旋转、腕部扭转限幅和既有平滑；Pink 的肩/肘/腕
  位置权重也从同一映射表读取。配置关闭时只保留阶段 A 路径供 golden 回归。
- 阶段 B 新增 6 个纯 Python 测试，覆盖配置完整性、错误配置拒绝、左右骨长缩放、
  中立链长度、90 度抬臂和与旧 aligned-skeleton 路径之间可解释的 reach 差异。
- 自动化测试已在 `source ~/env_isaaclab/bin/activate` 后通过。真实 Noitom 数据、
  `forearm` 模式和 `NOITOM_ORIENTATION_DEBUG=1` 的仿真/硬件手感验证无法由无设备
  的自动化会话代替，仍需在 PR 验收时执行。
- JSON 通过 `Path(__file__)` 相对源码定位；此 example 由 README 中的
  `PYTHONPATH=.../examples/noitom` 直接运行，不依赖 CMake 安装非 Python 资源。
