# CoppeliaSim Asti ROS 2 双足行走控制实验

[Uploading Screencast from 2026-05-18 18-39-14.webm…]()

## 项目意图

这个项目的目标是驱动 CoppeliaSim 内置的 Asti 双足机器人进行从零开始的运动控制与行走控制实验。Asti 的外形和控制目标接近本田 ASIMO 这类传统双足机器人，因此项目主线不是简单播放固定关节序列，而是逐步搭建一个包含足步规划、ZMP/CoM 规划、摆动脚轨迹、腿部 IK、姿态稳定和状态机保护的传统双足行走控制栈。

长期目标是在 ROS 2 中实现键盘遥操作，通过 `W/A/S/D/Q/E` 等按键让仿真环境中的机器人平滑完成前进、后退、左转朝向、右转朝向、左扭动腰部、右扭动腰部等动作。目前重点已经推进到稳定双足行走：机器人能够按保守步态完成重心转移、抬脚、前摆、落脚、双脚支撑稳定和最终站立恢复。后续可以在传统控制栈基础上继续探索强化学习、MPC 或全身控制等方法。

## 当前入口

ASIMO-style walker 的 ROS 2 可执行入口是：

```bash
ros2 run robot_simulation_experiment asimo_style_zmp_walker
```

自动调参模式入口是：

```bash
ros2 run robot_simulation_experiment asimo_style_zmp_walker --ros-args -p mode:=auto_tune
```

节点类为 `AsimoStyleZMPWalker`，节点名为 `asimo_style_zmp_walker`。它发布 `/legTargetJoints` 和 `/armTargetJoints`，并订阅 `/robot/ori`、`/robot/angVel`、`/robot/pos`、左右腿关节和左右臂关节反馈。

## 当前可调参数

主要参数集中在 `src/robot_simulation_experiment/scripts/asimo_walker/common.py` 的 `WalkerParams` 数据类。调参时优先改这里的默认值；如果只是自动调参候选组合，改 `src/robot_simulation_experiment/scripts/asimo_walker/main.py` 里的 `_build_profiles()`。

| 参数 | 当前值 | 调节接口 | 对机器人影响 |
| --- | ---: | --- | --- |
| `step_length` | `0.045` | `common.py: WalkerParams.step_length`；自动调参候选在 `main.py: _build_profiles()` | 单步前进距离。增大会走得更远，但摆腿幅度和重心转移压力更大，更容易横滚或触地不稳。 |
| `step_width` | `0.09` | `common.py: WalkerParams.step_width` | 左右脚横向间距。增大通常更稳但姿态更宽、转移更慢；减小更自然但单脚支撑风险更高。 |
| `step_time` | `1.75` | `common.py: WalkerParams.step_time`；`main.py: _build_profiles()` | 单脚摆动持续时间。增大更慢更稳；减小动作更快但更容易甩动和跌倒。 |
| `double_support_time` | `0.55` | `common.py: WalkerParams.double_support_time`；`main.py: _build_profiles()` | 双脚支撑落脚后的稳定时间。增大有利于站稳和恢复，减小会让步态更连续但风险更高。 |
| `foot_clearance` | `0.045` | `common.py: WalkerParams.foot_clearance`；`main.py: _build_profiles()` | 摆动脚抬脚高度。增大能减少拖地，但会提高单脚支撑扰动；减小更稳但可能扫地。 |
| `pelvis_height` | `0.48` | `common.py: WalkerParams.pelvis_height`；`main.py: _build_profiles()` | 骨盆高度，也是 LIPM/ZMP 近似里的质心高度。降低会增加屈膝稳定性但更费关节行程；升高姿态更直但容错变小。 |
| `total_steps` | `6` | `common.py: WalkerParams.total_steps` | 计划总步数。增大可连续走更久；调试时减小可以更快验证起步和落脚。 |
| `sagittal_sign` | `-1.0` | `common.py: WalkerParams.sagittal_sign` | 内部矢状方向符号。当前 CoppeliaSim 场景里机器人期望前进方向是世界坐标负 Y，因此保持 `-1.0`。 |
| `support_zmp_margin` | `0.004` | `common.py: WalkerParams.support_zmp_margin` | 单脚支撑时 ZMP 相对支撑脚中心的横向偏置。适当增大可给高摆腿留支撑余量，过大会导致横向倾倒。 |
| `zmp_preview_time` | `0.8` | `common.py: WalkerParams.zmp_preview_time` | ZMP 预瞄时间尺度。更长会让 CoM 提前响应，过长可能滞后；更短响应快但可能抖。 |
| `zmp_kp` | `1.6` | `common.py: WalkerParams.zmp_kp`；`main.py: _build_profiles()` | CoM 朝目标 ZMP 移动的比例增益。增大跟踪更紧，过大易摆动；减小更柔和但可能重心不到位。 |
| `zmp_kd` | `1.0` | `common.py: WalkerParams.zmp_kd`；`main.py: _build_profiles()` | CoM 速度阻尼。增大可抑制速度和超调，过大会走不动；减小响应更快但更容易晃。 |
| `ankle_pitch_kp` | `0.35` | `common.py: WalkerParams.ankle_pitch_kp`；`main.py: _build_profiles()` | 俯仰方向踝关节稳定补偿。增大抗前后倾能力，过大可能脚踝抖动。 |
| `ankle_roll_kp` | `0.35` | `common.py: WalkerParams.ankle_roll_kp`；`main.py: _build_profiles()` | 横滚方向踝关节稳定补偿。增大抗左右倾能力，过大可能单脚支撑时反复摆。 |
| `hip_pitch_kp` | `0.18` | `common.py: WalkerParams.hip_pitch_kp` | 俯仰方向髋关节稳定补偿。用于配合踝关节保持躯干，过大时腿部动作会变硬。 |
| `hip_roll_kp` | `0.18` | `common.py: WalkerParams.hip_roll_kp` | 横滚方向髋关节稳定补偿。用于单脚支撑横向平衡，过大容易造成跨步侧摆。 |
| `crouch_time` | `1.5` | `common.py: WalkerParams.crouch_time` | 起步下蹲/进入准备姿态时间。增大起步更缓，减小起步更快但容易冲击。 |
| `transfer_time` | `0.85` | `common.py: WalkerParams.transfer_time` | 双脚支撑时重心转移到支撑脚的时间。增大更稳，减小更快但容易在抬脚前重心不到位。 |
| `touchdown_time` | `0.25` | `common.py: WalkerParams.touchdown_time` | 摆动脚触地确认时间。增大落脚更谨慎，减小切换更快但接触不充分时风险更高。 |
| `stand_time` | `2.0` | `common.py: WalkerParams.stand_time` | 最后一步后从行走姿态恢复到初始站立姿态的时间。增大恢复更平滑，减小会更快但可能像“弹回”。 |
| `swing_lift_fraction` | `0.28` | `common.py: WalkerParams.swing_lift_fraction` | 摆腿周期中抬脚阶段占比。增大抬脚更慢，减小更快达到高点。 |
| `swing_lower_fraction` | `0.30` | `common.py: WalkerParams.swing_lower_fraction` | 摆腿周期中落脚阶段占比。增大落脚更慢，减小会更快下落。 |
| `dt` | `0.02` | `common.py: WalkerParams.dt` | 控制循环周期，约 50 Hz。改动会影响定时器、限速和所有时序计算。 |
| `max_joint_rate` | `1.45` | `common.py: WalkerParams.max_joint_rate` | 腿部关节目标角速度上限。增大动作更快但冲击更大；减小更柔和但可能跟不上步态。 |
| `max_arm_rate` | `1.2` | `common.py: WalkerParams.max_arm_rate` | 手臂关节目标角速度上限。影响手臂平衡摆动的快慢。 |
| `max_com_speed` | `0.075` | `common.py: WalkerParams.max_com_speed` | CoM 平面移动速度上限。增大可更快转移重心但更容易超调；减小更稳但步态慢。 |
| `max_com_accel` | `0.20` | `common.py: WalkerParams.max_com_accel` | CoM 平面加速度上限。增大响应更快但晃动更强；减小更平滑但可能无法及时到支撑脚上。 |
| `stable_pitch` | `5 deg` | `common.py: WalkerParams.stable_pitch` | 状态机允许进入下一阶段的俯仰稳定阈值。放宽会更容易继续走，收紧会更保守。 |
| `stable_roll` | `6 deg` | `common.py: WalkerParams.stable_roll` | 状态机允许进入下一阶段的横滚稳定阈值。放宽会减少等待，收紧会更稳但可能卡住。 |
| `abort_tilt` | `18 deg` | `common.py: WalkerParams.abort_tilt` | 跌倒/危险倾角中止阈值。调大风险更高，调小更安全但可能过早 ABORT。 |
| `mode` | `walk` | ROS 参数：`main.py` 中 `declare_parameter("mode", "walk")` | `walk` 为正常行走，`auto_tune` 会按 `_build_profiles()` 逐个试参数并写入调参记录。 |

自动调参候选 profile 在 `main.py` 的 `_build_profiles()` 中，目前包括 `front_minus_y_high_lift`、`front_minus_y_slower_hold`、`front_minus_y_longer_step`、`front_minus_y_medium_step`、`front_minus_y_extra_clearance`、`front_minus_y_low_pelvis`、`front_minus_y_more_damped`、`front_minus_y_ankle_soft`。

## 控制方法与代码结构

当前控制方法是保守的 ASIMO-style 传统双足控制链：先规划落脚点，再生成 ZMP 参考和 CoM 轨迹，摆动脚按平滑轨迹抬起和落下，腿部 IK 解算关节角，IMU/角速度反馈通过踝和髋做稳定补偿，最后经过状态机、安全检查、关节限幅和关节速度限制后发布到 ROS 2。

代码结构位于 `src/robot_simulation_experiment/scripts/asimo_walker/`：

| 文件 | 原理/职责 |
| --- | --- |
| `__init__.py` | Python 包初始化文件。 |
| `common.py` | 定义通用数据结构、物理常量、插值/限幅函数、`WalkerParams` 参数表和反馈数据结构。 |
| `footstep_planner.py` | 足步规划器。根据 `step_length`、`step_width`、`total_steps`、`sagittal_sign` 生成左右脚交替的目标落脚点。 |
| `zmp_reference.py` | ZMP 参考规划器。按状态机阶段生成双脚中心、支撑脚或触地过渡的 ZMP 目标，保证抬脚前先把重心转移到支撑脚。 |
| `zmp_preview.py` | 简化 ZMP preview / LIPM CoM planner。用 `pelvis_height` 近似倒立摆高度，用 `zmp_kp`、`zmp_kd`、速度/加速度限幅生成平滑 CoM 轨迹。 |
| `swing_foot.py` | 摆动脚轨迹规划器。用 smoothstep 插值生成前摆，同时按 `foot_clearance`、`swing_lift_fraction`、`swing_lower_fraction` 控制抬脚、空中保持和落脚。 |
| `leg_ik.py` | 腿部逆运动学。根据骨盆位姿和左右脚目标位姿求 6 自由度腿关节，并执行关节角限制。 |
| `stabilizer.py` | 闭环稳定器。使用 `/robot/ori` 和 `/robot/angVel` 的 pitch/roll 反馈，为踝、髋和手臂添加小幅补偿，并给下一步落脚点提供保守修正。 |
| `contact_state_machine.py` | 接触与步态状态机。执行 `CROUCH -> TRANSFER -> SWING -> TOUCHDOWN -> DOUBLE_SUPPORT -> STAND/DONE`，并在足底力缺失时使用相位回退逻辑。 |
| `main.py` | ROS 2 节点主入口。负责订阅反馈、串联完整控制链、发布关节目标、自动调参、写调参日志和最终站立恢复。 |


## 代码构建历史

### 2026.5.17

- 实现项目基本代码架构，开始从 ROS 2 控制 CoppeliaSim Asti。
- 明确了后续目标：从固定动作序列逐步升级到可遥操作的平滑双足控制。

### 2026.5.18

- 建立模块化 `asimo_walker` 控制栈，并将入口安装为 `asimo_style_zmp_walker`。
- 修正 CoppeliaSim 场景中的机器人前进方向，当前期望世界/地图方向为负 Y，内部使用 `sagittal_sign = -1.0`。
- 完成保守 ZMP/CoM 步态、摆腿轨迹、IK、稳定器、接触状态机、关节限幅、关节速度限幅和最终站立恢复。
- 在普通 `walk` 模式下完成 6 步行走，进入 `STAND` 后恢复到初始站姿，再进入 `DONE` 并保持直立。

2026.5.18 正常走路验证时的当前默认参数：

| 参数 | 值 |
| --- | ---: |
| `step_length` | `0.045` |
| `step_width` | `0.09` |
| `step_time` | `1.75` |
| `double_support_time` | `0.55` |
| `foot_clearance` | `0.045` |
| `pelvis_height` | `0.48` |
| `total_steps` | `6` |
| `sagittal_sign` | `-1.0` |
| `support_zmp_margin` | `0.004` |
| `zmp_preview_time` | `0.8` |
| `zmp_kp` | `1.6` |
| `zmp_kd` | `1.0` |
| `ankle_pitch_kp` | `0.35` |
| `ankle_roll_kp` | `0.35` |
| `hip_pitch_kp` | `0.18` |
| `hip_roll_kp` | `0.18` |
| `crouch_time` | `1.5` |
| `transfer_time` | `0.85` |
| `touchdown_time` | `0.25` |
| `stand_time` | `2.0` |
| `swing_lift_fraction` | `0.28` |
| `swing_lower_fraction` | `0.30` |
| `dt` | `0.02` |
| `max_joint_rate` | `1.45` |
| `max_arm_rate` | `1.2` |
| `max_com_speed` | `0.075` |
| `max_com_accel` | `0.20` |
| `stable_pitch` | `5 deg` |
| `stable_roll` | `6 deg` |
| `abort_tilt` | `18 deg` |
<video src="./videos/demo.mp4" controls width="600"></video>
