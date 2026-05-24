# CoppeliaSim Asti ROS 2 双足行走控制实验

[Screencast from 2026-05-18 18-39-14.webm](https://github.com/user-attachments/assets/54cd30d0-628e-40f6-b30a-78e2624fde47)

## 项目意图

这个项目的目标是驱动 CoppeliaSim 内置的 Asti 双足机器人进行从零开始的运动控制与行走控制实验。Asti 的外形和控制目标接近本田 ASIMO 这类传统双足机器人，因此项目主线不是简单播放固定关节序列，而是逐步搭建一个包含足步规划、ZMP/CoM 规划、摆动脚轨迹、腿部 IK、姿态稳定和状态机保护的传统双足行走控制栈。

长期目标是在 ROS 2 中实现键盘遥操作，通过 `W/A/S/D/Q/E` 等按键让仿真环境中的机器人平滑完成前进、后退、左转朝向、右转朝向、左扭动腰部、右扭动腰部等动作。目前重点已经推进到稳定双足行走：机器人能够按保守步态完成重心转移、抬脚、前摆、落脚、双脚支撑稳定和最终站立恢复。后续可以在传统控制栈基础上继续探索强化学习、MPC 或全身控制等方法。

当前从 ROS 2 话题接口、状态机、ZMP/CoM 轨迹、摆动脚轨迹到 IK 和稳定补偿都按项目需求逐步搭建

## 当前入口

ASIMO-style walker 的 ROS 2 可执行入口是：

```bash
ros2 run robot_simulation_experiment main.py
```

当前默认会启动键盘 GUI 遥操作。普通自动前进 `walk` 模式使用：

```bash
ros2 run robot_simulation_experiment main.py --ros-args -p mode:=walk
```

也可以显式启动 GUI：

```bash
ros2 run robot_simulation_experiment main.py --ros-args -p mode:=teleop_gui
```

当前 GUI 启用 `W` 前进、`S` 保守后退、`A/D` 原地左右转朝向，以及 `Q/E` 原地左右扭腰，其余动作只保留 profile 结构和界面状态，不会驱动机器人执行未调动作。

节点类为 `AsimoStyleZMPWalker`，节点名为 `asimo_style_zmp_walker`。它发布 `/legTargetJoints` 和 `/armTargetJoints`，并订阅 `/robot/ori`、`/robot/angVel`、`/robot/pos`、左右腿关节和左右臂关节反馈。

## 键盘遥操作 GUI

`mode=teleop_gui` 会启动 tkinter 图形界面。GUI 只负责写入键盘状态和选择 `MotionProfile`，不会直接发布 `/legTargetJoints`，底层仍然由 ASIMO-style ZMP walker 串联足步规划、ZMP/CoM、摆动脚、IK、稳定器、接触状态机、关节限幅和 ROS 2 发布。

| 按键 | 功能 | 当前状态 |
| --- | --- | --- |
| `W` | 前进 | enabled，加载当前 baseline forward profile；再按一次请求安全停步并站稳 |
| `S` | 后退 | enabled，加载 conservative backward profile；再按一次请求安全停步并站稳 |
| `A` | 左转朝向 | enabled，加载 faster left-turn profile，约 20 步从面向前方转到面向左方 |
| `D` | 右转朝向 | enabled，加载 faster right-turn profile，约 20 步从面向前方转到面向右方 |
| `Q` | 左拧腰 | enabled，站立时向左扭腰约 10 deg；再按一次回到中位 |
| `E` | 右拧腰 | enabled，站立时向右扭腰约 10 deg；再按一次回到中位 |
| `Shift` | 加速修饰 | reserved / disabled |
| `Space` | 暂停/恢复 | enabled |
| `R` | 清空按键状态，回到 idle | enabled |
| `Esc` | 紧急停止/安全保持 | enabled |

按下 disabled profile 时，GUI 会显示 reserved but disabled / 未配置，walker 保持 idle 或安全保持，不会修改转向、腰部 yaw、手臂动作或 CoM 参数。`W` 或 `S` 锁定后会连续追加对应方向的脚步，不再受普通 `walk` 模式 6 步上限限制；再次按当前方向键会完成当前安全步后进入站稳恢复。`A/D` 是有限步数原地转向，完成约 90 度朝向变化后进入站稳恢复。`Q/E` 不走步，只在站立状态下通过 IK 给骨盆一个小幅 yaw 目标，脚保持原地支撑；再次按当前键或按 `R` 会回到中位。如果在前进、后退、转向和扭腰之间切换，walker 会先完成当前安全步或站稳恢复，再加载新方向的参数。

## 按键 Profile 快照

当前 active benchmark 如下，已经在一次失败的激进提速试验后恢复为这些值。

- `W` benchmark: `step_length=0.045`, `step_width=0.09`, `step_time=1.75`, `double_support_time=0.55`, `transfer_time=0.85`, `touchdown_time=0.25`, `foot_clearance=0.045`, `swing_lift_fraction=0.28`, `swing_lower_fraction=0.30`, `zmp_kp=1.6`, `zmp_kd=1.0`, `max_joint_rate=1.45`, `max_arm_rate=1.2`, `max_com_speed=0.075`, `max_com_accel=0.20`.
- `S` benchmark: `step_length=0.032`, `step_width=0.095`, `step_time=1.95`, `double_support_time=0.68`, `transfer_time=0.96`, `touchdown_time=0.36`, `foot_clearance=0.050`, `swing_lift_fraction=0.32`, `swing_lower_fraction=0.38`, `zmp_kp=1.45`, `zmp_kd=1.25`, `max_joint_rate=1.20`, `max_arm_rate=1.0`, `max_com_speed=0.055`, `max_com_accel=0.13`.
- `A` benchmark: `step_length=0.0`, `step_width=0.108`, `step_time=2.00`, `double_support_time=0.72`, `transfer_time=1.00`, `touchdown_time=0.42`, `foot_clearance=0.054`, `swing_lift_fraction=0.35`, `swing_lower_fraction=0.40`, `turn_yaw_per_step=4.5 deg`, `zmp_kp=1.30`, `zmp_kd=1.45`, `max_joint_rate=1.00`, `max_arm_rate=0.85`, `max_com_speed=0.045`, `max_com_accel=0.10`, `total_steps=20`.
- `D` benchmark: mirror of `A` benchmark, `turn_yaw_per_step=-4.5 deg`.
- `Q` benchmark: `waist_yaw_target=10 deg`, `waist_yaw_rate=20 deg/s`, `max_joint_rate=0.85`, `max_arm_rate=0.8`.
- `E` benchmark: mirror of `Q` benchmark, `waist_yaw_target=-10 deg`.
- failed aggressive trial note: the faster forward retune from 2026-05-24 fell after about 3 corridor forward steps and was rolled back.

## 关键参数

参数集中在 `src/robot_simulation_experiment/scripts/asimo_walker/common.py` 的 `WalkerParams` 数据类。baseline 参数表放在后面的“代码构建历史”部分，这里只说明各参数作用。

| 参数 | 调节接口 | 对机器人影响 |
| --- | --- | --- |
| `step_length` | `common.py: WalkerParams.step_length` | 单步前进距离。增大会走得更远，但摆腿幅度和重心转移压力更大，更容易横滚或触地不稳。 |
| `step_width` | `common.py: WalkerParams.step_width` | 左右脚横向间距。增大通常更稳但姿态更宽、转移更慢；减小更自然但单脚支撑风险更高。 |
| `step_time` | `common.py: WalkerParams.step_time` | 单脚摆动持续时间。增大更慢更稳；减小动作更快但更容易甩动和跌倒。 |
| `double_support_time` | `common.py: WalkerParams.double_support_time` | 双脚支撑落脚后的稳定时间。增大有利于站稳和恢复，减小会让步态更连续但风险更高。 |
| `foot_clearance` | `common.py: WalkerParams.foot_clearance` | 摆动脚抬脚高度。增大能减少拖地，但会提高单脚支撑扰动；减小更稳但可能扫地。 |
| `pelvis_height` | `common.py: WalkerParams.pelvis_height` | 骨盆高度，也是 LIPM/ZMP 近似里的质心高度。降低会增加屈膝稳定性但更费关节行程；升高姿态更直但容错变小。 |
| `support_zmp_margin` | `common.py: WalkerParams.support_zmp_margin` | 单脚支撑时 ZMP 相对支撑脚中心的横向偏置。适当增大可给高摆腿留支撑余量，过大会导致横向倾倒。 |
| `zmp_kp` / `zmp_kd` | `common.py: WalkerParams.zmp_kp`, `common.py: WalkerParams.zmp_kd` | 控制 CoM 朝目标 ZMP 移动的跟踪强度和阻尼。 |
| `transfer_time` / `touchdown_time` | `common.py: WalkerParams.transfer_time`, `common.py: WalkerParams.touchdown_time` | 控制抬脚前重心转移和落脚确认的从容程度。 |
| `swing_lift_fraction` / `swing_lower_fraction` | `common.py: WalkerParams.swing_lift_fraction`, `common.py: WalkerParams.swing_lower_fraction` | 控制摆腿周期中抬脚和落脚阶段分配。 |
| `max_joint_rate` / `max_com_speed` / `max_com_accel` | `common.py: WalkerParams.max_joint_rate`, `common.py: WalkerParams.max_com_speed`, `common.py: WalkerParams.max_com_accel` | 控制关节和 CoM 命令变化的激进程度。 |
| `stable_pitch` / `stable_roll` / `abort_tilt` | `common.py: WalkerParams.stable_pitch`, `common.py: WalkerParams.stable_roll`, `common.py: WalkerParams.abort_tilt` | 控制状态切换稳定阈值和安全中止阈值。 |

## 控制方法与代码结构

当前控制方法是保守的 ASIMO-style 传统双足控制链：先规划落脚点，再生成 ZMP 参考和 CoM 轨迹，摆动脚按平滑轨迹抬起和落下，腿部 IK 解算关节角，IMU/角速度反馈通过踝和髋做稳定补偿，最后经过状态机、安全检查、关节限幅和关节速度限制后发布到 ROS 2。

代码结构位于 `src/robot_simulation_experiment/scripts/asimo_walker/`：

| 文件 | 原理/职责 |
| --- | --- |
| `__init__.py` | Python 包初始化文件。 |
| `common.py` | 定义通用数据结构、物理常量、插值/限幅函数、`WalkerParams` 参数表和反馈数据结构。 |
| `footstep_planner.py` | 足步规划器。根据 `step_length`、`step_width`、`total_steps`、`sagittal_sign` 和 `turn_yaw_per_step` 生成左右脚交替的目标落脚点。 |
| `zmp_reference.py` | ZMP 参考规划器。按状态机阶段生成双脚中心、支撑脚或触地过渡的 ZMP 目标，保证抬脚前先把重心转移到支撑脚。 |
| `zmp_preview.py` | 简化 ZMP preview / LIPM CoM planner。用 `pelvis_height` 近似倒立摆高度，用 `zmp_kp`、`zmp_kd`、速度/加速度限幅生成平滑 CoM 轨迹。 |
| `swing_foot.py` | 摆动脚轨迹规划器。用 smoothstep 插值生成前摆，同时按 `foot_clearance`、`swing_lift_fraction`、`swing_lower_fraction` 控制抬脚、空中保持和落脚。 |
| `leg_ik.py` | 腿部逆运动学。根据骨盆位姿和左右脚目标位姿求 6 自由度腿关节，并执行关节角限制。 |
| `stabilizer.py` | 闭环稳定器。使用 `/robot/ori` 和 `/robot/angVel` 的 pitch/roll 反馈，为踝、髋和手臂添加小幅补偿，并给下一步落脚点提供保守修正。 |
| `contact_state_machine.py` | 接触与步态状态机。执行 `CROUCH -> TRANSFER -> SWING -> TOUCHDOWN -> DOUBLE_SUPPORT -> STAND/DONE`，并在足底力缺失时使用相位回退逻辑。 |
| `teleop_command.py` | GUI 与 ROS 控制循环之间的线程安全键盘状态缓冲。 |
| `teleop_profiles.py` | 定义 `MotionProfile`，当前启用 idle、forward baseline、conservative backward profile、较快一些的左右转 profile，以及站立扭腰 profile。 |
| `teleop_gui.py` | tkinter 键盘遥操作界面，显示按键、profile、pause、emergency stop 和 walker 状态。 |
| `main.py` | ROS 2 节点主入口。负责订阅反馈、串联完整控制链、发布关节目标和最终站立恢复。 |


## 代码构建历史

### 2026.5.17

- 实现项目基本代码架构，开始从 ROS 2 控制 CoppeliaSim Asti。
- 明确了后续目标：从固定动作序列逐步升级到可遥操作的平滑双足控制。

### 2026.5.18

- 建立模块化 `asimo_walker` 控制栈，并将入口安装为 `main.py`。
- 修正 CoppeliaSim 场景中的机器人前进方向，当前期望世界/地图方向为负 Y，内部使用 `sagittal_sign = -1.0`。
- 完成保守 ZMP/CoM 步态、摆腿轨迹、IK、稳定器、接触状态机、关节限幅、关节速度限幅和最终站立恢复。
- 在普通 `walk` 模式下完成 6 步行走，进入 `STAND` 后恢复到初始站姿，再进入 `DONE` 并保持直立。

当前 baseline 参数：

| 参数 | baseline 值 |
| --- | ---: |
| `step_length` | `0.045` |
| `step_width` | `0.09` |
| `step_time` | `1.75` |
| `double_support_time` | `0.55` |
| `foot_clearance` | `0.045` |
| `pelvis_height` | `0.48` |
| `total_steps` | `6` |
| `sagittal_sign` | `-1.0` |
| `turn_yaw_per_step` | `0.0` |
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

### 2026.5.19

- 增加 `mode=teleop_gui` 键盘遥操作框架。
- 新增 `MotionProfile` 架构和线程安全按键缓冲；第一版只启用 `W` 前进，复用现有 baseline 控制栈。
- `S/A/D/Q/E/Shift` 只作为 reserved / disabled profile 显示，不产生真实运动。

### 2026.5.22

- 启用 `S` 保守后退 profile，仍复用足步规划、ZMP/CoM、摆动脚、IK、稳定器、接触状态机和关节限幅控制链。
- 启用 `A` 更快一点的左转 profile，通过小角度原地转向脚步让机器人从面向前方逐步转到面向左方。
- 启用与左转镜像的 `D` 右转 profile，让机器人从面向前方逐步转到面向右方。
- 启用 `Q/E` 站立扭腰 profile，通过固定双脚支撑下的小幅骨盆 yaw 命令实现左右扭腰。
- `Shift` 继续作为 reserved / disabled profile 显示，不产生真实运动。

### 2026.5.24

- 把 `W/S/A/D/Q/E` 当时的全部已绑定 profile 参数作为 baseline 快照保存，便于后续继续做人工对比。
- 将 `W` 的默认步态提速到更激进的前进 baseline，同时同步提高 `zmp_kp/zmp_kd`、关节速率和 CoM 速度上限。
- 将 `S` 后退 profile 从保守设置推进到更短周期、更快重心转移和更高关节/CoM 响应。
- 将 `A/D` 原地转向每步 yaw 从 `4.5 deg` 提到 `6.5 deg`，同时把总步数从 `20` 降到 `14`，明显提高朝向切换速度。
- 将 `Q/E` 站立扭腰目标从 `10 deg` 提到 `14 deg`，扭腰速率从 `20 deg/s` 提到 `42 deg/s`，让站立姿态响应更直接。
- 该组激进参数在前进走廊实测约 3 步后跌倒，因此全部 key-bound profile 与默认步态已恢复到 benchmark。
