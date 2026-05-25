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

当前 GUI 启用 `W` baseline 前进、`Shift` 较快前进、`S` 保守后退、`A/D` 较快原地左右转朝向，以及 `Q/E` 原地左右扭腰。

节点类为 `AsimoStyleZMPWalker`，节点名为 `asimo_style_zmp_walker`。它发布 `/legTargetJoints` 和 `/armTargetJoints`，并订阅 `/robot/ori`、`/robot/angVel`、`/robot/pos`、左右腿关节和左右臂关节反馈。

## 键盘遥操作 GUI

`mode=teleop_gui` 会启动 tkinter 图形界面。GUI 只负责写入键盘状态和选择 `MotionProfile`，不会直接发布 `/legTargetJoints`，底层仍然由 ASIMO-style ZMP walker 串联足步规划、ZMP/CoM、摆动脚、IK、稳定器、接触状态机、关节限幅和 ROS 2 发布。

| 按键 | 功能 | 当前状态 |
| --- | --- | --- |
| `W` | 前进 | enabled，加载当前 baseline forward profile；再按一次请求安全停步并站稳 |
| `Shift` | 较快前进 | enabled，加载当前验证通过的较快前进 profile；再按一次请求安全停步并站稳 |
| `S` | 后退 | enabled，加载 conservative backward profile；再按一次请求安全停步并站稳 |
| `A` | 左转朝向 | enabled，加载当前验证通过的 left-turn profile，约 10 步完成一轮转向 |
| `D` | 右转朝向 | enabled，加载与左转镜像的 right-turn profile，约 10 步完成一轮转向 |
| `Q` | 左拧腰 | enabled，站立时向左扭腰约 10 deg；再按一次回到中位 |
| `E` | 右拧腰 | enabled，站立时向右扭腰约 10 deg；再按一次回到中位 |
| `Space` | 暂停/恢复 | enabled |
| `R` | 清空按键状态，回到 idle | enabled |
| `Esc` | 紧急停止/安全保持 | enabled |

按键切换时，walker 会先完成当前安全步或站稳恢复，再加载新方向的参数。`W`、`Shift`、`S` 锁定后会连续追加对应方向的脚步；再次按当前方向键会完成当前安全步后进入站稳恢复。`A/D` 是有限步数原地转向；`Q/E` 不走步，只在站立状态下通过 IK 给骨盆一个小幅 yaw 目标。

## 当前按键绑定参数

README 只保留当前已经调试成功并实际绑定到按键的参数。

- `W` baseline forward: `step_length=0.045`, `step_width=0.09`, `step_time=1.75`, `double_support_time=0.55`, `transfer_time=0.85`, `touchdown_time=0.25`, `foot_clearance=0.045`, `swing_lift_fraction=0.28`, `swing_lower_fraction=0.30`, `zmp_kp=1.60`, `zmp_kd=1.00`, `max_joint_rate=1.45`, `max_arm_rate=1.20`, `max_com_speed=0.075`, `max_com_accel=0.20`, `total_steps=6`.
- `Shift` validated faster forward: `step_length=0.048`, `step_width=0.09`, `step_time=1.62`, `double_support_time=0.50`, `transfer_time=0.78`, `touchdown_time=0.28`, `foot_clearance=0.048`, `swing_lift_fraction=0.25`, `swing_lower_fraction=0.28`, `zmp_kp=1.65`, `zmp_kd=1.08`, `max_joint_rate=1.52`, `max_arm_rate=1.22`, `max_com_speed=0.082`, `max_com_accel=0.22`, `total_steps=6`.
- `S` backward: `step_length=0.032`, `step_width=0.095`, `step_time=1.95`, `double_support_time=0.68`, `transfer_time=0.96`, `touchdown_time=0.36`, `foot_clearance=0.050`, `swing_lift_fraction=0.32`, `swing_lower_fraction=0.38`, `zmp_kp=1.45`, `zmp_kd=1.25`, `max_joint_rate=1.20`, `max_arm_rate=1.00`, `max_com_speed=0.055`, `max_com_accel=0.13`, `total_steps=6`.
- `A` validated faster left turn: `step_length=0.0`, `step_width=0.106`, `step_time=1.84`, `double_support_time=0.64`, `transfer_time=0.90`, `touchdown_time=0.36`, `foot_clearance=0.055`, `swing_lift_fraction=0.31`, `swing_lower_fraction=0.35`, `turn_yaw_per_step=5.2 deg`, `zmp_kp=1.35`, `zmp_kd=1.42`, `max_joint_rate=1.08`, `max_arm_rate=0.90`, `max_com_speed=0.050`, `max_com_accel=0.115`, `total_steps=10`.
- `D` validated faster right turn: mirror of `A`, `turn_yaw_per_step=-5.2 deg`.
- `Q` waist left: `waist_yaw_target=10 deg`, `waist_yaw_rate=20 deg/s`, `max_joint_rate=0.85`, `max_arm_rate=0.80`.
- `E` waist right: mirror of `Q`, `waist_yaw_target=-10 deg`.

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
| `teleop_profiles.py` | 定义 `MotionProfile`，当前启用 baseline forward、validated faster forward、conservative backward、validated faster left/right turn，以及站立扭腰 profile。 |
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

### 2026.5.25

- 将 `A/D` 更新为当前验证通过的较快转向参数：相比早期 `turn_yaw_per_step=4.5 deg`、`step_width=0.108`、`step_time=2.00`、`double_support_time=0.72`、`transfer_time=1.00`、`touchdown_time=0.42`、`zmp_kp=1.30`、`zmp_kd=1.45`、`max_joint_rate=1.00`、`max_com_speed=0.045`、`total_steps=20` 的较慢转向设置，当前绑定改为 `turn_yaw_per_step=5.2 deg`、`step_width=0.106`、`step_time=1.84`、`double_support_time=0.64`、`transfer_time=0.90`、`touchdown_time=0.36`、`zmp_kp=1.35`、`zmp_kd=1.42`、`max_joint_rate=1.08`、`max_com_speed=0.050`、`total_steps=10`，在保持稳定的前提下更快完成朝向切换。
- 将 `Shift` 从占位键改为可执行的较快前进 profile：相比 `W` 的 baseline forward，当前 `Shift` 绑定把 `step_length` 从 `0.045` 提到 `0.048`，`step_time` 从 `1.75` 压缩到 `1.62`，`double_support_time` 从 `0.55` 压缩到 `0.50`，`transfer_time` 从 `0.85` 压缩到 `0.78`，`touchdown_time` 从 `0.25` 增到 `0.28`，`foot_clearance` 从 `0.045` 提到 `0.048`，`swing_lift_fraction` 从 `0.28` 调到 `0.25`，`swing_lower_fraction` 从 `0.30` 调到 `0.28`，并同步将 `zmp_kp` 从 `1.60` 提到 `1.65`、`zmp_kd` 从 `1.00` 提到 `1.08`、`max_joint_rate` 从 `1.45` 提到 `1.52`、`max_arm_rate` 从 `1.20` 提到 `1.22`、`max_com_speed` 从 `0.075` 提到 `0.082`、`max_com_accel` 从 `0.20` 提到 `0.22`，通过同时缩短步态时序、略增步长和提高 CoM / 关节响应上限，得到更快的前进响应。
