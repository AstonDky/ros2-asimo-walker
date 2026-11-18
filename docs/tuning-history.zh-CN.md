# 参数与开发记录

[English README](../README.md) · [中文 README](../README.zh-CN.md)

以下保留项目 2026 年 5 月原有调参及开发记录。文中的“验证通过”等说法对应当时的手动仿真记录，不是自动测试结论；当前参数以源码为准。完整版本与物理引擎设置未在原记录中保存。

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
