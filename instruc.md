
## 参数记录与对比要求

后续每次修改 ASIMO-style walker 参数后，都必须在本文件追加一条记录。记录不能只写新参数，还要写清楚“相比上一组参数具体提升了什么”。提升描述要尽量量化，例如：是否减少 ABORT、最大 pitch/roll 降低多少、完成步数从多少变成多少、前进距离增加多少、最终 `STAND/DONE` 是否更稳、是否减少脚拖地或落脚冲击。

建议记录格式：

| field | value |
| --- | --- |
| changed_parameters | `step_length: 0.055 -> 0.045`, `support_zmp_margin: 0.012 -> 0.004` |
| previous_result | 写上一组参数的完成步数、最大 pitch/roll、是否 ABORT、前进距离 |
| new_result | 写新参数的完成步数、最大 pitch/roll、是否 ABORT、前进距离 |
| concrete_improvement | 具体说明提升，例如“由第 2 步 ABORT 改为完成 6 步并进入 DONE；最大 roll 从约 22 deg 降到约 1.28 deg” |
| tradeoff | 写清楚代价，例如“步幅变小、速度变慢，但稳定性提高” |

如果没有检测到 CoppeliaSim 必需 ROS 2 话题，只能记录为尝试失败，不能记录为成功参数。

## Asti ASIMO-style ZMP walker tuning log

### Auto tune attempt - 2026-05-18 15:02:44

- workspace: `/home/astondky/Desktop/robot_simulation_experiment_ws`
- package: `robot_simulation_experiment`
- executable: `asimo_style_zmp_walker`
- CoppeliaSim topics detected: `True`
- build completed: `true`
- trials completed: `True`

| field | value |
| --- | --- |
| step_length | 0.04 |
| step_width | 0.09 |
| foot_clearance | 0.035 |
| pelvis_height | 0.48 |
| step_time | 1.2 |
| double_support_time | 0.35 |
| zmp_kp | 1.6 |
| zmp_kd | 1.0 |
| ankle_pitch_kp | 0.35 |
| ankle_roll_kp | 0.35 |

| metric | value |
| --- | --- |
| profile_name | baseline_conservative |
| max_abs_pitch_deg | 0.242 |
| max_abs_roll_deg | 18.765 |
| max_abs_gyro | 0.346 |
| forward_progress | 0.270 |
| duration | 2.780 |
| steps_completed | 0 |
| abort | True |
| score | -51.105 |

- failure reason: No profile met success thresholds.
- next step: 启动 CoppeliaSim Asti 场景后复测；若已启动，先根据 pitch/roll 方向确认 ankle/hip 补偿符号。

## Asti ASIMO-style ZMP walker tuning log

### Successful profile - 2026-05-18 15:04:17

- workspace: `/home/astondky/Desktop/robot_simulation_experiment_ws`
- package: `robot_simulation_experiment`
- executable: `asimo_style_zmp_walker`
- CoppeliaSim topics detected: `True`
- build completed: `true`
- trials completed: `True`

| field | value |
| --- | --- |
| step_length | 0.04 |
| step_width | 0.09 |
| foot_clearance | 0.035 |
| pelvis_height | 0.48 |
| step_time | 1.2 |
| double_support_time | 0.35 |
| zmp_kp | 1.6 |
| zmp_kd | 1.0 |
| ankle_pitch_kp | 0.35 |
| ankle_roll_kp | 0.35 |

| metric | value |
| --- | --- |
| profile_name | baseline_conservative |
| max_abs_pitch_deg | 0.224 |
| max_abs_roll_deg | 0.371 |
| max_abs_gyro | 0.042 |
| forward_progress | 0.034 |
| duration | 10.020 |
| steps_completed | 3 |
| abort | False |
| score | 6.218 |

- run command: `ros2 run robot_simulation_experiment asimo_style_zmp_walker --ros-args -p mode:=auto_tune`
- note: 参数是在 CoppeliaSim Asti 场景下自动试运行得到的。

## Asti ASIMO-style ZMP walker tuning log

### Auto tune attempt - 2026-05-18 15:17:51

- workspace: `/home/astondky/Desktop/robot_simulation_experiment_ws`
- package: `robot_simulation_experiment`
- executable: `asimo_style_zmp_walker`
- CoppeliaSim topics detected: `True`
- build completed: `true`
- trials completed: `True`

| field | value |
| --- | --- |
| step_length | 0.055 |
| step_width | 0.09 |
| sagittal_sign | -1.0 |
| foot_clearance | 0.05 |
| pelvis_height | 0.48 |
| step_time | 1.55 |
| double_support_time | 0.45 |
| swing_lift_fraction | 0.28 |
| swing_lower_fraction | 0.3 |
| zmp_kp | 1.6 |
| zmp_kd | 1.0 |
| ankle_pitch_kp | 0.35 |
| ankle_roll_kp | 0.35 |

| metric | value |
| --- | --- |
| profile_name | front_minus_y_high_lift |
| max_abs_pitch_deg | 6.353 |
| max_abs_roll_deg | 20.259 |
| max_abs_gyro | 1.070 |
| forward_progress | 0.103 |
| duration | 8.720 |
| steps_completed | 2 |
| abort | True |
| score | -50.293 |

- failure reason: No profile met success thresholds.
- next step: 启动 CoppeliaSim Asti 场景后复测；若已启动，先根据 pitch/roll 方向确认 ankle/hip 补偿符号。

## Asti ASIMO-style ZMP walker tuning log

### Auto tune attempt - 2026-05-18 15:19:14

- workspace: `/home/astondky/Desktop/robot_simulation_experiment_ws`
- package: `robot_simulation_experiment`
- executable: `asimo_style_zmp_walker`
- CoppeliaSim topics detected: `True`
- build completed: `true`
- trials completed: `True`

| field | value |
| --- | --- |
| step_length | 0.055 |
| step_width | 0.09 |
| sagittal_sign | -1.0 |
| foot_clearance | 0.05 |
| pelvis_height | 0.48 |
| step_time | 1.75 |
| double_support_time | 0.55 |
| swing_lift_fraction | 0.28 |
| swing_lower_fraction | 0.3 |
| zmp_kp | 1.6 |
| zmp_kd | 1.0 |
| ankle_pitch_kp | 0.35 |
| ankle_roll_kp | 0.35 |

| metric | value |
| --- | --- |
| profile_name | front_minus_y_high_lift |
| max_abs_pitch_deg | 7.090 |
| max_abs_roll_deg | 22.107 |
| max_abs_gyro | 0.613 |
| forward_progress | 0.146 |
| duration | 6.260 |
| steps_completed | 1 |
| abort | True |
| score | -52.382 |

- failure reason: No profile met success thresholds.
- next step: 启动 CoppeliaSim Asti 场景后复测；若已启动，先根据 pitch/roll 方向确认 ankle/hip 补偿符号。

## Asti ASIMO-style ZMP walker tuning log

### Successful profile - 2026-05-18 15:28:18 CST

- workspace: `/home/astondky/Desktop/robot_simulation_experiment_ws`
- package: `robot_simulation_experiment`
- executable: `asimo_style_zmp_walker`
- CoppeliaSim topics detected: `True`
- validation mode: `walk`
- result: completed all 6 planned steps, entered `STAND`, then entered `DONE` and held the final upright posture

| field | value |
| --- | --- |
| step_length | 0.045 |
| step_width | 0.09 |
| sagittal_sign | -1.0 |
| support_zmp_margin | 0.004 |
| foot_clearance | 0.045 |
| pelvis_height | 0.48 |
| step_time | 1.75 |
| double_support_time | 0.55 |
| stand_time | 2.0 |
| max_com_speed | 0.075 |
| max_com_accel | 0.20 |
| zmp_kp | 1.6 |
| zmp_kd | 1.0 |
| ankle_pitch_kp | 0.35 |
| ankle_roll_kp | 0.35 |

| metric | value |
| --- | --- |
| steps_completed | 6 |
| final_state | DONE |
| abort | False |
| observed_max_pitch_during_walk_deg | about 0.54 |
| observed_max_roll_during_walk_deg | about 1.28 |
| observed_done_pitch_roll_deg | within about 0.1 |

- run command: `ros2 run robot_simulation_experiment asimo_style_zmp_walker`
- note: 参数是在 CoppeliaSim Asti 场景下普通 `walk` 模式手动试运行得到的；不是 auto_tune 自动选择结果。
