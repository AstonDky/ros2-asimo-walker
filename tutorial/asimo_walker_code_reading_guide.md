# Asti Walker 核心控制链跟读笔记

> [!summary]
> 这份笔记只做一件事：
> **带你沿着 `按下 W` 这条主线，顺着当前代码一步一步走到仿真机器人真正迈步。**

---

## 0. 阅读前提

这份笔记你已经懂：

- ROS 2 节点、topic、publisher / subscriber
- tkinter GUI 只是输入界面
- humanoid walking 里 `ZMP / CoM / footstep / IK / stabilizer` 这些术语的基本背景

所以这里**不重复解释 ROS 基础**，重点只放在：

1. 当前代码里，`W` 是怎么一路传到控制器里的
2. 控制器每一帧到底按什么顺序算
3. `footstep_planner / zmp_reference / zmp_preview / swing_foot / leg_ik / stabilizer`
   在这套工程实现里分别承担什么职责
4. 这些模块拼起来以后，为什么机器人会“先移重心，再抬脚，再摆腿，再落脚，再站稳”

> [!warning]
> 按你的规则，这份笔记**忽略 `autotune`**，把它视为不存在。

---

## 0. 函数跳转入口

如果你是被某个函数名卡住，而不是想从头顺读，先看总表：

- [[walker_function_link_map]]：按函数名反查来源 `.py` 文件和对应讲解位置

这份主线笔记重点覆盖 `teleop_gui.py`、`teleop_command.py`、`teleop_profiles.py`、`contact_state_machine.py`、`main.py` 之间的控制链。更细的模块解释请跳到：

| 看到的函数 / 模块 | 实际来源文件 | 推荐跳转 |
|---|---|---|
| `FootstepPlanner.get_step()` / `modify_next_step()` | `footstep_planner.py` | [[footstep_planner_notes]] |
| `ZMPReferencePlanner.zmp_for_state()` / `preview_zmp()` | `zmp_reference.py` | [[zmp_reference_notes]] |
| `ZMPPreviewController.update()` | `zmp_preview.py` | [[com_planner_notes]] |
| `SwingFootPlanner.pose()` | `swing_foot.py` | [[asimo_walker_code_reading_guide#8.6 第六步：摆动脚怎么在空中走|本笔记 8.6 摆脚轨迹]] |
| `LegIK.solve()` / `_solve_leg()` | `leg_ik.py` | [[leg_ik_notes]] |
| `Stabilizer.compute()` | `stabilizer.py` | [[stabilizer_notes]] |

---

## 1. 先给你整条调用链

如果你只想先抓大图，`按下 W` 的主路径是：

```text
teleop_gui.py
  -> TeleopCommandBuffer.press_key("w")
  -> main.py::_teleop_hold_before_update()
  -> teleop_profiles.resolve_profile_from_input()
  -> main.py::_reset_teleop_walk_session()
  -> ContactAndStateMachine.start()
  -> main.py::loop() 周期执行
      -> state_machine.update()
      -> footsteps.get_step()
      -> zmp_ref.zmp_for_state()
      -> com.update()
      -> _foot_targets() / swing.pose()
      -> ik.solve()
      -> stabilizer.compute()
      -> _rate_limit()
      -> publish /legTargetJoints
```

你可以把它理解成三层：

- **输入层**：`W` 被 GUI 锁存成一个“前进请求”
- **步态层**：状态机决定当前是在移重心、摆左脚、摆右脚、落脚还是恢复站立
- **控制层**：ZMP / CoM / 摆脚 / IK / 稳定器在每一帧把“步态相位”变成关节命令

---

## 2. 第一段：你按下 `W` 的那一刻，代码发生了什么

---

### 2.1 GUI 不直接让机器人动，它只写入“请求”

文件：`src/robot_simulation_experiment/scripts/asimo_walker/teleop_gui.py`

先看按键入口：

```python
def _on_key_press(self, event) -> None:
    keysym = event.keysym
    if keysym in self._pressed:
        return
    self._pressed.add(keysym)

    lower = keysym.lower()
    if lower in ("w", "s", "a", "d", "q", "e"):
        self.command_buffer.press_key(lower)
    elif keysym in ("Shift_L", "Shift_R"):
        self.command_buffer.press_key("shift")
    elif keysym == "space":
        self.command_buffer.toggle_pause()
    elif lower == "r":
        self.command_buffer.reset_keys()
    elif keysym == "Escape":
        self.command_buffer.set_emergency_stop()
```

### 这一段的重点

- `teleop_gui.py` **没有**直接发 `/legTargetJoints`
- 它只是把 `w` 交给 `TeleopCommandBuffer`
- 真正让机器人进入 walking 的决定，是控制线程下一帧读取 buffer 后做出的

> [!note]
> 这就是当前架构里很重要的一个边界：
> **GUI 是输入层，不是控制层。**

---

### 2.2 `press_key("w")` 到底做了什么

文件：`src/robot_simulation_experiment/scripts/asimo_walker/teleop_command.py`

```python
def press_key(self, key: str) -> None:
    """Latch one teleop request; pressing the active key again releases it."""
    key = key.lower()
    with self._lock:
        if key == "w":
            if self._state.key_w:
                self._clear_motion_locked()
            else:
                self._clear_motion_locked()
                self._state.key_w = True
            return
```

### 这段代码的语义

`W` 不是“按住持续生效”的设计，而是**锁存式切换**：

- 第一次按 `W`：清掉别的动作锁定，设置 `key_w = True`
- 再按一次 `W`：清掉当前锁定，相当于请求安全停步

所以 `W` 更像一个模式切换按钮，而不是即时速度命令。

---

## 3. 第二段：控制器下一帧如何读到这个 `W`

文件：`src/robot_simulation_experiment/scripts/asimo_walker/main.py`

主控制循环是 `loop()`，在节点初始化时通过 timer 周期调用。

先看它在 GUI 模式下的关键入口：

```python
pitch = self.feedback.ori[0] - self.ori0[0]
roll = self.feedback.ori[1] - self.ori0[1]
gyro = list(self.feedback.gyro or [0.0, 0.0, 0.0])

if self.mode == "teleop_gui" and self._teleop_hold_before_update(pitch, roll):
    return
```

### 这里发生了什么

`loop()` 不会一上来就推进 walking。

在 `teleop_gui` 模式下，它先调用：

```python
_teleop_hold_before_update(pitch, roll)
```

这个函数就是**输入门控器**。

你可以把它当成：

```text
按键状态解释器
+ 安全模式切换器
+ 行走会话启动器
+ 停步管理器
+ 静态扭腰分流器
```

---

## 4. 第三段：`W` 如何被翻译成 “forward_walk profile”

文件：`src/robot_simulation_experiment/scripts/asimo_walker/main.py`

```python
def _teleop_hold_before_update(self, pitch: float, roll: float) -> bool:
    input_state = self.teleop_buffer.snapshot() if self.teleop_buffer else TeleopInputState()
    profile, status = resolve_profile_from_input(input_state)
    requested_name = requested_profile_name(input_state)
    self.pending_profile = profile
    self.teleop_requested_profile_name = requested_name
    self.teleop_status_message = status
```

这里最关键的不是 `snapshot()`，而是：

```python
resolve_profile_from_input(input_state)
```

---

### 4.1 `resolve_profile_from_input()` 是输入解释核心

文件：`src/robot_simulation_experiment/scripts/asimo_walker/teleop_profiles.py`

和 `W` 相关的核心逻辑是：

```python
if input_state.key_w:
    if requested_enabled:
        enabled_text = ", ".join(f"'{name}'" for name in requested_enabled)
        return (
            forward_walk_profile,
            f"Requested profile(s) {enabled_text} conflict with forward; using forward baseline only.",
        )
    return forward_walk_profile, "Forward baseline profile active."
```

### 这意味着什么

按下 `W` 之后，控制器并不是拿到“向前速度 = 某个值”。

它拿到的是：

```text
当前请求的 motion profile = forward_walk_profile
```

也就是一整组步态参数模板。

---

### 4.2 `forward_walk_profile` 究竟是什么

同一个文件里：

```python
def _forward_profile() -> MotionProfile:
    params = WalkerParams()
    return MotionProfile(
        name="forward_walk",
        enabled=True,
        description="Verified baseline forward walk",
        step_length=params.step_length,
        step_width=params.step_width,
        step_time=params.step_time,
        double_support_time=params.double_support_time,
        transfer_time=params.transfer_time,
        touchdown_time=params.touchdown_time,
        foot_clearance=params.foot_clearance,
        swing_lift_fraction=params.swing_lift_fraction,
        swing_lower_fraction=params.swing_lower_fraction,
        direction_sign=1.0,
        turn_yaw_per_step=0.0,
        waist_yaw_target=0.0,
        zmp_kp=params.zmp_kp,
        zmp_kd=params.zmp_kd,
        max_joint_rate=params.max_joint_rate,
        max_arm_rate=params.max_arm_rate,
        max_com_speed=params.max_com_speed,
        max_com_accel=params.max_com_accel,
        total_steps=params.total_steps,
        allow_continuous_steps=True,
    )
```

### 这一层的设计思想

`W` 不是“前进命令”，而是：

> **选中一套“前进步态参数模板”**

这套模板决定了：

- 步长
- 步宽
- 单步时间
- 双支撑时间
- 重心转移时间
- 摆脚抬高多高
- ZMP/CoM 跟踪增益
- 关节与 CoM 的最大变化率

所以当前 teleop 架构本质上是：

```text
按键 -> profile
profile -> WalkerParams
WalkerParams -> 整条 walking 控制链
```

---

## 5. 第四段：什么时候真的开始 walking

继续看 `_teleop_hold_before_update()`：

```python
if self._is_enabled_walk_profile(profile):
    ...
    self.active_profile = profile
    self.teleop_active_profile_name = profile.name
    self.state_machine.clear_stop_request()
    if state in (WalkState.WAIT, WalkState.DONE):
        self._reset_teleop_walk_session()
        self.state_machine.start()
    elif state == WalkState.STAND:
        self._reset_teleop_walk_session()
        self.state_machine.start()
    return False
```

### 这里是 `W` 真正生效的瞬间

当当前状态在：

- `WAIT`
- `DONE`
- `STAND`

那么按下 `W` 之后会做两件关键事：

1. `_reset_teleop_walk_session()`
2. `state_machine.start()`

这两步一启动，walker 才算真正进入新的行走会话。

---

## 6. 第五段：walk session 重置时到底重置了什么

文件：`main.py`

```python
def _reset_teleop_walk_session(self) -> None:
    self.params = walker_params_for_profile(self.active_profile)
    self.dt = self.params.dt
    self.footsteps.reset(self.params)
    self.zmp_ref.reset(self.params)
    self.com.reset(self.params, 0.0, 0.0)
    self.swing.reset(self.params)
    self.ik.reset(self.params)
    self.stabilizer.reset(self.params)
    self.state_machine.reset(self.params)
    self.state_machine.set_continuous_walk(self.active_profile.allow_continuous_steps)
    self.left_pose = self.footsteps.initial_left()
    self.right_pose = self.footsteps.initial_right()
    self.swing_start = {"left": self.left_pose.copy(), "right": self.right_pose.copy()}
    self.initial_left_q = list(self.feedback.left_leg)
    self.initial_right_q = list(self.feedback.right_leg)
    self.prev_left_cmd = list(self.feedback.left_leg)
    self.prev_right_cmd = list(self.feedback.right_leg)
    ...
```

### 这一步非常重要

它不是只改一个 `mode` 标志，而是把整条控制链都切到新的参数上下文里：

- 足步规划器重新加载参数
- ZMP 规划器重新加载参数
- CoM planner 重置状态
- swing 轨迹重置
- IK 重置参数
- stabilizer 重置参数
- 状态机重置

所以 `W` 不是“给老状态机发一个向前信号”，而是：

> **开启一段新的 forward walking 会话，并让整条控制链从统一的前进参数重新起步。**

---

## 7. 第六段：状态机是如何真正开步的

文件：`src/robot_simulation_experiment/scripts/asimo_walker/contact_state_machine.py`

启动入口：

```python
def start(self) -> None:
    if self.state == WalkState.WAIT:
        self._set(WalkState.CROUCH)
```

也就是说，按下 `W` 之后，机器人不是立刻抬脚，而是先进入：

```text
WAIT -> CROUCH
```

这点很关键，因为它解释了为什么这个 walker 看起来像保守的 ASIMO-style，而不是立刻甩腿。

---

## 8. 第七段：`loop()` 每一帧怎么把 walking 算出来

下面开始进入真正的核心。

主循环在 `main.py::loop()` 里。我们按**执行顺序**拆。

---

### 8.1 第一步：状态机先推进当前步态相位

```python
state = self.state_machine.update(self.dt, self.feedback, pitch, roll)
self._handle_state_entry(state)
```

你要先把状态机理解成：

> 当前这帧，控制器决定自己正处于哪一个 walking phase。

核心状态推进逻辑：

```python
if self.state == WalkState.CROUCH and self.state_t >= p.crouch_time and stable:
    self._set(WalkState.TRANSFER_TO_RIGHT)
elif self.state == WalkState.TRANSFER_TO_RIGHT and self.state_t >= p.transfer_time and self._support_loaded(feedback, "right", stable):
    self._set(WalkState.LEFT_SWING)
elif self.state == WalkState.LEFT_SWING and self.state_t >= p.step_time:
    self._set(WalkState.LEFT_TOUCHDOWN)
elif self.state == WalkState.LEFT_TOUCHDOWN and self.state_t >= p.touchdown_time and self._swing_contact(feedback, "left", stable):
    self._set(WalkState.DOUBLE_SUPPORT_AFTER_LEFT)
...
```

### 这一段应该怎么理解

它对应的是经典双足起步逻辑：

1. `CROUCH`
   先把身体放到一个更适合走路的屈膝状态

2. `TRANSFER_TO_RIGHT`
   先把重心往右支撑脚挪，准备让左脚离地

3. `LEFT_SWING`
   左脚开始摆动

4. `LEFT_TOUCHDOWN`
   左脚落地，但还要等确认接触稳定

5. `DOUBLE_SUPPORT_AFTER_LEFT`
   双脚一起支撑，收一口气，稳定一下

然后镜像进入右脚摆动周期。

> [!important]
> 这就是这套 walker 最核心的节拍器。
> **所有 planner 都不是自己决定什么时候摆脚、什么时候移重心，而是跟着状态机节拍走。**

---

### 8.2 第二步：拿到当前这一步和下一步的足步目标

```python
current_step = self.footsteps.get_step(self.state_machine.step_index)
next_step = None
if self.mode == "teleop_gui" and self.active_profile.allow_continuous_steps:
    self.footsteps.ensure_steps(self.state_machine.step_index + 2)
if self.state_machine.step_index + 1 < len(self.footsteps.steps):
    next_step = self.footsteps.get_step(self.state_machine.step_index + 1)
```

这里依赖的是 `footstep_planner.py`。

先看它的核心：

```python
if index % 2 == 0:
    support = "right"
    swing = "left"
    yaw = right.yaw + p.turn_yaw_per_step
    dx, dy = self._rotated_step_offset(p.sagittal_sign * p.step_length, p.step_width, right.yaw, yaw)
    left = Pose2D(right.x + dx, right.y + dy, 0.0, 0.0, 0.0, yaw)
else:
    support = "left"
    swing = "right"
    yaw = left.yaw + p.turn_yaw_per_step
    dx, dy = self._rotated_step_offset(p.sagittal_sign * p.step_length, -p.step_width, left.yaw, yaw)
    right = Pose2D(left.x + dx, left.y + dy, 0.0, 0.0, 0.0, yaw)
```

### 这里到底在干什么

这段代码的职责非常纯粹：

> **生成“下一拍左右脚理论上应该落到哪里”**

`W` 对应的前进 profile 中：

- `turn_yaw_per_step = 0`
- `direction_sign = 1.0`
- `sagittal_sign` 经过 `walker_params_for_profile()` 保持前进方向

因此按下 `W` 之后，这里生成的就是：

- 左右脚交替向前的落脚点序列
- 每一步的横向分离由 `step_width` 决定
- 每一步的前进距离由 `step_length` 决定

### `footstep_planner` 的技术定位

它不解决：

- 重心怎么跟
- 脚在空中怎么走
- 关节怎么解

它只解决：

> **脚步几何目标在哪里**

这层很像高层几何步态规划。

---

### 8.3 第三步：当前 walking phase 下，ZMP 应该放哪里

主循环：

```python
zmp_phase = self.state_machine.double_support_phase()
if state in (WalkState.LEFT_TOUCHDOWN, WalkState.RIGHT_TOUCHDOWN):
    zmp_phase = min(1.0, self.state_machine.state_t / max(0.1, self.params.touchdown_time))
zmp_now = self.zmp_ref.zmp_for_state(state.name, current_step, zmp_phase, self.left_pose, self.right_pose)
zmp_preview = self.zmp_ref.preview_zmp(current_step, next_step)
if state in (WalkState.LEFT_SWING, WalkState.RIGHT_SWING):
    zmp_preview = zmp_now
```

这里调用的是 `zmp_reference.py`。

核心规则表：

```python
if state_name == "TRANSFER_TO_RIGHT":
    return lerp(center_x, right_current.x, u), lerp(center_y, right_support_y, u)
if state_name == "TRANSFER_TO_LEFT":
    return lerp(center_x, left_current.x, u), lerp(center_y, left_support_y, u)
if state_name == "LEFT_SWING":
    return right_current.x, right_support_y
if state_name == "RIGHT_SWING":
    return left_current.x, left_support_y
if state_name == "LEFT_TOUCHDOWN":
    return lerp(right_current.x, left_landing_center_x, u), lerp(right_support_y, left_landing_center_y, u)
if state_name == "RIGHT_TOUCHDOWN":
    return lerp(left_current.x, right_landing_center_x, u), lerp(left_support_y, right_landing_center_y, u)
```

### 这段是这套步态“看起来像走路”的关键之一

如果你从控制意义上翻译：

- `TRANSFER_TO_RIGHT`
  把 ZMP 从双脚中间移向右脚

- `LEFT_SWING`
  左脚摆动时，ZMP 固定在右脚支撑区域

- `LEFT_TOUCHDOWN`
  左脚落地后，把 ZMP 从右脚渐渐转回新双支撑中心

### 用一句更直白的话说

`zmp_reference.py` 的作用是：

> **在每个步态相位里告诉 CoM planner：重心现在该往哪边压。**

这就是当前代码里“先移重心，再抬脚”的实现核心。

---

### 8.4 第四步：根据 `zmp_now` 和 `zmp_preview` 推出 CoM / 骨盆平移

主循环：

```python
com_x, com_y, com_vx, com_vy, _, _ = self.com.update(self.dt, zmp_now, zmp_preview)
```

这里调用的是 `zmp_preview.py`：

```python
target_x = 0.72 * zmp_now[0] + 0.28 * zmp_preview[0]
target_y = 0.78 * zmp_now[1] + 0.22 * zmp_preview[1]

omega2 = G / max(0.25, p.pelvis_height)
ax_cmd = p.zmp_kp * omega2 * (target_x - self.com_x) - p.zmp_kd * self.vx / horizon
ay_cmd = p.zmp_kp * omega2 * (target_y - self.com_y) - p.zmp_kd * self.vy / horizon
ax_cmd = clamp(ax_cmd, -p.max_com_accel, p.max_com_accel)
ay_cmd = clamp(ay_cmd, -p.max_com_accel, p.max_com_accel)

self.vx = clamp(self.vx + ax_cmd * dt, -p.max_com_speed, p.max_com_speed)
self.vy = clamp(self.vy + ay_cmd * dt, -p.max_com_speed, p.max_com_speed)
self.com_x += self.vx * dt
self.com_y += self.vy * dt
```

### 如何理解这版 `zmp_preview`

它不是论文里的完整 preview control 实现，而是一个工程化简化版：

1. 用 `zmp_now` 和 `zmp_preview` 混成当前要靠近的“目标重心参考”
2. 用一个 LIPM 风格的近似，把目标位置误差转成 CoM 加速度命令
3. 再通过加速度限幅、速度限幅，积分出 `com_x/com_y`

### 它的工程角色

`footstep_planner` 只告诉系统“脚要落哪里”，但身体不是瞬移过去的。

`zmp_preview.py` 负责的是：

> **给骨盆 / CoM 一条平滑、可执行、有限速的跟随轨迹。**

没有这一层，你会得到很生硬的“几何上正确，但动力学上很难站住”的步态。

---

### 8.5 第五步：在当前状态下，左右脚此刻应该在哪里

主循环：

```python
left_target_pose, right_target_pose = self._foot_targets(state, current_step)
pelvis_yaw = self._average_yaw(left_target_pose.yaw, right_target_pose.yaw)
pelvis = Pose2D(com_x, com_y, self.params.pelvis_height, roll * 0.25, pitch * 0.25, pelvis_yaw)
left_q, right_q = self.ik.solve(pelvis, left_target_pose, right_target_pose)
```

先看 `_foot_targets()`：

```python
def _foot_targets(self, state: WalkState, step) -> tuple:
    left_target = self.left_pose.copy()
    right_target = self.right_pose.copy()
    phase = self.state_machine.swing_phase()
    if state in (WalkState.LEFT_SWING, WalkState.LEFT_TOUCHDOWN):
        left_target = self.swing.pose(self.swing_start["left"], step.left_target, phase)
    elif state in (WalkState.RIGHT_SWING, WalkState.RIGHT_TOUCHDOWN):
        right_target = self.swing.pose(self.swing_start["right"], step.right_target, phase)
    return left_target, right_target
```

### 这段的意义

当前帧的双脚目标不总是“下一步最终落脚点”。

它分两类：

- **支撑脚**：保持在当前已经站住的脚位
- **摆动脚**：沿着 swing 轨迹，从起点走到本步目标落脚点

也就是说：

`left_pose / right_pose` 是“已经落地站住的脚”

而

`swing.pose(...)` 生成的是“空中正在移动的脚”

---

### 8.6 第六步：摆动脚怎么在空中走

调用的是 `swing_foot.py`：

```python
def pose(self, start: Pose2D, target: Pose2D, phase: float) -> Pose2D:
    phase = max(0.0, min(1.0, phase))
    u = smoothstep(phase)
    lift_end = max(0.10, min(0.45, self.params.swing_lift_fraction))
    lower_start = max(lift_end + 0.10, 1.0 - self.params.swing_lower_fraction)
    if phase < lift_end:
        z_scale = smoothstep(phase / lift_end)
    elif phase > lower_start:
        z_scale = 1.0 - smoothstep((phase - lower_start) / max(0.05, 1.0 - lower_start))
    else:
        z_scale = 1.0
    z = lerp(start.z, target.z, u) + z_scale * self.params.foot_clearance
    return Pose2D(
        lerp(start.x, target.x, u),
        lerp(start.y, target.y, u),
        z,
        0.0,
        0.0,
        lerp(start.yaw, target.yaw, u),
    )
```

### `swing_foot` 的职责

它只回答一个问题：

> **摆动脚从起点到终点，中间走什么样的空间轨迹？**

其中：

- `x/y/yaw`：做平滑插值
- `z`：先抬、再悬空、再放下

这层很重要，因为它把“脚最终落哪里”和“脚在空中怎么走”解耦了：

- `footstep_planner`：落点几何
- `swing_foot`：空中轨迹形状

---

### 8.7 第七步：把骨盆和脚位姿变成腿关节角

调用的是 `leg_ik.py`：

```python
def solve(self, pelvis: Pose2D, left_foot: Pose2D, right_foot: Pose2D) -> tuple:
    return self._solve_leg(pelvis, left_foot, "left"), self._solve_leg(pelvis, right_foot, "right")
```

内部几何核心：

```python
dx_world = foot.x - hip_x
dy_world = foot.y - hip_y
dx = yaw_c * dx_world + yaw_s * dy_world
dy = -yaw_s * dx_world + yaw_c * dy_world
dz = hip_z - foot.z
...
knee_pitch = math.pi - math.acos(knee_cos)
...
hip_pitch = line_angle - reach
hip_roll = clamp(math.atan2(dy, max(0.08, dz)), -18.0 * D, 18.0 * D)
hip_yaw = wrap_pi(foot.yaw - pelvis.yaw)

ankle_pitch = foot.pitch - pelvis.pitch - hip_pitch - knee_pitch
ankle_roll = foot.roll - pelvis.roll - hip_roll
```

### IK 在这套系统里的角色

在前面所有模块里，输出都还是“几何和轨迹层”的量：

- 骨盆应该在哪
- 左右脚应该在哪

而真正要发给仿真的，是：

- 左腿 6 个关节角
- 右腿 6 个关节角

所以 `leg_ik.py` 的职责非常明确：

> **把上层的位姿目标翻译成底层的关节命令。**

这是整条链里从“任务空间”转到“关节空间”的关键边界。

---

### 8.8 第八步：稳定器在这时往里加什么

主循环：

```python
stab = self.stabilizer.compute(
    pitch,
    roll,
    gyro,
    self.state_machine.support_foot(),
    com_vx,
    com_vy,
    len(self.left_arm_cmd),
    len(self.right_arm_cmd),
)
self.pending_step_adjustment = (stab.next_step_dx, stab.next_step_dy)

left_q = self.ik.limit([left_q[i] + stab.left_add[i] for i in range(LEG_DOF)])
right_q = self.ik.limit([right_q[i] + stab.right_add[i] for i in range(LEG_DOF)])
```

稳定器内部：

```python
pitch_fb = clamp(pitch + 0.09 * gyro_pitch, -10.0 * D, 10.0 * D)
roll_fb = clamp(roll + 0.09 * gyro_roll, -10.0 * D, 10.0 * D)
...
ankle_pitch = clamp(-p.ankle_pitch_kp * pitch_fb, -5.5 * D, 5.5 * D)
ankle_roll = clamp(-p.ankle_roll_kp * roll_fb, -5.0 * D, 5.0 * D)
hip_pitch = clamp(-p.hip_pitch_kp * pitch_fb, -3.5 * D, 3.5 * D)
hip_roll = clamp(-p.hip_roll_kp * roll_fb, -3.5 * D, 3.5 * D)
...
out.next_step_dx = clamp(0.08 * pitch + 0.12 * com_vx, -0.018, 0.018)
out.next_step_dy = clamp(0.06 * roll + 0.10 * com_vy, -0.014, 0.014)
```

### 稳定器干了两件事

#### 1. 当前帧关节补偿

它会根据 `pitch / roll / gyro` 给：

- ankle pitch / roll
- hip pitch / roll

加一个小修正。

而且支撑脚和摆动脚的权重不同：

- 支撑脚修正更强
- 摆动脚修正更弱

这很合理，因为真正能抗倒的是支撑脚。

#### 2. 下一步足步修正

它还会输出：

- `next_step_dx`
- `next_step_dy`

这个不是马上改当前脚位，而是让后面 `_apply_next_step_adjustment()` 去改**下一步落脚点**。

### 这一层的控制意义

`footstep_planner + zmp + com + swing + ik` 更多像一条保守的参考轨迹链。

`stabilizer.py` 则是在这条参考链上增加：

> **基于姿态反馈的闭环修正**

所以如果你问“这套 walker 里真正的反馈控制主要落在哪里”，答案基本就是这里。

---

### 8.9 第九步：并不是直接发布，还要过几道工程安全层

主循环后半段：

```python
if state == WalkState.CROUCH:
    u = smoothstep(self.state_machine.state_t / max(0.1, self.params.crouch_time))
    left_q = [lerp(self.initial_left_q[i], left_q[i], u) for i in range(LEG_DOF)]
    right_q = [lerp(self.initial_right_q[i], right_q[i], u) for i in range(LEG_DOF)]
elif state == WalkState.ABORT:
    left_q = self.prev_left_cmd
    right_q = self.prev_right_cmd

left_q = self._rate_limit(left_q, self.prev_left_cmd, self.params.max_joint_rate)
right_q = self._rate_limit(right_q, self.prev_right_cmd, self.params.max_joint_rate)
self.prev_left_cmd = left_q
self.prev_right_cmd = right_q

self._publish_legs(left_q, right_q)
```

### 这里又做了三层处理

#### 1. `CROUCH` 期插值

从初始站姿慢慢插值进入 walking 姿态，而不是瞬时切换。

#### 2. `ABORT` 冻结

如果状态机判定跌倒风险过大，命令冻结在上一帧，不继续推进危险动作。

#### 3. `_rate_limit()`

```python
def _rate_limit(self, target, previous, max_rate):
    if previous is None:
        return list(target)
    limit = max_rate * self.dt
    return [previous[i] + clamp(target[i] - previous[i], -limit, limit) for i in range(len(target))]
```

这一步的作用是：

- 限制单帧关节变化率
- 防止 IK / stabilizer 输出尖跳
- 提高仿真电机关节的可跟踪性

> [!important]
> 最后发出去的 `/legTargetJoints`，
> **不是原始 IK 解，也不是原始 ZMP/CoM 结果，而是经过补偿、过渡、限速之后的最终执行命令。**

---

## 9. 第八段：为什么机器人真的会“先移重心再抬脚”

这件事不是一句注释写出来的，而是由多层逻辑共同造成的。

---

### 9.1 状态机层面

状态顺序明确要求：

```text
TRANSFER_TO_RIGHT
-> LEFT_SWING
```

也就是：

- 先重心转移
- 再左脚摆动

---

### 9.2 ZMP 层面

`TRANSFER_TO_RIGHT` 时：

```python
return lerp(center_x, right_current.x, u), lerp(center_y, right_support_y, u)
```

说明 ZMP 参考在往右支撑脚走。

`LEFT_SWING` 时：

```python
return right_current.x, right_support_y
```

说明左脚摆动期间，ZMP 固定留在右脚。

---

### 9.3 CoM 层面

`zmp_preview.py` 根据这个 ZMP 参考，把 `com_x/com_y` 推向对应支撑区域。

所以骨盆 / CoM 在几何上也会跟着移。

---

### 9.4 Swing 层面

只有当状态进入 `LEFT_SWING`，`_foot_targets()` 才会让左脚调用：

```python
self.swing.pose(...)
```

也就是说，不到 swing phase，摆动脚就不会真正起飞。

---

### 9.5 总结成一句话

“先移重心，再抬脚”在当前代码里不是某个单模块独自负责的，而是：

```text
状态机定相位
+ ZMP 参考定支撑重心方向
+ CoM planner 真正移动骨盆
+ swing planner 只在 swing phase 才抬脚
```

这四层叠起来，机器人才表现出保守的传统双足节奏。

---

## 10. `W` 持续按住时，为什么能连续前进

这点藏在 `MotionProfile` 里：

```python
allow_continuous_steps=True
```

以及 `main.py`：

```python
if self.mode == "teleop_gui" and self.active_profile.allow_continuous_steps:
    self.footsteps.ensure_steps(self.state_machine.step_index + 2)
```

这说明前进 profile 的设计是：

- 不只准备固定一步
- 而是在需要时继续向后补充理论足步

同时状态机里：

```python
if self.stop_requested or (not self.continuous_walk and self.step_index >= p.total_steps):
    ...
else:
    self._set(WalkState.TRANSFER_TO_LEFT)
```

因此只要 `continuous_walk` 开着，状态机在一拍结束后不会停，而会继续进入下一拍 transfer。

### 这一层的本质

`W` 的 forward profile 是一个**连续步行会话**。

不是按一次就只走一脚，而是持续追加步态周期，直到你再次按键请求停步。

---

## 11. 再按一次 `W` 为什么不是立刻刹停

再回头看 `press_key("w")`，第二次按会清掉锁存。

之后 `_teleop_hold_before_update()` 会走到 idle / stop 分支：

```python
self.active_profile = get_profile("idle")
self.teleop_active_profile_name = self.active_profile.name
self._request_safe_stop_or_hold(state)
```

而 `_request_safe_stop_or_hold()` 是：

```python
if state in (WalkState.CROUCH, WalkState.TRANSFER_TO_LEFT, WalkState.TRANSFER_TO_RIGHT):
    self._enter_stand_recovery()
elif state in (
    WalkState.LEFT_SWING,
    WalkState.LEFT_TOUCHDOWN,
    WalkState.DOUBLE_SUPPORT_AFTER_LEFT,
    WalkState.RIGHT_SWING,
    WalkState.RIGHT_TOUCHDOWN,
    WalkState.DOUBLE_SUPPORT_AFTER_RIGHT,
):
    self.state_machine.request_stop_after_current_step()
```

### 这意味着什么

系统不会在摆脚半空中硬停。

它的逻辑是：

- 如果还在比较早的 transfer / crouch 阶段，可以直接转恢复
- 如果已经进入某只脚摆动或刚落脚，就先把当前这一步安全做完
- 做完之后进入 `STAND`

> [!note]
> 这也是这套代码“看起来很保守”的另一个关键点：
> **停步是相位感知的安全停步，不是瞬时强行置零。**

---

## 12. 这套控制链里，各核心模块最准确的技术定位

这部分给你一个更“工程角色”式的总结。

---

### `footstep_planner.py`

**定位**：高层步点几何规划

**输入**：

- `step_length`
- `step_width`
- `turn_yaw_per_step`
- `sagittal_sign`

**输出**：

- 左右脚理论落脚点序列

**不负责**：

- 重心轨迹
- 摆脚轨迹细节
- 关节解算

---

### `zmp_reference.py`

**定位**：按步态相位生成支撑重心参考

**输入**：

- 当前状态名
- 当前步
- 当前左右脚已落地 pose

**输出**：

- 当前 ZMP 参考
- 下一拍 preview ZMP

**控制意义**：

- 把 walking 拆成“移重心 / 单脚支撑 / 落脚回中”几个支撑相位

---

### `zmp_preview.py`

**定位**：CoM / 骨盆平移的平滑跟踪器

**输入**：

- 当前 ZMP
- preview ZMP

**输出**：

- `com_x/com_y`
- `com_vx/com_vy`

**控制意义**：

- 把离散支撑参考变成平滑可执行的身体运动

---

### `swing_foot.py`

**定位**：摆动脚任务空间轨迹生成

**输入**：

- 起点脚位
- 目标落脚点
- 当前 swing phase

**输出**：

- 当前帧摆动脚 pose

**控制意义**：

- 决定脚在空中怎么抬、怎么放，而不是落点在哪里

---

### `leg_ik.py`

**定位**：任务空间到关节空间的变换层

**输入**：

- 骨盆 pose
- 左右脚 pose

**输出**：

- 左右腿 12 个关节角

**控制意义**：

- 把所有高层 planner 的几何结果变成关节命令

---

### `stabilizer.py`

**定位**：姿态反馈闭环补偿层

**输入**：

- pitch / roll / gyro
- 当前支撑脚
- CoM 速度

**输出**：

- 当前帧关节补偿
- 下一步足步修正
- 手臂平衡补偿

**控制意义**：

- 给原本偏参考轨迹式的 walker 加上反馈稳定性

---

## 13. 你如果要真正精读，建议就按这个顺序边跳边看

> [!tip]
> 这一版顺序不是“按文件目录”，而是按 `W` 的真实控制链。

### 主线顺序

1. `teleop_gui.py`：`_on_key_press()`
2. `teleop_command.py`：`press_key()`
3. `teleop_profiles.py`：`resolve_profile_from_input()` + `_forward_profile()`
4. `main.py`：`_teleop_hold_before_update()`
5. `main.py`：`_reset_teleop_walk_session()`
6. `contact_state_machine.py`：`start()` + `update()`
7. `main.py`：`loop()`
8. `footstep_planner.py`：`_append_next_step()`
9. `zmp_reference.py`：`zmp_for_state()`
10. `zmp_preview.py`：`update()`
11. `main.py`：`_foot_targets()`
12. `swing_foot.py`：`pose()`
13. `leg_ik.py`：`solve()`
14. `stabilizer.py`：`compute()`
15. `main.py`：`_rate_limit()` + `_publish_legs()`

---

## 14. 最后给一个“真正理解了没有”的自测标准

如果你已经读到位了，你应该能自己回答下面这 8 个问题：

1. 为什么按下 `W` 后不会立刻抬脚，而是先进入 `CROUCH` / `TRANSFER`？
2. 为什么前进不是直接用速度控制，而是切换到 `forward_walk_profile`？
3. `footstep_planner` 和 `swing_foot` 的边界到底在哪里？
4. `zmp_reference` 和 `zmp_preview` 的边界在哪里？
5. 为什么 `LEFT_SWING` 时 ZMP 要留在右脚？
6. 为什么 `stabilizer` 不只补当前关节，还要改下一步脚位？
7. 为什么发布前一定要 `_rate_limit()`？
8. 再按一次 `W`，为什么系统要“安全停步”而不是瞬停？

如果这 8 个问题你都能对照代码讲清楚，那你已经不是“看过这份代码”，而是**真正走通了当前 walker 的核心控制链**。

---

## 15. 一句话收尾

当前这套代码里，`W` 不是“让机器人前进”的直接命令。

它真正做的事情是：

> **切换到一套前进步态参数模板，启动状态机，让 `footstep -> ZMP -> CoM -> swing -> IK -> stabilizer -> rate limit` 这条控制链在每个控制周期里持续生成可执行的腿关节命令。**

这也是为什么它看起来不是简单播动作，而是真正在“走路”。

