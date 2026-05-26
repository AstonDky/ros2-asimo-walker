# Walker 函数跳转地图

> [!summary]
> 这份笔记不重新解释算法，只做一件事：
> **把主控制链里出现的函数名，标清楚来自哪个 `.py` 文件，并链接到对应讲解笔记。**

---

## 怎么用

- 在任意笔记里看到不熟的函数名，先来这里按函数名查
- “来源文件”告诉你它实际定义在哪个 `.py`
- “去哪里看”是 Obsidian 内部跳转链接
- 如果某个函数目前没有独立模块笔记，会先链接到主线笔记中的对应段落

---

## 主控制链函数

| 函数 / 类 | 来源文件 | 在控制链里的作用 | 去哪里看 |
|---|---|---|---|
| `TeleopWindow._on_key_press()` | `teleop_gui.py` | GUI 按键入口，把 `W` 写入输入 buffer | [[asimo_walker_code_reading_guide#2.1 GUI 不直接让机器人动，它只写入“请求”|GUI 按键入口]] |
| `TeleopCommandBuffer.press_key()` | `teleop_command.py` | 把 `W` 做成锁存式前进请求 | [[asimo_walker_code_reading_guide#2.2 `press_key("w")` 到底做了什么|press_key]] |
| `TeleopCommandBuffer.snapshot()` | `teleop_command.py` | 给控制循环读取当前按键快照 | [[asimo_walker_code_reading_guide#4. 第三段：`W` 如何被翻译成 “forward_walk profile”|输入快照进入 profile 解析]] |
| `resolve_profile_from_input()` | `teleop_profiles.py` | 把按键状态翻译成 motion profile | [[asimo_walker_code_reading_guide#4.1 `resolve_profile_from_input()` 是输入解释核心|profile 解析]] |
| `_forward_profile()` | `teleop_profiles.py` | 定义前进步态参数模板 | [[asimo_walker_code_reading_guide#4.2 `forward_walk_profile` 究竟是什么|forward profile]] |
| `walker_params_for_profile()` | `teleop_profiles.py` / 参数转换层 | 把 profile 落到 `WalkerParams` | [[asimo_walker_code_reading_guide#6. 第五段：walk session 重置时到底重置了什么|walk session 重置]] |
| `WalkerController._teleop_hold_before_update()` | `main.py` | teleop 模式下的输入门控、启动、停步管理 | [[asimo_walker_code_reading_guide#4. 第三段：`W` 如何被翻译成 “forward_walk profile”|输入门控]] |
| `WalkerController._reset_teleop_walk_session()` | `main.py` | 用新 profile 重置整条 walking 控制链 | [[asimo_walker_code_reading_guide#6. 第五段：walk session 重置时到底重置了什么|重置 walking 会话]] |
| `ContactAndStateMachine.start()` | `contact_state_machine.py` | 从 `WAIT` 进入 `CROUCH` | [[asimo_walker_code_reading_guide#7. 第六段：状态机是如何真正开步的|状态机启动]] |
| `ContactAndStateMachine.update()` | `contact_state_machine.py` | 每帧推进 walking phase | [[asimo_walker_code_reading_guide#8.1 第一步：状态机先推进当前步态相位|状态机更新]] |
| `WalkerController.loop()` | `main.py` | 主控制循环，串起所有 planner 和发布 | [[asimo_walker_code_reading_guide#8. 第七段：`loop()` 每一帧怎么把 walking 算出来|主循环]] |

---

## 步点与摆脚

| 函数 / 类 | 来源文件 | 在控制链里的作用 | 去哪里看 |
|---|---|---|---|
| `FootstepPlanner._append_next_step()` | `footstep_planner.py` | 生成下一拍理论落脚点 | [[footstep_planner_notes#4. 核心代码怎么生成步点|步点生成]] |
| `FootstepPlanner.get_step()` | `footstep_planner.py` | 按 `step_index` 取当前步 | [[footstep_planner_notes#2. 代码入口|FootstepPlanner 入口]] |
| `FootstepPlanner.ensure_steps()` | `footstep_planner.py` | 连续前进时补足后续步点 | [[asimo_walker_code_reading_guide#10. `W` 持续按住时，为什么能连续前进|连续步行补步]] |
| `FootstepPlanner.modify_next_step()` | `footstep_planner.py` | 接收 stabilizer 的下一步落脚修正 | [[footstep_planner_notes#8. `modify_next_step()` 是这层最有意思的接口|下一步落脚修正接口]] |
| `WalkerController._foot_targets()` | `main.py` | 把“已落地脚位”和“摆动脚轨迹”合成当前帧双脚目标 | [[asimo_walker_code_reading_guide#8.5 第五步：在当前状态下，左右脚此刻应该在哪里|当前帧双脚目标]] |
| `SwingFootPlanner.pose()` | `swing_foot.py` | 生成摆动脚从起点到落点的空中轨迹 | [[asimo_walker_code_reading_guide#8.6 第六步：摆动脚怎么在空中走|摆脚轨迹]] |

---

## ZMP / CoM

| 函数 / 类 | 来源文件 | 在控制链里的作用 | 去哪里看 |
|---|---|---|---|
| `ZMPReferencePlanner.zmp_for_state()` | `zmp_reference.py` | 根据当前状态相位给出当前 ZMP 参考 | [[zmp_reference_notes#6. 数学上它在做什么|ZMP 相位规则]] |
| `ZMPReferencePlanner.preview_zmp()` | `zmp_reference.py` | 给 CoM planner 一个下一拍支撑参考 | [[zmp_reference_notes#8. `preview_zmp()` 在当前实现里有多“preview”|preview_zmp]] |
| `ZMPPreviewController.update()` | `zmp_preview.py` | 把 ZMP 当前值和预览值积分成平滑 CoM 轨迹 | [[com_planner_notes#4. 核心代码先贴出来|CoM update]] |

---

## IK / 稳定 / 发布

| 函数 / 类 | 来源文件 | 在控制链里的作用 | 去哪里看 |
|---|---|---|---|
| `LegIK.solve()` | `leg_ik.py` | 把骨盆和左右脚 pose 解成左右腿关节角 | [[leg_ik_notes#4. 先看核心代码|LegIK.solve]] |
| `LegIK._solve_leg()` | `leg_ik.py` | 单条腿的几何 IK 计算 | [[leg_ik_notes#5. 它的计算思路怎么拆|单腿 IK 拆解]] |
| `LegIK.limit()` | `leg_ik.py` | 对 IK 结果做关节限位 | [[leg_ik_notes#6. 关节限位在这个模块里也是核心组成部分|关节限位]] |
| `Stabilizer.compute()` | `stabilizer.py` | 生成当前帧关节补偿、手臂补偿和下一步足步修正 | [[stabilizer_notes#3. 先看当前代码|Stabilizer.compute]] |
| `WalkerController._apply_next_step_adjustment()` | `main.py` | 把 stabilizer 的下一步修正写回 footstep planner | [[stabilizer_notes#8. 这个修正怎么真正进入足步规划|足步修正写回]] |
| `WalkerController._rate_limit()` | `main.py` | 限制关节命令单帧变化率 | [[asimo_walker_code_reading_guide#8.9 第九步：并不是直接发布，还要过几道工程安全层|rate limit]] |
| `WalkerController._publish_legs()` | `main.py` | 发布最终 `/legTargetJoints` | [[asimo_walker_code_reading_guide#8.9 第九步：并不是直接发布，还要过几道工程安全层|最终发布]] |

---

## 模块边界速记

```text
teleop_gui / teleop_command
  只负责把按键变成请求

teleop_profiles
  把请求变成 MotionProfile / WalkerParams

contact_state_machine
  决定现在处于哪个 walking phase

footstep_planner
  决定下一脚理论上落在哪里

zmp_reference
  决定当前相位重心应该压向哪里

zmp_preview
  把离散 ZMP 参考变成平滑 CoM / 骨盆平移

swing_foot
  决定摆动脚在空中怎么走

leg_ik
  把骨盆和脚位姿变成腿关节角

stabilizer
  用姿态反馈修正当前关节和下一步落脚点

main.py
  把所有模块按控制周期串起来
```
