# Walker 技术详解索引

这是一组拆开的技术笔记，每份只讲一个核心模块，适合在 Obsidian 里按主题阅读。

## 阅读入口

- [[asimo_walker_code_reading_guide]]：按 `W` 后整条主控制链怎么走
- [[walker_function_link_map]]：函数 / 文件 / 笔记的跳转地图，读代码卡住时从这里查
- [[footstep_planner_notes]]：脚步几何规划，回答“下一脚要落在哪里”
- [[zmp_reference_notes]]：支撑重心参考，回答“当前重心应该压向哪里”
- [[com_planner_notes]]：CoM / 骨盆平移规划，回答“身体怎么平滑地跟过去”
- [[leg_ik_notes]]：腿部 IK，回答“脚和骨盆位姿怎么变成 12 个腿关节”
- [[stabilizer_notes]]：稳定器，回答“反馈如何修正当前关节和下一步落脚”

## 推荐阅读顺序

如果你已经在某个函数名上卡住，优先打开 [[walker_function_link_map]]，按函数名反查它来自哪个 `.py` 文件、在哪篇笔记里解释。

如果你想按控制链顺序通读，建议：

```text
asimo_walker_code_reading_guide
-> walker_function_link_map
-> footstep_planner
-> zmp_reference
-> com_planner
-> leg_ik
-> stabilizer
```

如果你只想按模块笔记复习，保留原来的顺序：

```text
footstep_planner
-> zmp_reference
-> com_planner
-> leg_ik
-> stabilizer
```

## 这组笔记的边界

- 只分析当前 walker 主控制链
- `autotune` 按之前约定忽略
- UI 只在需要说明数据流入口时顺带提及，不展开
