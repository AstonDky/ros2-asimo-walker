# Contributing / 参与贡献

Small, reproducible changes are welcome. For a new gait or control method, open an issue explaining the problem and intended behavior before making a large change.

欢迎范围明确、可复现的修改。新增步态或更改控制方法前，先通过 issue 说明实际问题和预期行为。

## Development checks

```bash
python3 -B -m unittest discover -s tests -v
```

These checks cover offline controller behavior and the public entry-point import; they are not simulator tests. For ROS packaging changes, also run `colcon build --symlink-install --packages-select robot_simulation_experiment` in a sourced ROS environment. For motion changes, include the actual simulator version, ROS distribution, physics engine/time step, mode/profile, changed parameters, and a recording or log showing the behavior. State which checks were not run.

上述检查只覆盖离线控制逻辑和公开入口导入。修改 ROS 打包时还需在对应环境实际构建；修改运动行为时，记录模拟器版本、ROS 发行版、物理引擎与时间步、模式/profile、参数差异及录像或日志。没有运行的检查应明确说明，不能用离线测试代替行走验证。

## Pull requests

Keep changes focused. Preserve units, joint ordering and coordinate conventions. Update both README languages when public behavior changes. Do not claim optimal preview control, measured ZMP feedback, hardware deployment or quantitative performance without a corresponding implementation and evidence.

Keep build/install/log output, IDE state, credentials and local experiments out of commits. Submit only material you are entitled to contribute. Original source/documentation contributions use this project's MIT terms; do not relicense third-party scene assets without recorded permission. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

修改应保持单位、关节顺序和坐标约定清楚，公开行为变化时同步中英文 README。构建输出、IDE 状态、凭据和本地实验不提交。贡献内容须具有相应权利，第三方场景素材不能未经依据重新授权。

Use [GitHub issues](https://github.com/AstonDky/ros2-asimo-walker/issues) for reproducible bugs and proposals. Remove private paths and personal information from attached logs.
