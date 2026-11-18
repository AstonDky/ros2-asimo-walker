# ROS 2 topic contract / 话题约定

[English README](../README.md) · [中文 README](../README.zh-CN.md)

The node is `asimo_style_zmp_walker`. All topics below use `std_msgs/msg/Float64MultiArray` with queue depth 10. This contract follows [`main.py`](../src/robot_simulation_experiment/scripts/asimo_walker/main.py); the scene must use the same ordering and units. Embedded scene scripts are not separately exported in this repository.

节点名为 `asimo_style_zmp_walker`。下表全部使用 `std_msgs/msg/Float64MultiArray`，队列深度 10。约定按当前节点源码整理；场景应匹配相同顺序和单位。仓库没有另外导出场景内脚本源码。

| Direction / 方向 | Topic | Data / 数据 | Required / 必需 |
| --- | --- | --- | --- |
| Input / 输入 | `/robot/ori` | At least 3 values. Reads `[0]` as pitch and `[1]` as roll, radians; subtracts the initial sample. / 至少三项，前两项按 pitch、roll 处理，单位弧度，减去初始值。 | Yes |
| Input / 输入 | `/robot/angVel` | Angular velocity, rad/s; first two values enter attitude checks. / 角速度，前两项用于姿态检查。 | Yes |
| Input / 输入 | `/robot/pos` | Position array, normally XYZ in metres; required for readiness, not used as measured CoM feedback. / 通常为米制 XYZ，用于启动就绪判断，未作为实测 CoM 闭环反馈。 | Yes |
| Input / 输入 | `/leftLegJoints`, `/rightLegJoints` | At least six angles per leg, radians. / 每腿至少六个弧度制角度。 | Yes |
| Input / 输入 | `/leftArmJoints`, `/rightArmJoints` | Arm angles, radians. Both arrays must be present when arms initialize to enable arm output. / 双臂初始化时均有数据才启用手臂输出。 | Optional |
| Input / 输入 | `/leftFootForce`, `/rightFootForce` | One scalar or at least three force components. Uses absolute scalar or Euclidean norm, then left/right ratios. / 标量绝对值或前三项合力模长，再算左右占比。 | Optional |
| Input / 输入 | `/leftFootCOP`, `/rightFootCOP` | Stores the first two values; unused by the current control law. / 保存前两项，当前不参与控制律。 | Optional |
| Output / 输出 | `/legTargetJoints` | 12 angles: left six, then right six; radians. / 左腿六项在前，右腿六项在后。 | After readiness |
| Output / 输出 | `/armTargetJoints` | Left-arm targets followed by right-arm targets, radians. / 左臂在前，右臂在后。 | If initialized |

Leg order from [`leg_ik.py`](../src/robot_simulation_experiment/scripts/asimo_walker/leg_ik.py):

```text
hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_roll, ankle_pitch
```

Do not substitute an Euler/quaternion topic or arbitrary URDF joint order without adapting the scene bridge. Planar footstep coordinates and `sagittal_sign=-1` are internal conventions; the old tuning record describes forward travel as world −Y. Check the actual scene mapping before retargeting.

不要直接用四元数、其他欧拉角顺序或任意 URDF 关节排列替换输入。规划器平面坐标与 `sagittal_sign=-1` 属于内部约定；旧调参记录将场景前进方向描述为世界 −Y。移植前需确认场景实际映射。

Force feedback enables support-load (>56%) and touchdown (>20%) ratio checks. If either force is absent, or their sum is near zero, the state machine uses timing/attitude fallback. CoP availability does not enable a measured ZMP controller.

双脚力反馈可用于支撑加载（占比 >56%）和落脚（占比 >20%）判断；缺少任一脚力或合力接近零时，退回时间/姿态判断。提供 CoP 话题不会自动启用实测 ZMP 闭环。
