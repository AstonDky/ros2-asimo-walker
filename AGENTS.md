# Project Notes

## Active Goal

This workspace controls the CoppeliaSim Asti humanoid model through ROS 2
Jazzy. The current project direction is to make Asti walk in an ASIMO-style
traditional control stack rather than a direct sinusoidal or fixed joint
sequence.

The walking controller should execute a conservative biped sequence:
shift weight, lift one leg, keep balance on the support leg, swing forward,
land, settle in double support, then alternate legs. Prefer closed-loop
balance control, ZMP/LIPM-style feedback, conservative step timing, joint rate
limits, and explicit recovery/abort checks over aggressive walking.

## ASIMO-style Walker Implementation Scope

For the modular ASIMO-style walker, use this source structure only:

- `src/robot_simulation_experiment/scripts/asimo_walker/__init__.py`
- `src/robot_simulation_experiment/scripts/asimo_walker/common.py`
- `src/robot_simulation_experiment/scripts/asimo_walker/footstep_planner.py`
- `src/robot_simulation_experiment/scripts/asimo_walker/zmp_reference.py`
- `src/robot_simulation_experiment/scripts/asimo_walker/zmp_preview.py`
- `src/robot_simulation_experiment/scripts/asimo_walker/swing_foot.py`
- `src/robot_simulation_experiment/scripts/asimo_walker/leg_ik.py`
- `src/robot_simulation_experiment/scripts/asimo_walker/stabilizer.py`
- `src/robot_simulation_experiment/scripts/asimo_walker/contact_state_machine.py`
- `src/robot_simulation_experiment/scripts/asimo_walker/main.py`

Do not add `scripts/asimo_style_zmp_walker_node.py`. The executable entry is
`scripts/asimo_walker/main.py`, installed as:

```bash
ros2 run robot_simulation_experiment asimo_style_zmp_walker
```

Required control chain:

```text
FootstepPlanner
-> ZMPReferencePlanner
-> ZMPPreviewPlanner / CoM planner
-> SwingFootPlanner
-> LegIK
-> Stabilizer
-> ContactAndStateMachine
-> joint limit + rate limit
-> ROS2 publisher
```

Do not collapse these modules into a single direct joint-angle state machine.
The robot should perform actual weight transfer, foot lift, forward swing,
landing, settling, and leg alternation.

## ROS 2 Interface

ASIMO-style node:

- class: `AsimoStyleZMPWalker`
- node name: `asimo_style_zmp_walker`
- publishes `/legTargetJoints` and `/armTargetJoints` as
  `std_msgs/msg/Float64MultiArray`
- subscribes `/robot/ori`, `/robot/angVel`, `/robot/pos`,
  `/leftLegJoints`, `/rightLegJoints`, `/leftArmJoints`, `/rightArmJoints`
- optional subscriptions: `/leftFootForce`, `/rightFootForce`,
  `/leftFootCOP`, `/rightFootCOP`

If foot force or COP topics are absent, the controller must not crash.
`ContactAndStateMachine` should use phase-based fallback.

The Asti scene subscribes to `/legTargetJoints` with 12 values:
left leg 6 values followed by right leg 6 values. Leg joint order is:

1. hip yaw
2. hip roll
3. hip pitch
4. knee pitch
5. ankle roll
6. ankle pitch

The scene subscribes to `/armTargetJoints` as left arm targets followed by
right arm targets. Arm count may vary, so publish arms only after feedback is
known.

## Default Walker Parameters

Use conservative defaults unless tuning data proves better values. As of the
2026-05-18 direction correction, the robot's desired map/world walking
direction is negative Y, and internal sagittal footstep progression uses
`sagittal_sign = -1.0` to avoid stepping backward in the scene.

- `step_length = 0.045`
- `step_width = 0.09`
- `step_time = 1.75`
- `double_support_time = 0.55`
- `foot_clearance = 0.045`
- `pelvis_height = 0.48`
- `total_steps = 6`
- `sagittal_sign = -1.0`

Safety thresholds:

- stable pitch `< 5 deg`
- stable roll `< 6 deg`
- abort if pitch or roll `> 18 deg`

The sensor script convention treats `/robot/ori[0]` as pitch-like tilt and
`/robot/ori[1]` as roll-like tilt. Preserve this unless testing proves the
axes are reversed.

## Build, Test, and Tuning Rules

After editing the ASIMO-style walker:

```bash
python3 -m py_compile \
  src/robot_simulation_experiment/scripts/asimo_walker/main.py \
  src/robot_simulation_experiment/scripts/asimo_walker/*.py

source /opt/ros/jazzy/setup.bash
rm -rf build install log
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 run robot_simulation_experiment asimo_style_zmp_walker --help
```

Before auto tuning, run `ros2 topic list` and require these topics:

- `/robot/ori`
- `/robot/angVel`
- `/robot/pos`
- `/leftLegJoints`
- `/rightLegJoints`

If the topics are absent, do not claim CoppeliaSim testing succeeded and do not
record successful parameters. It is acceptable to append an attempted tuning
record saying the simulation topics were not detected.

Auto-tune mode:

```bash
ros2 run robot_simulation_experiment asimo_style_zmp_walker --ros-args -p mode:=auto_tune
```

Use conservative coordinate/profile search only. Avoid random broad sweeps and
stop a trial immediately on ABORT or excessive pitch/roll. Record results in
`Instruc.md` if present, otherwise `instruc.md`, otherwise create
`Instruc.md`. Successful profiles require no ABORT, pitch and roll under
12 deg, forward progress over 0.02 m, and at least one completed step.

## Documentation Automation Rules

When changing walker parameters, always update the tuning record in
`instruc.md` in the same change. The record must include the old parameter
values, new parameter values, measured result before and after when available,
and a concrete explanation of what improved or regressed. Examples of concrete
comparison points are ABORT/no ABORT, completed step count, max pitch/roll,
forward progress, final `STAND`/`DONE` stability, foot dragging, touchdown
impact, and speed/stability tradeoff.

When changing the walking architecture or adding a new module, also update
`readme.md`. The README must stay current for project intent, control method,
code structure, tunable parameters, tuning interfaces, and build history. This
includes future additions such as keyboard teleoperation, waist yaw/twist
control, turning gait, reinforcement learning, MPC, whole-body control, or any
other new control layer.

When updating README parameter tables, read the values from the current source
instead of copying stale values from older notes. The primary source for walker
defaults is `src/robot_simulation_experiment/scripts/asimo_walker/common.py`
`WalkerParams`; auto-tune candidate overrides are in
`src/robot_simulation_experiment/scripts/asimo_walker/main.py`
`_build_profiles()`.

## Reference Material

Primary references for this project direction:

- Honda P2 / ASIMO autonomous biped walking control technology
- Kajita et al., ZMP preview control and cart-table model
- Park and Youm, general ZMP preview control
- Kajita et al., Introduction to Humanoid Robotics
- Step timing adaptation for walking stabilization
- MPC humanoid gait generation stability and feasibility
- Force-feedback whole-body stabilizers for position-controlled humanoids
- Caron, variable-height inverted pendulum feedback stabilization
- HECTOR humanoid loco-manipulation work
- Robust humanoid walking on compliant and uneven terrain with RL

Useful codebase references:

- `chauby/ZMP_preview_control`
- `jrl-umi3218/jrl-walkgen`
- `stephane-caron/pymanoid`
- `stephane-caron/pink`
- `stack-of-tasks/pinocchio`
- `Rhoban/placo`
- `stephane-caron/qpmpc`
- `stephane-caron/vhip_light`
- `DRCL-USC/Hector_Simulation`
- `rohanpsingh/LearningHumanoidWalking`

## Project Log

### 2026-05-18 ASIMO-style ZMP walker scaffold and first successful tuning

- Added the modular `scripts/asimo_walker/` Python controller stack.
- Installed `scripts/asimo_walker/main.py` as
  `ros2 run robot_simulation_experiment asimo_style_zmp_walker`.
- Fixed the CMake install path for existing `scripts/humanoid_walk_only_ros2.py`.
- First auto-tune attempt failed because transfer-to-right ZMP was incorrectly
  moving toward the next left foothold before left swing; this caused roll to
  exceed the 18 deg ABORT threshold.
- Corrected ZMP reference generation to be state-aware:
  CROUCH uses foot center, TRANSFER_TO_RIGHT moves to right foot,
  LEFT_SWING holds right-foot ZMP, and the opposite sequence mirrors it.
- Successful CoppeliaSim auto-tune profile:
  `baseline_conservative`, `step_length=0.04`, `step_width=0.09`,
  `foot_clearance=0.035`, `pelvis_height=0.48`, `step_time=1.2`,
  `double_support_time=0.35`, `zmp_kp=1.6`, `zmp_kd=1.0`,
  `ankle_pitch_kp=0.35`, `ankle_roll_kp=0.35`.
- Metrics from the successful run: max pitch `0.224 deg`, max roll
  `0.371 deg`, max gyro `0.042`, forward progress `0.034 m`,
  completed steps `3`, ABORT `false`, score `6.218`.
- Detailed tuning record is in `instruc.md` under
  `Asti ASIMO-style ZMP walker tuning log`.

### 2026-05-18 Direction and higher-step adjustment

- User observed that the previous walker stepped backward in the CoppeliaSim
  scene; the intended robot-forward map direction is negative Y.
- Changed default internal sagittal progression to `sagittal_sign=-1.0`.
- Changed auto-tune forward-progress scoring to use map `-Y`:
  `forward_progress = -robot_pos_y - start_forward`.
- Increased default foot lift and step size:
  `step_length=0.055`, `foot_clearance=0.05`, `step_time=1.55`,
  `double_support_time=0.45`.
- Added a high-foot hold plateau in `SwingFootPlanner` using
  `swing_lift_fraction=0.28` and `swing_lower_fraction=0.30`, so the swing
  foot lifts, stays high briefly for balance, then lands.

### 2026-05-18 Larger-step stability refinement

- First high-lift negative-Y trial moved forward about `0.103 m`, confirming
  the direction correction, but ABORTed during the third step from excessive
  roll.
- Kept the higher lift and larger step intent, then slowed the default gait:
  `step_time=1.75`, `double_support_time=0.55`, `transfer_time=0.85`,
  `max_com_speed=0.09`, `max_com_accel=0.25`.
- During single support, the preview ZMP is held at the active support foot
  instead of looking ahead to the next support. This keeps CoM tracking more
  conservative while one foot is high.
- Added `support_zmp_margin=0.012` so transfer and single-support ZMP sit
  slightly outside the nominal support foot center, giving the high swing leg
  more lateral balance margin before the next foot leaves the ground.

### 2026-05-18 Final stand recovery requirement

- User observed that the robot can still fall after the last step and requested
  the final step to recover the initial stable upright state.
- `STAND`/`DONE` should no longer continue the walking IK/ZMP preview path.
  After the last touchdown and double-support settle, the controller must
  smoothly interpolate both legs from the final walking command back to the
  startup joint feedback posture and keep publishing that posture.
- `stand_time` is now `2.0` seconds, and the final recovery uses a reduced
  joint rate limit so the robot straightens gradually instead of snapping.
- During `STAND`/`DONE`, support is treated as `both`, arm targets return to
  feedback baselines, and pelvis/footstep preview logic is bypassed.

### 2026-05-18 Stability rollback after high-lift test

- A normal `walk` test with `step_length=0.055`, `foot_clearance=0.05`, and
  `support_zmp_margin=0.012` ABORTed at step 2 before reaching final stand.
- Kept the user-requested direction and final stand recovery, but reduced the
  aggressive gait defaults to still-improved conservative values:
  `step_length=0.045`, `foot_clearance=0.045`,
  `support_zmp_margin=0.004`, `max_com_speed=0.075`,
  `max_com_accel=0.20`.
- Added graceful handling for ROS external shutdown so timeout-based tests do
  not print a traceback.

### 2026-05-18 Final stand recovery validated

- With `step_length=0.045`, `foot_clearance=0.045`, `step_time=1.75`,
  `double_support_time=0.55`, `support_zmp_margin=0.004`,
  `max_com_speed=0.075`, and `max_com_accel=0.20`, a normal `walk` run in
  CoppeliaSim completed all 6 planned steps.
- The state sequence reached `STAND` at step 6, then `DONE`, and remained
  upright with final pitch/roll around `0.1 deg` or less in the log.
- This was a manual `walk` validation, not an auto-tune selected profile.


# asti.ttt script
## asti
```lua
-- CoppeliaSim Asti model script, ROS2 version.
-- Replace the script attached to the Asti model with this file.
-- This version keeps the original leg control and adds robust arm control.

function getIndexedObjects(aliasName, maxCount)
    local out = {}
    for i = 0, maxCount - 1, 1 do
        local ok, h = pcall(sim.getObject, './' .. aliasName, {index = i})
        if ok and h and h ~= -1 then
            out[#out + 1] = h
        else
            break
        end
    end
    return out
end

function getTreeJoints(aliasName)
    local out = {}
    local ok, base = pcall(sim.getObject, './' .. aliasName)
    if not ok or not base or base == -1 then
        return out
    end

    local okType, objType = pcall(sim.getObjectType, base)
    if okType and objType == sim.object_joint_type then
        out[#out + 1] = base
    end

    local okTree, tree = pcall(sim.getObjectsInTree, base, sim.object_joint_type, 0)
    if okTree and tree then
        for i = 1, #tree, 1 do
            local h = tree[i]
            local already = false
            for j = 1, #out, 1 do
                if out[j] == h then
                    already = true
                    break
                end
            end
            if not already then
                out[#out + 1] = h
            end
        end
    end
    return out
end

function getJointsRobust(aliasName, maxCount)
    -- First use the same indexed-name method as the original leg code.
    local joints = getIndexedObjects(aliasName, maxCount)

    -- If that found only one or no joint, use tree traversal as fallback.
    if #joints <= 1 then
        local treeJoints = getTreeJoints(aliasName)
        if #treeJoints > #joints then
            joints = treeJoints
        end
    end

    return joints
end

function setJointTargetSafe(jointHandle, targetPosition, forceLimit)
    -- Keep visible motion even if a joint is not dynamic.
    -- Use protected calls so this script survives different CoppeliaSim versions.
    if sim.isDynamicallyEnabled(jointHandle) then
        pcall(sim.setJointTargetForce, jointHandle, forceLimit)
        pcall(sim.setJointTargetPosition, jointHandle, targetPosition)
    else
        pcall(sim.setJointPosition, jointHandle, targetPosition)
    end
end

function sysCall_init()
    asti = sim.getObject('.')
    lFootTip = sim.getObject('./leftFootTip')
    rFootTip = sim.getObject('./rightFootTip')
    lFoot = sim.getObject('./leftFootTarget')
    rFoot = sim.getObject('./rightFootTarget')
    pathRef = sim.getObject('./astiPathRef')

    -- =========================
    -- Leg joints: keep the original structure exactly.
    -- =========================
    leftJointHandles = {}
    rightJointHandles = {}

    for i = 1, 6, 1 do
        leftJointHandles[i] = sim.getObject('./leftLegJoint', {index = i - 1})
        rightJointHandles[i] = sim.getObject('./rightLegJoint', {index = i - 1})
    end

    -- =========================
    -- Arm joints: robust lookup.
    -- This is the important fix. Do not require an exact hard-coded count.
    -- =========================
    leftArmJointHandles = getJointsRobust('leftArmJoint', 12)
    rightArmJointHandles = getJointsRobust('rightArmJoint', 12)

    sim.addLog(sim.verbosity_scriptinfos, 'Left leg joint count: ' .. #leftJointHandles)
    sim.addLog(sim.verbosity_scriptinfos, 'Right leg joint count: ' .. #rightJointHandles)
    sim.addLog(sim.verbosity_scriptinfos, 'Left arm joint count: ' .. #leftArmJointHandles)
    sim.addLog(sim.verbosity_scriptinfos, 'Right arm joint count: ' .. #rightArmJointHandles)

    for i = 1, #leftArmJointHandles, 1 do
        sim.addLog(sim.verbosity_scriptinfos, 'Left arm joint ' .. i .. ': ' .. sim.getObjectAlias(leftArmJointHandles[i], 1))
    end
    for i = 1, #rightArmJointHandles, 1 do
        sim.addLog(sim.verbosity_scriptinfos, 'Right arm joint ' .. i .. ': ' .. sim.getObjectAlias(rightArmJointHandles[i], 1))
    end

    controlMode = 'MOVEJ'

    leftLegTargetJoints = {0, 0, 0, 0, 0, 0}
    rightLegTargetJoints = {0, 0, 0, 0, 0, 0}

    for i = 1, 6, 1 do
        leftLegTargetJoints[i] = sim.getJointPosition(leftJointHandles[i])
        rightLegTargetJoints[i] = sim.getJointPosition(rightJointHandles[i])
    end

    leftArmTargetJoints = {}
    rightArmTargetJoints = {}

    for i = 1, #leftArmJointHandles, 1 do
        leftArmTargetJoints[i] = sim.getJointPosition(leftArmJointHandles[i])
    end
    for i = 1, #rightArmJointHandles, 1 do
        rightArmTargetJoints[i] = sim.getJointPosition(rightArmJointHandles[i])
    end

    -- Use moderate joint forces. If the robot still collapses from weak motors,
    -- increase these values inside CoppeliaSim joint dynamics settings too.
    legForceLimit = 120.0
    armForceLimit = 35.0

    -- Original correction parameters kept.
    leftCorrectionMult = {1, 1, 1}
    leftCorrectionOff = {0.0, 0, 0.031}
    rightCorrectionMult = {1, 1, 1}
    rightCorrectionOff = {0.0, 0, 0.03}
    vel = 0.75

    -- IK kept from original scene script.
    ikEnv = simIK.createEnvironment()

    ikGroupLeftLeg = simIK.createGroup(ikEnv)
    simIK.addElementFromScene(
        ikEnv,
        ikGroupLeftLeg,
        asti,
        lFootTip,
        lFoot,
        simIK.constraint_pose
    )

    ikGroupRightLeg = simIK.createGroup(ikEnv)
    simIK.addElementFromScene(
        ikEnv,
        ikGroupRightLeg,
        asti,
        rFootTip,
        rFoot,
        simIK.constraint_pose
    )

    firstArmCommandLogged = false

    -- =========================
    -- ROS2
    -- =========================
    if simROS2 then
        sim.addLog(sim.verbosity_scriptinfos, 'ROS2 interface was found.')

        modeSub = simROS2.createSubscription(
            '/mode',
            'std_msgs/msg/String',
            'change_mode'
        )

        leftLegJointsPub = simROS2.createPublisher(
            '/leftLegJoints',
            'std_msgs/msg/Float64MultiArray'
        )

        rightLegJointsPub = simROS2.createPublisher(
            '/rightLegJoints',
            'std_msgs/msg/Float64MultiArray'
        )

        leftArmJointsPub = simROS2.createPublisher(
            '/leftArmJoints',
            'std_msgs/msg/Float64MultiArray'
        )

        rightArmJointsPub = simROS2.createPublisher(
            '/rightArmJoints',
            'std_msgs/msg/Float64MultiArray'
        )

        leftFootPosePub = simROS2.createPublisher(
            '/leftFootPose',
            'std_msgs/msg/Float64MultiArray'
        )

        rightFootPosePub = simROS2.createPublisher(
            '/rightFootPose',
            'std_msgs/msg/Float64MultiArray'
        )

        legTargetJointsSub = simROS2.createSubscription(
            '/legTargetJoints',
            'std_msgs/msg/Float64MultiArray',
            'set_leg_target_joints_position'
        )

        armTargetJointsSub = simROS2.createSubscription(
            '/armTargetJoints',
            'std_msgs/msg/Float64MultiArray',
            'set_arm_target_joints_position'
        )

        sim.addLog(sim.verbosity_scriptinfos, 'Subscribed: /legTargetJoints')
        sim.addLog(sim.verbosity_scriptinfos, 'Subscribed: /armTargetJoints')
    else
        sim.addLog(sim.verbosity_scripterrors, 'ROS2 interface was not found. Cannot run.')
    end
end

function sysCall_cleanup()
    if ikEnv then
        simIK.eraseEnvironment(ikEnv)
    end

    if simROS2 then
        if modeSub then simROS2.shutdownSubscription(modeSub) end
        if legTargetJointsSub then simROS2.shutdownSubscription(legTargetJointsSub) end
        if armTargetJointsSub then simROS2.shutdownSubscription(armTargetJointsSub) end

        if leftLegJointsPub then simROS2.shutdownPublisher(leftLegJointsPub) end
        if rightLegJointsPub then simROS2.shutdownPublisher(rightLegJointsPub) end
        if leftArmJointsPub then simROS2.shutdownPublisher(leftArmJointsPub) end
        if rightArmJointsPub then simROS2.shutdownPublisher(rightArmJointsPub) end
        if leftFootPosePub then simROS2.shutdownPublisher(leftFootPosePub) end
        if rightFootPosePub then simROS2.shutdownPublisher(rightFootPosePub) end
    end
end

function change_mode(msg)
    controlMode = msg.data
end

function set_leg_target_joints_position(msg)
    if not msg or not msg.data or #msg.data < 12 then
        sim.addLog(sim.verbosity_scripterrors, 'Invalid /legTargetJoints message. Need 12 values.')
        return
    end

    local data = msg.data
    for i = 1, 6, 1 do
        leftLegTargetJoints[i] = data[i]
        rightLegTargetJoints[i] = data[i + 6]
    end
end

function set_arm_target_joints_position(msg)
    if not msg or not msg.data then
        sim.addLog(sim.verbosity_scripterrors, 'Invalid /armTargetJoints message.')
        return
    end

    local data = msg.data
    local half = math.floor(#data / 2)
    local leftCount = math.min(#leftArmJointHandles, half)
    local rightCount = math.min(#rightArmJointHandles, #data - half)

    -- Important: no hard rejection on count mismatch. Apply as many as possible.
    for i = 1, leftCount, 1 do
        leftArmTargetJoints[i] = data[i]
    end
    for i = 1, rightCount, 1 do
        rightArmTargetJoints[i] = data[half + i]
    end

    if not firstArmCommandLogged then
        firstArmCommandLogged = true
        sim.addLog(
            sim.verbosity_scriptinfos,
            'First /armTargetJoints received: data=' .. #data ..
            ', applied left=' .. leftCount .. ', right=' .. rightCount
        )
    end
end

function sysCall_sensing()
    if not simROS2 then
        return
    end

    local leftLegJointsMsg = {}
    local rightLegJointsMsg = {}
    local leftJoints = {0, 0, 0, 0, 0, 0}
    local rightJoints = {0, 0, 0, 0, 0, 0}

    for i = 1, 6, 1 do
        leftJoints[i] = sim.getJointPosition(leftJointHandles[i])
        rightJoints[i] = sim.getJointPosition(rightJointHandles[i])
    end

    leftLegJointsMsg.data = leftJoints
    rightLegJointsMsg.data = rightJoints
    simROS2.publish(leftLegJointsPub, leftLegJointsMsg)
    simROS2.publish(rightLegJointsPub, rightLegJointsMsg)

    local leftArmJointsMsg = {}
    local rightArmJointsMsg = {}
    local leftArmJoints = {}
    local rightArmJoints = {}

    for i = 1, #leftArmJointHandles, 1 do
        leftArmJoints[i] = sim.getJointPosition(leftArmJointHandles[i])
    end
    for i = 1, #rightArmJointHandles, 1 do
        rightArmJoints[i] = sim.getJointPosition(rightArmJointHandles[i])
    end

    leftArmJointsMsg.data = leftArmJoints
    rightArmJointsMsg.data = rightArmJoints
    simROS2.publish(leftArmJointsPub, leftArmJointsMsg)
    simROS2.publish(rightArmJointsPub, rightArmJointsMsg)

    local leftFootPoseMsg = {}
    local rightFootPoseMsg = {}
    leftFootPoseMsg.data = sim.getObjectPose(lFootTip, pathRef)
    rightFootPoseMsg.data = sim.getObjectPose(rFootTip, pathRef)
    simROS2.publish(leftFootPosePub, leftFootPoseMsg)
    simROS2.publish(rightFootPosePub, rightFootPoseMsg)
end

function sysCall_actuation()
    for i = 1, #leftJointHandles, 1 do
        setJointTargetSafe(leftJointHandles[i], leftLegTargetJoints[i], legForceLimit)
    end

    for i = 1, #rightJointHandles, 1 do
        setJointTargetSafe(rightJointHandles[i], rightLegTargetJoints[i], legForceLimit)
    end

    for i = 1, #leftArmJointHandles, 1 do
        if leftArmTargetJoints[i] then
            setJointTargetSafe(leftArmJointHandles[i], leftArmTargetJoints[i], armForceLimit)
        end
    end

    for i = 1, #rightArmJointHandles, 1 do
        if rightArmTargetJoints[i] then
            setJointTargetSafe(rightArmJointHandles[i], rightArmTargetJoints[i], armForceLimit)
        end
    end
end
```
## Sensor
```lua
function sysCall_init()
    model = sim.getObject('.')
    ref = sim.getObject('./reference')
    massObject = sim.getObject('./mass')
    sensor = sim.getObject('./forceSensor')

    mass = sim.getObjectFloatParam(massObject, sim.shapefloatparam_mass)

    oldTransformationMatrix = sim.getObjectMatrix(ref, sim.handle_world)
    oldGyro = {0, 0, 0}
    lastTime = sim.getSimulationTime()

    if simROS2 then
        sim.addLog(sim.verbosity_scriptinfos, "ROS2 interface was found.")

        accPub = simROS2.createPublisher(
            '/robot/acc',
            'std_msgs/msg/Float64MultiArray'
        )

        velPub = simROS2.createPublisher(
            '/robot/vel',
            'std_msgs/msg/Float64MultiArray'
        )

        gyroPub = simROS2.createPublisher(
            '/robot/angVel',
            'std_msgs/msg/Float64MultiArray'
        )

        angAccPub = simROS2.createPublisher(
            '/robot/angAcc',
            'std_msgs/msg/Float64MultiArray'
        )

        positionPub = simROS2.createPublisher(
            '/robot/pos',
            'std_msgs/msg/Float64MultiArray'
        )

        orientationPub = simROS2.createPublisher(
            '/robot/ori',
            'std_msgs/msg/Float64MultiArray'
        )
    else
        sim.addLog(sim.verbosity_scripterrors, "ROS2 interface was not found. Cannot run.")
    end
end


function sysCall_sensing()
    if not simROS2 then
        return
    end

    local accMsg = {}
    local gyroMsg = {}
    local velMsg = {}
    local angAccMsg = {}
    local posMsg = {}
    local oriMsg = {}

    -- acceleration from force sensor
    local result, force = sim.readForceSensor(sensor)

    if result > 0 then
        local accel = {
            force[1] / mass,
            force[2] / mass,
            force[3] / mass + 9.81
        }

        accMsg.data = accel
        simROS2.publish(accPub, accMsg)
    end

    -- transform difference
    local transformationMatrix = sim.getObjectMatrix(ref, sim.handle_world)

    local oldInverse = sim.copyTable(oldTransformationMatrix)
    oldInverse = sim.getMatrixInverse(oldInverse)

    local m = sim.multiplyMatrices(oldInverse, transformationMatrix)

    local currentTime = sim.getSimulationTime()
    local dt = currentTime - lastTime

    oldTransformationMatrix = sim.copyTable(transformationMatrix)
    lastTime = currentTime

    -- angular velocity
    local euler = sim.getEulerAnglesFromMatrix(m)
    local gyroData = {0, 0, 0}

    if dt ~= 0 then
        gyroData[1] = euler[1] / dt
        gyroData[2] = euler[2] / dt
        gyroData[3] = euler[3] / dt

        gyroMsg.data = gyroData
        simROS2.publish(gyroPub, gyroMsg)
    end

    -- linear velocity
    local velData = {0, 0, 0}

    if dt ~= 0 then
        velData[1] = m[4] / dt
        velData[2] = m[8] / dt
        velData[3] = m[12] / dt

        velMsg.data = velData
        simROS2.publish(velPub, velMsg)
    end

    -- angular acceleration
    local angAcc = {0, 0, 0}

    if dt ~= 0 then
        angAcc[1] = (gyroData[1] - oldGyro[1]) / dt
        angAcc[2] = (gyroData[2] - oldGyro[2]) / dt
        angAcc[3] = (gyroData[3] - oldGyro[3]) / dt

        angAccMsg.data = angAcc
        simROS2.publish(angAccPub, angAccMsg)
    end

    oldGyro = gyroData

    -- orientation
    local oriData = sim.getEulerAnglesFromMatrix(transformationMatrix)
    oriMsg.data = oriData
    simROS2.publish(orientationPub, oriMsg)

    -- position
    local posData = {
        transformationMatrix[4],
        transformationMatrix[8],
        transformationMatrix[12]
    }

    posMsg.data = posData
    simROS2.publish(positionPub, posMsg)
end


function sysCall_cleanup()
    if simROS2 then
        if accPub then
            simROS2.shutdownPublisher(accPub)
        end

        if velPub then
            simROS2.shutdownPublisher(velPub)
        end

        if gyroPub then
            simROS2.shutdownPublisher(gyroPub)
        end

        if angAccPub then
            simROS2.shutdownPublisher(angAccPub)
        end

        if positionPub then
            simROS2.shutdownPublisher(positionPub)
        end

        if orientationPub then
            simROS2.shutdownPublisher(orientationPub)
        end
    end
end
```
