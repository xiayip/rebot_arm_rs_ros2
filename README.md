# rebot_arm_rs_ros2

Native ROS 2 control stack for the six-axis Seeed reBot Arm B601-RS.

The driver uses the reusable `shared_can_hardware` stack for ros2_control and SocketCAN, with a
RobStride protocol plugin provided by this repository. It does not depend on MotorBridge at
runtime; MotorBridge is retained as an independent protocol oracle during driver development and
hardware validation.

## Packages

| Package | Purpose |
| --- | --- |
| `rebot_arm_rs` | Aggregate package |
| `rebot_arm_rs_description` | Six-axis URDF/Xacro and ros2_control description |
| `rebot_robstride_protocol` | RobStride private-CAN codec and protocol plugin |
| `rebot_arm_rs_bringup` | Controller configuration and launch files |

The six-axis URDF inertial data and aggregate meshes are synchronized from Seeed Studio's official
RS CAD export. See `rebot_arm_rs_description/urdf/UPSTREAM.md` for the pinned upstream commit,
checksums, and the no-gripper adaptation policy.

## Dependency

The ROS 2 workspace must also contain the `shared_can_hardware` repository. It provides:

- `shared_can_interface`: the versioned transport/protocol ABI used by the RobStride plugin.
- `shared_can_ros2`: the generic ros2_control bus manager and SocketCAN transport.

See that repository's README for the shared-bus architecture, plugin contract, and instructions for
adding another motor, gripper, hand, or sensor protocol.

## Hardware startup behavior

- The generic hardware plugin opens the CAN bus once and reads each motor's position, `run_mode`,
  and filtered phase current during activation.
- The startup position command preserves the existing load preload. The driver reads filtered
  phase current (`iqf`), converts peak current to output torque with the model torque constant, and
  applies the bounded offset `torque / mit_kp` to the measured position. This prevents resetting a
  loaded impedance hold to zero torque when ros2_control starts.
- Activation always enables the arm. The driver first sends the preload-preserving MIT hold as a
  probe: motors already in Motor mode are not enabled again, while stopped motors receive adjacent
  enable and hold frames. Motors already in MIT `run_mode` are not reconfigured.
- The trajectory controller is spawned as the head of a three-controller chain by default:
  `JointTrajectoryController -> MIT position adapter -> MIT output controller`. Pass
  `activate_trajectory_controller:=false` to keep the MIT output controller active in safe HOLD
  while loading the trajectory controller and position adapter as inactive.
- Each joint exports the complete MIT command tuple as `position`, `velocity`, `kp`, `kd`, and
  `t_ff`. The MIT output controller exclusively claims that tuple and exposes chainable reference
  interfaces. The position adapter completes each JTC position reference with the configured
  position-mode gains, the JTC velocity reference, and an optional Pinocchio gravity
  feed-forward torque.
  The per-joint `position_kp` and `position_kd` arrays are runtime-adjustable and are transferred
  into the realtime loop without requiring controller restart.
  The default real-hardware configuration applies the commissioned affine gravity calibration for
  joints 2-4 at `gravity_compensation.factor=1.0`; the remaining joint scales are zero.
- The gravity model is built from the same `robot_description` used by ros2_control. Controller
  activation preserves the previous MIT tuple's equivalent torque, then rate-limits and blends the
  command into `g(q)` instead of stepping the feed-forward command. The runtime factor is limited
  to `[0, 1]` and follows changes over the configured `factor_transition_duration`.
- The position adapter publishes `~/mit_state` as `control_msgs/msg/DynamicJointState` at the
  configured rate. For every joint it reports the position and velocity references, raw and
  scaled gravity torque, runtime gravity factor, applied feed-forward torque, P and D feedback
  torques, and their summed MIT command torque.
- The joint impedance controller sends its stiffness and damping through the motor-side MIT
  `kp`/`kd` loop. The ROS realtime loop supplies calibrated gravity and URDF soft-limit torque
  through shaped `t_ff`, captures the current posture as the activation equilibrium, and accepts
  rate-limited references from a dedicated impedance JTC. Per-joint spring torque limits bound
  `kp * (q_ref - q)` by reducing the motor equilibrium error without moving feedback back into the
  ROS loop. The standalone `~/commands` topic remains available while the impedance controller is
  not chained to its JTC.
- The impedance trajectory guard monitors tracking error and measured velocity. If any joint stays
  beyond its configured error threshold while approximately stationary for the configured blocking
  duration, it cancels the active impedance JTC goal. JTC cancellation captures the measured
  posture as a zero-velocity hold. Its wider path tolerances remain a second-level fallback for
  larger tracking errors.
- Runtime feedback has one source: every operation-status response updates position, velocity,
  effort, temperature, and fault state at the control rate. `0x7019` is read once during activation
  only to establish the safe initial hold position; it is not polled and cannot overwrite runtime
  state. Parameter reads remain available to separate diagnostic tools.
- All MIT commands are clamped to the model-specific limits. Position is additionally constrained
  to the joint limits and to a configurable maximum step.
- J1-J3 are configured as RS-06, J4-J6 as RS-00, all with host ID `0xFD`.
- On a normal Ctrl-C shutdown the driver never sends motor-disable frames, so each motor keeps
  executing its last MIT position command and holds the arm instead of immediately losing torque.
  Holding still depends on uninterrupted motor power and the motor-side CAN timeout configuration;
  a process cannot guarantee holding after a power loss or an independently configured
  communication-timeout shutdown.

MotorBridge Gateway and this hardware plugin must not control the same CAN bus at the same time.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install --packages-up-to rebot_arm_rs
source install/setup.bash
```

Run `colcon` from the ROS 2 workspace root rather than from one of its parent directories; this
prevents package discovery from scanning unrelated Python sources.

## Start

```bash
# No physical hardware: validates the description and controller stack.
ros2 launch rebot_arm_rs_bringup mock.launch.py

# Physical CAN bus; stopped motors enable, and Ctrl-C retains the last motor-side hold command.
ros2 launch rebot_arm_rs_bringup real_world.launch.py can_interface:=can1

# Optional startup with the trajectory controller loaded but inactive.
# The hardware driver still enables and holds the arm.
ros2 launch rebot_arm_rs_bringup real_world.launch.py \
  can_interface:=can1 activate_trajectory_controller:=false

# Tune the position-mode gains online. Each array must contain one nonnegative value per joint.
ros2 param set /rebot_arm_rs_position_adapter \
  position_kp "[50.0, 120.0, 120.0, 50.0, 50.0, 50.0]"
ros2 param set /rebot_arm_rs_position_adapter \
  position_kd "[3.0, 4.0, 4.0, 5.0, 4.0, 4.0]"

# Inspect the realtime MIT command decomposition.
ros2 topic echo /rebot_arm_rs_position_adapter/mit_state

# Switch from the stiff trajectory chain to the compliant trajectory chain.
ros2 control switch_controllers \
  --deactivate rebot_arm_rs_trajectory_controller rebot_arm_rs_position_adapter \
  --activate rebot_arm_rs_impedance_controller \
             rebot_arm_rs_impedance_trajectory_controller \
  --strict -c /controller_manager

# Send a trajectory through the impedance JTC.
ros2 action send_goal \
  /rebot_arm_rs_impedance_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint2], points: [{positions: [0.3], \
  time_from_start: {sec: 3, nanosec: 0}}]}}"

# Return to the stiff trajectory chain.
ros2 control switch_controllers \
  --deactivate rebot_arm_rs_impedance_trajectory_controller \
               rebot_arm_rs_impedance_controller \
  --activate rebot_arm_rs_position_adapter rebot_arm_rs_trajectory_controller \
  --strict -c /controller_manager

# Inspect the desired/limited equilibrium, MIT gains, and torque decomposition.
ros2 topic echo /rebot_arm_rs_impedance_controller/state
```

## Source references

- Seeed B601-RS URDF kinematics and meshes: `Seeed-Projects/reBotArmController_ROS2`
- RobStride private CAN protocol behavior: RobStride manuals and MotorBridge 0.5.1
