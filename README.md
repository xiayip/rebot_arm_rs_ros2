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

- The generic hardware plugin opens the CAN bus once and reads all configured device states during
  activation.
- `auto_enable_on_activate` defaults to `true`; after all six initial positions are received, the
  hardware enables every configured motor and holds the measured startup positions.
- The trajectory controller is spawned active by default. Pass
  `activate_trajectory_controller:=false` to load and configure it without claiming `position`.
- Each joint exports the complete MIT command tuple as `position`, `velocity`, `kp`, `kd`, and
  `t_ff`. The trajectory controller claims only `position`; the remaining interfaces retain their
  configured startup values until a future MIT controller claims them.
- Runtime feedback has one source: every operation-status response updates position, velocity,
  effort, temperature, and fault state at the control rate. `0x7019` is read once during activation
  only to establish the safe initial hold position; it is not polled and cannot overwrite runtime
  state. Parameter reads remain available to separate diagnostic tools.
- All MIT commands are clamped to the model-specific limits. Position is additionally constrained
  to the joint limits and to a configurable maximum step.
- J1-J3 are configured as RS-06, J4-J6 as RS-00, all with host ID `0xFD`.

Pass `auto_enable_on_activate:=false` when a read-only diagnostic session is required.

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

# Physical CAN bus; all six motors enable and the trajectory controller becomes active.
ros2 launch rebot_arm_rs_bringup real_world.launch.py can_interface:=can1

# Optional read-only diagnostic startup with the trajectory controller inactive.
ros2 launch rebot_arm_rs_bringup real_world.launch.py \
  can_interface:=can1 auto_enable_on_activate:=false \
  activate_trajectory_controller:=false
```

## Source references

- Seeed B601-RS URDF kinematics and meshes: `Seeed-Projects/reBotArmController_ROS2`
- RobStride private CAN protocol behavior: RobStride manuals and MotorBridge 0.5.1
