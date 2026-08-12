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

## Dependency

The ROS 2 workspace must also contain the `shared_can_hardware` repository. It provides:

- `shared_can_interface`: the versioned transport/protocol ABI used by the RobStride plugin.
- `shared_can_ros2`: the generic ros2_control bus manager and SocketCAN transport.

See that repository's README for the shared-bus architecture, plugin contract, and instructions for
adding another motor, gripper, hand, or sensor protocol.

## Safety defaults

- The generic hardware plugin opens the CAN bus once and reads all configured device states during
  activation.
- `auto_enable_on_activate` defaults to `false`; no enable or motion command is sent in this mode.
- The trajectory controller is spawned inactive by the real-hardware launch file.
- Position commands are clamped to per-joint limits and to a configurable maximum step.
- J1-J3 are configured as RS-06, J4-J6 as RS-00, all with host ID `0xFD`.

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

## Start safely

```bash
# No physical hardware: validates the description and controller stack.
ros2 launch rebot_arm_rs_bringup mock.launch.py

# Physical CAN bus, read-only activation; trajectory controller stays inactive.
ros2 launch rebot_arm_rs_bringup real_world.launch.py can_interface:=can1
```

Enabling real motors is intentionally not part of the initial smoke test. It requires an explicit
`auto_enable_on_activate:=true` launch argument after the read-only path has passed.

## Source references

- Seeed B601-RS URDF kinematics and meshes: `Seeed-Projects/reBotArmController_ROS2`
- RobStride private CAN protocol behavior: RobStride manuals and MotorBridge 0.5.1
