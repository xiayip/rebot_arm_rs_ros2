# rebot_arm_rs_ros2

Native ROS 2 control stack for the six-axis Seeed reBot Arm B601-RS.

The hardware plugin talks directly to RobStride RS motors over Linux SocketCAN. It does not depend
on MotorBridge at runtime; MotorBridge is retained as an independent protocol oracle during driver
development and hardware validation.

## Packages

| Package | Purpose |
| --- | --- |
| `rebot_arm_rs` | Aggregate package |
| `rebot_arm_rs_description` | Six-axis URDF/Xacro and ros2_control description |
| `rebot_arm_rs_hardware` | RobStride protocol, SocketCAN transport, and SystemInterface plugin |
| `rebot_arm_rs_bringup` | Controller configuration and launch files |

## Safety defaults

- The real hardware plugin opens the CAN bus and reads all six zeroed positions during activation.
- `auto_enable_on_activate` defaults to `false`; no enable or motion command is sent in this mode.
- The trajectory controller is spawned inactive by the real-hardware launch file.
- Position commands are clamped to per-joint limits and to a configurable maximum step.
- J1-J3 are configured as RS-06, J4-J6 as RS-00, all with host ID `0xFD`.

MotorBridge Gateway and this hardware plugin must not control the same CAN bus at the same time.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd /workspaces/zephyr-dev/zephyr_ws
colcon build --symlink-install --packages-up-to rebot_arm_rs
source install/setup.bash
```

Run `colcon` from `zephyr_ws`, not from `/workspaces/zephyr-dev`; the latter also contains Miniforge
and makes package discovery scan unrelated Python sources.

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
