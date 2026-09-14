# reBot Arm RS Isaac Sim asset

`rebot_arm_rs/rebot_arm_rs.usda` is generated from the package URDF and is kept in the
repository so simulation startup is fast and deterministic. Regenerate it after changing
the robot geometry, joints, inertial parameters, or joint limits.

From the `rebot_arm_rs_ros2` repository in the development container:

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/zephyr-dev/zephyr_ws/install/setup.bash
./rebot_arm_rs_description/scripts/generate_isaac_usd.sh
```

The checked-in asset was imported with Isaac Sim 6.0.1, a fixed base, force drives,
position targets, stiffness 200, and damping 20.

The Action Graph is intentionally created by `rebot_arm_rs_bringup` at runtime rather than saved
into this generated USD, so regenerating the asset cannot remove the ROS 2 interface.
