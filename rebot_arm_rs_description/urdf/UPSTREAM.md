# Upstream reBot Arm RS model

The six-axis physical model and visual materials in this package are synchronized from Seeed
Studio's ROS 2 repository:

- Repository: `https://github.com/Seeed-Projects/reBotArmController_ROS2`
- Upstream branch: `main`
- Verified upstream HEAD: `e134941b71236523e831f15b470bc81186b0649f`
- Source URDF: `src/rebotarm_bringup/description/urdf/00-arm-rs_asm-v3.urdf`
- Source URDF Git blob: `0f70c833897af37225f089622d58fbcd58d4153f`
- Source URDF SHA-256: `807c17cda3a5193fc4aadad5b73aebecc16e53e60707b3fa8901c90ac48ca551`
- Source meshes: `src/rebotarm_bringup/description/meshes_rs`
- Upstream package license declaration: `Apache-2.0`
- Synchronization date: 2026-08-21

For `base_link` through `link6`, joint origins, axes, parent/child relationships, link inertial
parameters, effort limits, position limits, URDF velocity limits, collision meshes, visual meshes,
material assignments, and palette RGBA values match the upstream model numerically.

The upstream source includes a motorized gripper. This robot currently has no gripper, so the local
description ends at a massless `tool0` fixed to `link6`. Its transform is identical to upstream
`j_gripper_end`; gripper links, inertials, joints, and meshes are intentionally omitted.

## Local ROS 2 extensions

The upstream model does not contain this package's `ros2_control` hardware description. Position
command limits are kept equal to the upstream URDF. J4-J6 use the RS-00 protocol velocity encoding
range of `33 rad/s`, rather than the CAD URDF's `40 rad/s`; `33 rad/s` is fixed by the RobStride
RS-00 codec and is not a physical-model difference.

Each visual repeats its palette RGBA inline as well as referencing the shared material name. This is
intentional because URDF consumers differ in whether they resolve robot-level materials into
visual-level objects.

Jazzy RViz loads every entry in a link's `visual_array`, but resolves an empty per-entry material
name back to the link's primary visual material. Secondary split meshes are therefore represented as
zero-offset, fixed, visual-only child links. Each rendered link has exactly one visual and retains its
own upstream material, while the physical six-axis links keep all upstream inertial and collision
data.