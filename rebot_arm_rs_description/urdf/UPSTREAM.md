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

For `base_link` through `link5`, joint origins, axes, parent/child relationships, link inertial
parameters, effort limits, position limits, URDF velocity limits, original collision meshes, visual meshes,
material assignments, and palette RGBA values match the upstream model numerically. The upstream
`link6` body, mesh, and inertial parameters are preserved numerically in the separate
`rebot_arm_rs_stock_flange` macro. Its fixed attachment to the empty `link6` output frame is
kinematically and dynamically equivalent to the original model while allowing flange replacement.

The default standalone description still ends at the stock flange's `tool0` frame. The optional
`rebot_arm_rs_with_official_parallel_gripper.urdf.xacro` composes the same six-axis core and stock
flange with Seeed's official parallel gripper.

## Official parallel gripper

The optional gripper model is synchronized from Seeed Studio's hardware-design repository:

- Repository: `https://github.com/Seeed-Projects/reBot-DevArm`
- Upstream branch: `main`
- Verified upstream HEAD: `cfdc0a92312ae232d0fdb98fff847269a382380a`
- Source URDF: `Rebot_Arm_description/RS/urdf/ReBot_Arm_RS.urdf`
- Source URDF SHA-256: `3838ca8f9d586be22a89b33c7cbdd4abd43d4a92a2de377b294eef4c5bd7654d`
- Source meshes: `Rebot_Arm_description/RS/meshes/{shared,visual}`
- Hardware-design license: `CERN-OHL-W-2.0`
- Synchronization date: 2026-09-28

The gripper body and finger inertials, the flange-to-gripper transform, both prismatic joints,
their axes and limits, collision meshes, visual meshes, and material assignments match that source.
Because this package models the upstream physical `link6` as the separate
`rebot_arm_rs_stock_flange_link`, the gripper's fixed joint is attached to that link instead of the
empty, stable `link6` output frame. This is kinematically and dynamically equivalent to the source.

The only presentation change is that each visual component is placed on a zero-offset fixed child
link. This works around differing multi-visual material handling among URDF consumers and does not
change the physical model. The imported assets and this modification are distributed under
`LICENSES/CERN-OHL-W-2.0.txt`; the rest of this ROS integration retains its declared licenses.

The source defines two independent prismatic finger joints and no mimic relation. This package
preserves that definition and does not invent a gripper ros2_control interface or motor protocol.

## Local ROS 2 extensions

The upstream model does not contain this package's `ros2_control` hardware description. Position
command limits are kept equal to the upstream URDF. J4-J6 use the RS-00 protocol velocity encoding
range of `33 rad/s`, rather than the CAD URDF's `40 rad/s`; `33 rad/s` is fixed by the RobStride
RS-00 codec and is not a physical-model difference.

Each visual repeats its palette RGBA inline as well as referencing the shared material name. This is
intentional because URDF consumers differ in whether they resolve robot-level materials into
visual-level objects.

Arm visual definitions are isolated in `rebot_arm_rs_visuals.urdf.xacro`, while the replaceable
stock flange owns its visual directly. Jazzy RViz loads every entry
in a link's `visual_array`, but resolves an empty per-entry material name back to the link's primary
visual material. The split meshes are therefore represented as zero-offset, fixed, visual-only child
links. Each rendered link has exactly one visual and retains its own upstream material, while the
physical six-axis macro contains only inertial, collision, and joint data plus the stable,
massless `link6` output frame.

## Lightweight arm collision meshes

The six core links (`base_link`, `link1` through `link5`) now reference local
`meshes/rebot_arm_rs/collision/*.STL` derivatives of the original Seeed meshes.
Visual meshes, inertials, joint frames/limits, and controller interfaces stay
unchanged. The original full-resolution collision files remain alongside the
visual assets for provenance and verification.

These are compound convex approximations generated with V-HACD (100,000 voxel
resolution, up to 32 pieces, 32 seed vertices per piece). Each original surface
triangle is assigned to one piece; its supporting planes and local bounding
planes are expanded to contain that whole triangle with a nominal 0.1 mm margin.
Coordinates are rounded to 0.1 micrometre before float32 STL export. Each shell's
STL attribute stores a positive piece ID for the coverage regression test.

Total triangles: 691,710 -> 24,906 (96.4% reduction). These conservative meshes
can fill small holes/recesses and require more clearance than the dense CAD.
Stock flange and gripper collision geometry are unaffected by this change.
