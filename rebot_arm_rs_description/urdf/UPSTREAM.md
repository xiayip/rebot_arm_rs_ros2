# Upstream reBot Arm RS model

The six-axis kinematic geometry, collision meshes, and inertial parameters in this package are
synchronized from Seeed Studio's official RS model:

- Repository: `https://github.com/Seeed-Projects/reBotArm_control_py`
- Upstream branch: `main`
- Verified upstream HEAD: `edf9279905f7ba399da0b5d81ef60fd359851d0c`
- Source URDF: `urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf`
- Source URDF Git blob: `81cff74391249ed2c9da1c3e5278e3ff2c86ddc7`
- Source URDF SHA-256: `2012b5aa3b58878109cb9e3c5deef919a87bd09a67d561662b2904a30dd4397e`
- Synchronization date: 2026-08-13

The upstream source contains a motorized gripper. This robot currently has no gripper, so the ROS
description intentionally ends at the massless `tool0` frame fixed to `link6`; gripper links,
inertials, joints, and meshes are not imported.

The upstream joint origins and axes match this package. Existing driver operating limits are
retained as a separate safety policy rather than being replaced by the CAD-export limits. The two
sets are not consistently wider or narrower (for example, the upstream J3 upper limit is `3.14`
rad while the current driver uses `3.50` rad), and the limits published in the product Wiki also
differ from the CAD export. They must be reconciled and hardware-validated independently.

The seven aggregate collision meshes in this package are byte-identical to the corresponding
upstream `base_link.STL` and `link1.STL` through `link6.STL` files.
