"""Open and frame the reBot Arm USD in the running Isaac Sim editor."""

import asyncio
import os

import carb
import omni.kit.app
import omni.usd


USD_PATH_ENVIRONMENT_VARIABLE = "REBOT_ARM_RS_ISAAC_USD_PATH"
EXPECTED_ROBOT_PRIM_PATH = "/rebot_arm_rs"


def _quit_after_error(message):
    carb.log_error(message)
    omni.kit.app.get_app().post_quit(1)


async def _open_rebot_arm_stage():
    app = omni.kit.app.get_app()
    usd_path = os.environ.get(USD_PATH_ENVIRONMENT_VARIABLE)
    if not usd_path:
        _quit_after_error(f"{USD_PATH_ENVIRONMENT_VARIABLE} is not set")
        return

    usd_context = omni.usd.get_context()
    for _ in range(120):
        if usd_context.can_open_stage():
            break
        await app.next_update_async()
    else:
        _quit_after_error("Isaac USD context did not become ready to open a stage")
        return

    success, error = await usd_context.open_stage_async(usd_path)
    if not success:
        _quit_after_error(f"Failed to open reBot Arm USD '{usd_path}': {error}")
        return

    for _ in range(10):
        await app.next_update_async()

    stage = usd_context.get_stage()
    robot_prim = stage.GetPrimAtPath(EXPECTED_ROBOT_PRIM_PATH) if stage else None
    if not robot_prim or not robot_prim.IsValid():
        _quit_after_error(
            f"USD opened but expected robot prim {EXPECTED_ROBOT_PRIM_PATH} was not found"
        )
        return

    default_prim = stage.GetDefaultPrim()
    prim_count = sum(1 for _ in stage.Traverse())
    carb.log_info(
        "REBOT_ARM_RS_ISAAC_STAGE_LOADED "
        f"path={usd_path} default_prim={default_prim.GetPath()} prim_count={prim_count}"
    )

    if os.environ.get("REBOT_ARM_RS_ISAAC_HEADLESS", "false") != "true":
        from omni.kit.viewport.utility import frame_viewport_prims

        if not frame_viewport_prims(prims=[EXPECTED_ROBOT_PRIM_PATH]):
            carb.log_warn(
                "reBot Arm USD loaded, but the viewport could not be framed automatically"
            )


asyncio.ensure_future(_open_rebot_arm_stage())
