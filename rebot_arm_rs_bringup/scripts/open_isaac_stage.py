"""Open and frame the reBot Arm USD in the running Isaac Sim editor."""

import asyncio
import os

import carb
import omni.graph.core as og
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Usd, UsdPhysics
import usdrt


USD_PATH_ENVIRONMENT_VARIABLE = "REBOT_ARM_RS_ISAAC_USD_PATH"
EXPECTED_ROBOT_PRIM_PATH = "/rebot_arm_rs"
ROS2_CONTROL_GRAPH_PATH = "/RebotArmRsROS2ControlGraph"
JOINT_COMMANDS_TOPIC = "/rebot_arm_rs/isaac_joint_commands"
JOINT_STATES_TOPIC = "/rebot_arm_rs/isaac_joint_states"


def _quit_after_error(message):
    carb.log_error(message)
    omni.kit.app.get_app().post_quit(1)


def _find_articulation_root(stage, robot_prim_path):
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    for prim in Usd.PrimRange(robot_prim):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    return None


def _create_ros2_control_graph(articulation_root_path):
    og.Controller.edit(
        {
            "graph_path": ROS2_CONTROL_GRAPH_PATH,
            "evaluator_name": "execution",
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                (
                    "ArticulationController",
                    "isaacsim.core.nodes.IsaacArticulationController",
                ),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
                ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
                (
                    "ReadJointState.outputs:jointPositions",
                    "PublishJointState.inputs:jointPositions",
                ),
                (
                    "ReadJointState.outputs:jointVelocities",
                    "PublishJointState.inputs:jointVelocities",
                ),
                (
                    "ReadJointState.outputs:jointEfforts",
                    "PublishJointState.inputs:jointEfforts",
                ),
                (
                    "ReadJointState.outputs:jointDofTypes",
                    "PublishJointState.inputs:jointDofTypes",
                ),
                (
                    "ReadJointState.outputs:stageMetersPerUnit",
                    "PublishJointState.inputs:stageMetersPerUnit",
                ),
                ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                (
                    "SubscribeJointState.outputs:jointNames",
                    "ArticulationController.inputs:jointNames",
                ),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                (
                    "SubscribeJointState.outputs:effortCommand",
                    "ArticulationController.inputs:effortCommand",
                ),
            ],
            og.Controller.Keys.SET_VALUES: [
                (
                    "ArticulationController.inputs:robotPath",
                    articulation_root_path,
                ),
                (
                    "ReadJointState.inputs:prim",
                    [usdrt.Sdf.Path(articulation_root_path)],
                ),
                ("PublishJointState.inputs:topicName", JOINT_STATES_TOPIC),
                ("SubscribeJointState.inputs:topicName", JOINT_COMMANDS_TOPIC),
            ],
        },
    )


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

    articulation_root_path = _find_articulation_root(
        stage, EXPECTED_ROBOT_PRIM_PATH
    )
    if articulation_root_path is None:
        _quit_after_error(
            f"No articulation root was found below {EXPECTED_ROBOT_PRIM_PATH}"
        )
        return
    try:
        _create_ros2_control_graph(articulation_root_path)
    except Exception as error:
        _quit_after_error(f"Failed to create reBot Arm ROS 2 graph: {error}")
        return

    omni.timeline.get_timeline_interface().play()

    default_prim = stage.GetDefaultPrim()
    prim_count = sum(1 for _ in stage.Traverse())
    carb.log_info(
        "REBOT_ARM_RS_ISAAC_STAGE_LOADED "
        f"path={usd_path} articulation_root={articulation_root_path} "
        f"default_prim={default_prim.GetPath()} prim_count={prim_count}"
    )
    carb.log_info(
        "REBOT_ARM_RS_ISAAC_ROS2_GRAPH_READY "
        f"graph={ROS2_CONTROL_GRAPH_PATH} state_topic={JOINT_STATES_TOPIC} "
        f"command_topic={JOINT_COMMANDS_TOPIC}"
    )

    if os.environ.get("REBOT_ARM_RS_ISAAC_HEADLESS", "false") != "true":
        from omni.kit.viewport.utility import frame_viewport_prims

        if not frame_viewport_prims(prims=[EXPECTED_ROBOT_PRIM_PATH]):
            carb.log_warn(
                "reBot Arm USD loaded, but the viewport could not be framed automatically"
            )


asyncio.ensure_future(_open_rebot_arm_stage())
