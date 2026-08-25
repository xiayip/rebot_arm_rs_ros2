import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_isaac_sim(context, stage_loader_path):
    isaac_sim_root = LaunchConfiguration("isaac_sim_root").perform(context)
    usd_path = LaunchConfiguration("usd_path").perform(context)
    headless_value = LaunchConfiguration("headless").perform(context).lower()
    ros_distro = LaunchConfiguration("ros_distro").perform(context)
    rmw_implementation = LaunchConfiguration("rmw_implementation").perform(context)
    ros_domain_id = LaunchConfiguration("ros_domain_id").perform(context)
    stage_loader_path = stage_loader_path.perform(context)

    if headless_value not in ("true", "false"):
        raise RuntimeError("headless must be either 'true' or 'false'")

    isaac_sim_script = os.path.join(isaac_sim_root, "isaac-sim.sh")
    if not os.path.isfile(isaac_sim_script):
        raise RuntimeError(f"Isaac Sim launcher not found: {isaac_sim_script}")
    if not os.path.isfile(usd_path):
        raise RuntimeError(
            f"reBot Arm USD not found: {usd_path}. "
            "Regenerate it with rebot_arm_rs_description/scripts/generate_isaac_usd.sh."
        )
    if not os.path.isfile(stage_loader_path):
        raise RuntimeError(f"Isaac stage loader not found: {stage_loader_path}")

    ros_library_path = os.path.join(
        isaac_sim_root, "exts", "isaacsim.ros2.core", ros_distro, "lib"
    )
    library_paths = [path for path in (os.environ.get("LD_LIBRARY_PATH"), ros_library_path) if path]

    command = [
        isaac_sim_script,
        "--enable",
        "isaacsim.ros2.bridge",
        "--/isaac/startup/create_new_stage=false",
    ]
    if headless_value == "true":
        command.append("--no-window")
    command.extend(["--exec", stage_loader_path])

    return [
        ExecuteProcess(
            cmd=command,
            additional_env={
                "LD_LIBRARY_PATH": os.pathsep.join(library_paths),
                "MPLCONFIGDIR": "/tmp/rebot_arm_rs_isaac_matplotlib",
                "RMW_IMPLEMENTATION": rmw_implementation,
                "ROS_DISTRO": ros_distro,
                "ROS_DOMAIN_ID": ros_domain_id,
                "REBOT_ARM_RS_ISAAC_HEADLESS": headless_value,
                "REBOT_ARM_RS_ISAAC_USD_PATH": usd_path,
            },
            output="screen",
            emulate_tty=True,
            sigterm_timeout="30",
            sigkill_timeout="10",
        )
    ]


def generate_launch_description():
    default_usd_path = PathJoinSubstitution(
        [
            FindPackageShare("rebot_arm_rs_description"),
            "usd",
            "rebot_arm_rs",
            "rebot_arm_rs.usda",
        ]
    )
    stage_loader_path = PathJoinSubstitution(
        [
            FindPackageShare("rebot_arm_rs_bringup"),
            "scripts",
            "open_isaac_stage.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "isaac_sim_root",
                default_value="/opt/isaacsim",
                description="Isaac Sim installation directory.",
            ),
            DeclareLaunchArgument(
                "usd_path",
                default_value=default_usd_path,
                description="USD stage to open.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Isaac Sim without a window.",
            ),
            DeclareLaunchArgument(
                "ros_distro",
                default_value="jazzy",
                description="ROS distribution used by the Isaac ROS 2 Bridge.",
            ),
            DeclareLaunchArgument(
                "rmw_implementation",
                default_value="rmw_fastrtps_cpp",
                description="RMW implementation used by the Isaac ROS 2 Bridge.",
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value=EnvironmentVariable("ROS_DOMAIN_ID", default_value="0"),
                description="ROS domain used by the Isaac ROS 2 Bridge.",
            ),
            OpaqueFunction(
                function=_launch_isaac_sim,
                kwargs={"stage_loader_path": stage_loader_path},
            ),
        ]
    )
