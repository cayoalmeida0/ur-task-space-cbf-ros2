import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ur_cbf_bringup.grippers import SUPPORTED_ONROBOT_TYPES, get_gripper_spec
from ur_cbf_bringup.models import SUPPORTED_UR_TYPES


def _launch_boolean(value: str, argument_name: str) -> bool:
    normalized = value.strip().lower()
    if normalized not in {"true", "false"}:
        raise ValueError(f"{argument_name} deve ser true ou false.")
    return normalized == "true"


def launch_setup(context):
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless_mode = LaunchConfiguration("headless_mode")
    onrobot_device = LaunchConfiguration("onrobot_device")
    onrobot_ip = LaunchConfiguration("onrobot_ip")
    onrobot_port = LaunchConfiguration("onrobot_port")

    launch_gripper = _launch_boolean(
        LaunchConfiguration("launch_gripper").perform(context), "launch_gripper"
    )
    onrobot_type = LaunchConfiguration("onrobot_type").perform(context)
    connection_type = LaunchConfiguration("onrobot_connection_type").perform(context)
    use_tool_communication = launch_gripper and connection_type == "serial"

    upstream_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "initial_joint_controller": "forward_velocity_controller",
            "activate_joint_controller": "true",
            "launch_rviz": launch_rviz,
            "headless_mode": headless_mode,
            "use_tool_communication": str(use_tool_communication).lower(),
            "tool_parity": "2",
            "tool_baud_rate": "1000000",
            "tool_stop_bits": "1",
            "tool_rx_idle_chars": "1.5",
            "tool_tx_idle_chars": "3.5",
            "tool_device_name": onrobot_device,
            "tool_voltage": "24" if use_tool_communication else "0",
        }.items(),
    )

    if not launch_gripper:
        return [upstream_launch]

    start_delay_s = float(
        LaunchConfiguration("onrobot_start_delay_s").perform(context)
    )
    if not math.isfinite(start_delay_s) or start_delay_s < 0.0:
        raise ValueError("onrobot_start_delay_s deve ser finito e nao negativo.")

    gripper_spec = get_gripper_spec(onrobot_type)
    bringup_share = Path(get_package_share_directory("ur_cbf_bringup"))
    visual_xacro = bringup_share / "urdf" / gripper_spec.visual_description_file
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    str(visual_xacro),
                ]
            ),
            value_type=str,
        )
    }

    gripper_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("onrobot_driver"),
                    "launch",
                    "onrobot_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "onrobot_type": onrobot_type,
            "connection_type": connection_type,
            "device": onrobot_device,
            "ip_address": onrobot_ip,
            "port": onrobot_port,
            "prefix": "",
            "ns": "onrobot",
            "launch_rviz": "false",
            "launch_rsp": "false",
            "use_fake_hardware": "false",
        }.items(),
    )

    delayed_gripper_driver = TimerAction(
        period=start_delay_s,
        actions=[gripper_driver],
    )

    gripper_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="onrobot_state_publisher",
        output="screen",
        parameters=[robot_description],
        remappings=[("/joint_states", "/onrobot/visual_joint_states")],
    )

    gripper_adapter = Node(
        package="ur_cbf_bringup",
        executable="onrobot_real_adapter",
        name="onrobot_real_adapter",
        output="screen",
        parameters=[{"onrobot_type": onrobot_type}],
    )

    return [
        upstream_launch,
        gripper_state_publisher,
        gripper_adapter,
        delayed_gripper_driver,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur3e",
                choices=SUPPORTED_UR_TYPES,
                description="Modelo do manipulador Universal Robots real.",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                description="Endereco IPv4 do controlador Universal Robots.",
            ),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("headless_mode", default_value="false"),
            DeclareLaunchArgument(
                "launch_gripper",
                default_value="true",
                choices=("true", "false"),
                description="Inicia o backend real OnRobot.",
            ),
            DeclareLaunchArgument(
                "onrobot_type",
                default_value="rg2",
                choices=SUPPORTED_ONROBOT_TYPES,
                description="Modelo OnRobot conectado ao robo real.",
            ),
            DeclareLaunchArgument(
                "onrobot_connection_type",
                default_value="serial",
                choices=("serial", "tcp"),
                description="serial usa Tool I/O; tcp usa a Compute Box.",
            ),
            DeclareLaunchArgument(
                "onrobot_device",
                default_value="/tmp/ttyUR",
                description="Dispositivo Modbus serial criado pelo driver UR.",
            ),
            DeclareLaunchArgument(
                "onrobot_ip",
                default_value="192.168.1.1",
                description="Endereco da OnRobot Compute Box no modo TCP.",
            ),
            DeclareLaunchArgument(
                "onrobot_port",
                default_value="502",
                description="Porta Modbus TCP da OnRobot Compute Box.",
            ),
            DeclareLaunchArgument(
                "onrobot_start_delay_s",
                default_value="5.0",
                description="Espera pelo dispositivo Tool I/O antes do driver RG.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
