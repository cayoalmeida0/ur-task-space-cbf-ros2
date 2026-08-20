from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ur_cbf_bringup.grippers import SUPPORTED_ONROBOT_TYPES, get_gripper_spec
from ur_cbf_bringup.models import SUPPORTED_UR_TYPES


def launch_setup(context):
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")
    onrobot_type = LaunchConfiguration("onrobot_type").perform(context)

    gripper_spec = get_gripper_spec(onrobot_type)
    bringup_share = Path(get_package_share_directory("ur_cbf_bringup"))
    onrobot_share = Path(get_package_share_directory("onrobot_description"))
    description_file = bringup_share / "urdf" / gripper_spec.description_file
    controllers_file = bringup_share / "config" / "ur_onrobot_controllers.yaml"

    gazebo_onrobot_resources = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=str(onrobot_share.parent),
    )

    upstream_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "initial_joint_controller": "forward_velocity_controller",
            "activate_joint_controller": "true",
            "launch_rviz": launch_rviz,
            "gazebo_gui": gazebo_gui,
            "world_file": world_file,
            "description_file": str(description_file),
            "controllers_file": str(controllers_file),
        }.items(),
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["onrobot_joint_position_controller", "-c", "/controller_manager"],
        output="screen",
    )

    gripper_width_adapter = Node(
        package="ur_cbf_bringup",
        executable="onrobot_width_adapter",
        name="onrobot_width_adapter",
        output="screen",
        parameters=[{"use_sim_time": True, "onrobot_type": onrobot_type}],
    )

    return [
        gazebo_onrobot_resources,
        upstream_launch,
        gripper_controller_spawner,
        gripper_width_adapter,
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur3e",
                choices=SUPPORTED_UR_TYPES,
                description="Modelo Universal Robots usado na simulacao.",
            ),
            DeclareLaunchArgument(
                "onrobot_type",
                default_value="rg2",
                choices=SUPPORTED_ONROBOT_TYPES,
                description="Modelo da gripper OnRobot acoplada ao tool0.",
            ),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("world_file", default_value="empty.sdf"),
            OpaqueFunction(function=launch_setup),
        ]
    )
