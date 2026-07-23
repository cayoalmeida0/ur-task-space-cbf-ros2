from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from ur_cbf_bringup.models import SUPPORTED_UR_TYPES


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless_mode = LaunchConfiguration("headless_mode")

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
        }.items(),
    )

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
            upstream_launch,
        ]
    )
