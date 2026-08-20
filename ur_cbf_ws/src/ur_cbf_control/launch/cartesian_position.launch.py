from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    execute_test = LaunchConfiguration("execute_test")
    ur_type = LaunchConfiguration("ur_type")
    config_file = PathJoinSubstitution(
        [FindPackageShare("ur_cbf_control"), "config", "cartesian_position.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "execute_test",
                default_value="false",
                choices=["true", "false"],
                description="Armamento explicito da regulacao cartesiana.",
            ),
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur3e",
                description="Modelo que deve coincidir com a simulacao ativa.",
            ),
            Node(
                package="ur_cbf_control",
                executable="cartesian_position_test",
                name="cartesian_position_test",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "execute_test": ParameterValue(
                            execute_test,
                            value_type=bool,
                        ),
                        "ur_type": ur_type,
                    },
                ],
            ),
        ]
    )
