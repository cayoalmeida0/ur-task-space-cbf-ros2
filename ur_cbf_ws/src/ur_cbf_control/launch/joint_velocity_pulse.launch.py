from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    target_joint = LaunchConfiguration("target_joint")
    execute_test = LaunchConfiguration("execute_test")
    config_file = PathJoinSubstitution(
        [FindPackageShare("ur_cbf_control"), "config", "joint_velocity_pulse.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "target_joint",
                default_value="",
                description="Junta que recebera o pulso de validacao.",
            ),
            DeclareLaunchArgument(
                "execute_test",
                default_value="false",
                choices=["true", "false"],
                description="Armamento explicito do ensaio de movimento.",
            ),
            Node(
                package="ur_cbf_control",
                executable="joint_velocity_pulse_test",
                name="joint_velocity_pulse_test",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "target_joint": target_joint,
                        "execute_test": ParameterValue(execute_test, value_type=bool),
                    },
                ],
            ),
        ]
    )
