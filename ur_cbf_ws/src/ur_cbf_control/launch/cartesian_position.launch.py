from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ur_cbf_control.task_frames import SUPPORTED_ONROBOT_TYPES
from ur_cbf_control.task_frames import get_task_frame_spec


def launch_setup(context):
    onrobot_type = LaunchConfiguration("onrobot_type").perform(context)
    get_task_frame_spec(onrobot_type)
    config_file = PathJoinSubstitution(
        [FindPackageShare("ur_cbf_control"), "config", "cartesian_position.yaml"]
    )

    return [
        Node(
            package="ur_cbf_control",
            executable="cartesian_position_test",
            name="cartesian_position_test",
            output="screen",
            parameters=[
                config_file,
                {
                    "execute_test": ParameterValue(
                        LaunchConfiguration("execute_test"),
                        value_type=bool,
                    ),
                    "ur_type": LaunchConfiguration("ur_type"),
                    "onrobot_type": onrobot_type,
                    "controller_mode": LaunchConfiguration("controller_mode"),
                    "max_control_duration": ParameterValue(
                        LaunchConfiguration("max_control_duration"),
                        value_type=float,
                    ),
                    "max_wall_control_duration": ParameterValue(
                        LaunchConfiguration("max_wall_control_duration"),
                        value_type=float,
                    ),
                    "experiment_id": LaunchConfiguration("experiment_id"),
                    "result_directory": LaunchConfiguration("result_directory"),
                },
            ],
        )
    ]


def generate_launch_description():
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
            DeclareLaunchArgument(
                "onrobot_type",
                default_value=EnvironmentVariable(
                    "ONROBOT_TYPE",
                    default_value="rg2",
                ),
                choices=SUPPORTED_ONROBOT_TYPES,
                description="Gripper que define o frame cartesiano controlado.",
            ),
            DeclareLaunchArgument(
                "controller_mode",
                default_value="qp",
                choices=["dls", "qp"],
                description="Resolvedor nominal usado no ensaio comparativo.",
            ),
            DeclareLaunchArgument(
                "max_control_duration",
                default_value="30.0",
                description="Limite de convergencia em segundos simulados.",
            ),
            DeclareLaunchArgument(
                "max_wall_control_duration",
                default_value="180.0",
                description="Limite absoluto de seguranca em segundos reais.",
            ),
            DeclareLaunchArgument(
                "experiment_id",
                default_value="cartesian_position_ur3e_001",
                description="Identificador registrado no resultado experimental.",
            ),
            DeclareLaunchArgument(
                "result_directory",
                default_value="/workspace/results",
                description="Diretorio dos arquivos JSON experimentais.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
