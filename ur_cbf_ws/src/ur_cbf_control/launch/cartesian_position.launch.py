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
    controller_mode = LaunchConfiguration("controller_mode")
    max_control_duration = LaunchConfiguration("max_control_duration")
    max_wall_control_duration = LaunchConfiguration(
        "max_wall_control_duration"
    )
    experiment_id = LaunchConfiguration("experiment_id")
    result_directory = LaunchConfiguration("result_directory")
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
                        "controller_mode": controller_mode,
                        "max_control_duration": ParameterValue(
                            max_control_duration,
                            value_type=float,
                        ),
                        "max_wall_control_duration": ParameterValue(
                            max_wall_control_duration,
                            value_type=float,
                        ),
                        "experiment_id": experiment_id,
                        "result_directory": result_directory,
                    },
                ],
            ),
        ]
    )
