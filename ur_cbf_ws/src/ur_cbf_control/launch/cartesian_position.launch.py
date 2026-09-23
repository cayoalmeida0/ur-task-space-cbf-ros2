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
                    "task_type": LaunchConfiguration("task_type"),
                    "controller_mode": LaunchConfiguration("controller_mode"),
                    "task_control_mode": LaunchConfiguration("task_control_mode"),
                    "orientation_target_mode": LaunchConfiguration(
                        "orientation_target_mode"
                    ),
                    "manipulation_object_frame": LaunchConfiguration(
                        "manipulation_object_frame"
                    ),
                    # Sem value_type explicito, o launch_ros interpreta as
                    # listas YAML como arrays de parametros ROS.
                    "cube_position": LaunchConfiguration("cube_position"),
                    "drop_position": LaunchConfiguration("drop_position"),
                    "cylinder_position": LaunchConfiguration(
                        "cylinder_position"
                    ),
                    "cylinder_radius": ParameterValue(
                        LaunchConfiguration("cylinder_radius"), value_type=float
                    ),
                    "cylinder_height": ParameterValue(
                        LaunchConfiguration("cylinder_height"), value_type=float
                    ),
                    "manipulation_approach_height": ParameterValue(
                        LaunchConfiguration("manipulation_approach_height"),
                        value_type=float,
                    ),
                    "manipulation_lift_height": ParameterValue(
                        LaunchConfiguration("manipulation_lift_height"),
                        value_type=float,
                    ),
                    "trajectory_profile": LaunchConfiguration(
                        "trajectory_profile"
                    ),
                    "self_collision_cbf_mode": LaunchConfiguration(
                        "self_collision_cbf_mode"
                    ),
                    "self_collision_witness_mode": LaunchConfiguration(
                        "self_collision_witness_mode"
                    ),
                    "workspace_cbf_mode": LaunchConfiguration(
                        "workspace_cbf_mode"
                    ),
                    "cylinder_cbf_mode": LaunchConfiguration(
                        "cylinder_cbf_mode"
                    ),
                    "cylinder_safe_distance": ParameterValue(
                        LaunchConfiguration("cylinder_safe_distance"),
                        value_type=float,
                    ),
                    "cylinder_cbf_gain": ParameterValue(
                        LaunchConfiguration("cylinder_cbf_gain"),
                        value_type=float,
                    ),
                    "cylinder_witness_mode": LaunchConfiguration(
                        "cylinder_witness_mode"
                    ),
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
                "trajectory_profile",
                default_value="simple",
                choices=["simple", "complex", "challenging"],
                description="Seleciona regulacao simples ou trajetoria multi-waypoint.",
            ),
            DeclareLaunchArgument(
                "task_type",
                default_value="cartesian",
                choices=["cartesian", "manipulation"],
                description="Ensaio cartesiano simples ou pick-and-place fisico.",
            ),
            DeclareLaunchArgument(
                "controller_mode",
                default_value="qp",
                choices=["dls", "qp"],
                description="Resolvedor nominal usado no ensaio comparativo.",
            ),
            DeclareLaunchArgument(
                "task_control_mode",
                default_value="pose",
                choices=["position", "pose"],
                description=(
                    "position usa Jv; pose usa o Jacobiano geometrico completo [v; omega]."
                ),
            ),
            DeclareLaunchArgument(
                "orientation_target_mode",
                default_value="initial",
                choices=["initial", "vertical", "rpy"],
                description="Alvo angular inicial, vertical ou RPY configurado.",
            ),
            DeclareLaunchArgument(
                "manipulation_object_frame",
                default_value="base_link",
                choices=["base_link", "base"],
                description=(
                    "Frame das posicoes do cubo e da caixa na cena de manipulacao."
                ),
            ),
            DeclareLaunchArgument(
                "cube_position",
                default_value="[-0.35, 0.0, 0.17]",
                description=(
                    "Posicao do centro do cubo no frame manipulation_object_frame."
                ),
            ),
            DeclareLaunchArgument(
                "drop_position",
                default_value="[-0.30, 0.18]",
                description=(
                    "Posicao x,y da caixa no frame manipulation_object_frame."
                ),
            ),
            DeclareLaunchArgument(
                "cylinder_position",
                default_value="[-0.35, 0.0]",
                description="Centro x,y do cilindro no frame da cena.",
            ),
            DeclareLaunchArgument(
                "cylinder_radius",
                default_value="0.08",
                description="Raio do cilindro usado pela CBF externa.",
            ),
            DeclareLaunchArgument(
                "cylinder_height",
                default_value="0.15",
                description="Altura do cilindro usada pela CBF externa.",
            ),
            DeclareLaunchArgument(
                "manipulation_approach_height",
                default_value="0.05",
                description="Altura acima do centro do cubo antes da aproximacao.",
            ),
            DeclareLaunchArgument(
                "manipulation_lift_height",
                default_value="0.05",
                description="Altura acima do centro do cubo apos a pega.",
            ),
            DeclareLaunchArgument(
                "self_collision_cbf_mode",
                default_value="off",
                choices=["off", "monitor", "enforce"],
                description=(
                    "Desliga, monitora ou impoe a CBF de autocolisao no QP."
                ),
            ),
            DeclareLaunchArgument(
                "self_collision_witness_mode",
                default_value="closest",
                choices=["off", "closest", "all"],
                description="Seleciona os pares exibidos como witness points.",
            ),
            DeclareLaunchArgument(
                "workspace_cbf_mode",
                default_value="monitor",
                choices=["off", "monitor", "enforce"],
                description=(
                    "Desliga, monitora ou impoe a CBF do envelope cartesiano."
                ),
            ),
            DeclareLaunchArgument(
                "cylinder_cbf_mode",
                default_value="off",
                choices=["off", "monitor", "enforce"],
                description="CBF para o cilindro da cena.",
            ),
            DeclareLaunchArgument(
                "cylinder_safe_distance",
                default_value="0.03",
                description="Margem geométrica para o cilindro.",
            ),
            DeclareLaunchArgument(
                "cylinder_cbf_gain",
                default_value="5.0",
                description="Ganho da CBF do cilindro.",
            ),
            DeclareLaunchArgument(
                "cylinder_witness_mode",
                default_value="closest",
                choices=["off", "closest", "all"],
                description="Witness points do cilindro.",
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
