# Copyright (c) 2021 Stogl Robotics Consulting UG (haftungsbeschraenkt)
# Copyright (c) 2026 Cayo Sousa
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the names of the copyright holders nor the names of their
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DAMAGES ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE.
#
# A estrutura de inicializacao deriva de ur_sim_control.launch.py, fixado em
# UR_SIMULATION_GZ_COMMIT no Dockerfile. Esta adaptacao gera descricoes distintas
# para robot_state_publisher/RViz e para a entidade criada no Gazebo.

from pathlib import Path
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ur_cbf_bringup.grippers import SUPPORTED_ONROBOT_TYPES, get_gripper_spec
from ur_cbf_bringup.models import SUPPORTED_UR_TYPES
from ur_cbf_bringup.visualization import resolve_cbf_visibility


def _description_command(
    description_file,
    controllers_file,
    ur_type,
    tf_prefix,
    safety_limits,
    safety_pos_margin,
    safety_k_position,
    show_cbf_volumes,
):
    return Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            str(description_file),
            " safety_limits:=",
            safety_limits,
            " safety_pos_margin:=",
            safety_pos_margin,
            " safety_k_position:=",
            safety_k_position,
            " name:=ur",
            " ur_type:=",
            ur_type,
            " tf_prefix:=",
            tf_prefix,
            " simulation_controllers:=",
            str(controllers_file),
            " show_cbf_volumes:=",
            show_cbf_volumes,
        ]
    )


def _resolve_world_file(context, source_file, mappings):
    """Processa uma cena Xacro para um SDF temporario aceito pelo Gazebo."""

    source = Path(source_file)
    if source.suffix != ".xacro":
        return str(source)
    try:
        import xacro

        document = xacro.process_file(str(source), mappings=mappings)
    except Exception as error:
        raise RuntimeError(
            f"Falha ao processar a cena Xacro {source}: {error}"
        ) from error
    generated = Path(tempfile.gettempdir()) / (
        f"ur_cbf_world_{os.getpid()}_{abs(hash(str(source)))}.sdf"
    )
    generated.write_text(document.toxml(), encoding="utf-8")
    return str(generated)


def _manipulation_world_mappings(context):
    """Lê dimensões da cena e calcula as poses derivadas para o Xacro."""

    def value(name):
        return float(LaunchConfiguration(name).perform(context))

    table_height = value("table_height")
    cube_size = value("cube_size")
    box_size_x = value("box_size_x")
    box_size_y = value("box_size_y")
    box_height = value("box_height")
    wall = value("box_wall_thickness")
    return {
        "table_x": str(value("table_x")),
        "table_y": str(value("table_y")),
        "table_radius": str(value("table_radius")),
        "table_height": str(table_height),
        "cube_x": str(value("cube_x")),
        "cube_y": str(value("cube_y")),
        "cube_size": str(cube_size),
        "cube_z": str(table_height + cube_size / 2.0),
        "table_z": str(table_height / 2.0),
        "drop_x": str(value("drop_x")),
        "drop_y": str(value("drop_y")),
        "box_size_x": str(box_size_x),
        "box_size_y": str(box_size_y),
        "box_height": str(box_height),
        "box_wall_thickness": str(wall),
        "box_base_z": str(wall / 2.0),
        "box_wall_z": str(box_height / 2.0),
        "box_wall_x": str(box_size_x / 2.0 - wall / 2.0),
        "box_wall_y": str(box_size_y / 2.0 - wall / 2.0),
    }


def launch_setup(context):
    ur_type = LaunchConfiguration("ur_type")
    tf_prefix = LaunchConfiguration("tf_prefix")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")

    rviz_cbf_value, gazebo_cbf_value = resolve_cbf_visibility(
        LaunchConfiguration("show_cbf_volumes").perform(context),
        LaunchConfiguration("show_cbf_volumes_gazebo").perform(context),
    )

    onrobot_type = LaunchConfiguration("onrobot_type").perform(context)
    resolved_world_file = _resolve_world_file(
        context,
        world_file.perform(context),
        _manipulation_world_mappings(context),
    )
    gripper_spec = get_gripper_spec(onrobot_type)
    bringup_share = Path(get_package_share_directory("ur_cbf_bringup"))
    onrobot_share = Path(get_package_share_directory("onrobot_description"))
    description_file = bringup_share / "urdf" / gripper_spec.description_file
    controllers_file = bringup_share / "config" / "ur_onrobot_controllers.yaml"

    rviz_description_content = _description_command(
        description_file,
        controllers_file,
        ur_type,
        tf_prefix,
        safety_limits,
        safety_pos_margin,
        safety_k_position,
        rviz_cbf_value,
    )
    gazebo_description_content = _description_command(
        description_file,
        controllers_file,
        ur_type,
        tf_prefix,
        safety_limits,
        safety_pos_margin,
        safety_k_position,
        gazebo_cbf_value,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {
                "robot_description": ParameterValue(
                    rviz_description_content, value_type=str
                )
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    delayed_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz],
        ),
        condition=IfCondition(launch_rviz),
    )

    activate_controller = LaunchConfiguration("activate_joint_controller")
    initial_controller = LaunchConfiguration("initial_joint_controller")
    controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_controller, "-c", "/controller_manager"],
        condition=IfCondition(activate_controller),
    )
    controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(activate_controller),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
                "gz_args": IfElseSubstitution(
                gazebo_gui,
                if_value=[" -r -v 4 ", resolved_world_file],
                else_value=[" -s -r -v 4 ", resolved_world_file],
            )
        }.items(),
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            gazebo_description_content,
            "-name",
            "ur",
            "-allow_renaming",
            "true",
        ],
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
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

    gazebo_onrobot_resources = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=str(onrobot_share.parent),
    )

    return [
        gazebo_onrobot_resources,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        delayed_rviz,
        controller_spawner_stopped,
        controller_spawner_started,
        gazebo,
        spawn_robot,
        clock_bridge,
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
            DeclareLaunchArgument("tf_prefix", default_value='""'),
            DeclareLaunchArgument("safety_limits", default_value="false"),
            DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
            DeclareLaunchArgument("safety_k_position", default_value="20"),
            DeclareLaunchArgument(
                "activate_joint_controller", default_value="true"
            ),
            DeclareLaunchArgument(
                "initial_joint_controller",
                default_value="forward_velocity_controller",
            ),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ur_cbf_bringup"), "rviz", "ur_cbf.rviz"]
                ),
            ),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument(
                "world_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("ur_cbf_bringup"),
                        "worlds",
                        "manipulation.sdf.xacro",
                    ]
                ),
            ),
            DeclareLaunchArgument("table_x", default_value="-0.25"),
            DeclareLaunchArgument("table_y", default_value="0.0"),
            DeclareLaunchArgument("table_radius", default_value="0.08"),
            DeclareLaunchArgument("table_height", default_value="0.30"),
            DeclareLaunchArgument("cube_x", default_value="-0.25"),
            DeclareLaunchArgument("cube_y", default_value="0.0"),
            DeclareLaunchArgument("cube_size", default_value="0.04"),
            DeclareLaunchArgument("drop_x", default_value="0.20"),
            DeclareLaunchArgument("drop_y", default_value="0.0"),
            DeclareLaunchArgument("box_size_x", default_value="0.08"),
            DeclareLaunchArgument("box_size_y", default_value="0.08"),
            DeclareLaunchArgument("box_height", default_value="0.04"),
            DeclareLaunchArgument("box_wall_thickness", default_value="0.005"),
            DeclareLaunchArgument(
                "show_cbf_volumes",
                default_value=EnvironmentVariable(
                    "CBF_VOLUMES", default_value="true"
                ),
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "show_cbf_volumes_gazebo",
                default_value=EnvironmentVariable(
                    "CBF_VOLUMES_GAZEBO", default_value="false"
                ),
                choices=["true", "false"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
