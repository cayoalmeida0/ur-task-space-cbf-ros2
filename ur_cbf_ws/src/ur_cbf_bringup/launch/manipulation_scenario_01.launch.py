"""Cena e controlador do cenário base de manipulação."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulation = PathJoinSubstitution(
        [FindPackageShare("ur_cbf_bringup"), "launch", "simulation.launch.py"]
    )
    controller = PathJoinSubstitution(
        [FindPackageShare("ur_cbf_control"), "launch", "manipulation_scenario_01.launch.py"]
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation),
                launch_arguments={
                    "table_x": "-0.35", "table_y": "0.0",
                    "table_radius": "0.08", "table_height": "0.15",
                    "cube_x": "-0.35", "cube_y": "0.0",
                    "drop_x": "-0.30", "drop_y": "0.18",
                    "show_cbf_volumes": "true",
                    "show_cbf_volumes_gazebo": "false",
                }.items(),
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(controller)),
        ]
    )
