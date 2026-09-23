"""Cena e controlador do cenário lateral de manipulação."""

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
        [FindPackageShare("ur_cbf_control"), "launch", "manipulation_scenario_02.launch.py"]
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation),
                launch_arguments={
                    "table_x": "-0.28", "table_y": "-0.20",
                    "table_radius": "0.08", "table_height": "0.20",
                    "cube_x": "-0.28", "cube_y": "-0.20",
                    "drop_x": "-0.10", "drop_y": "0.33",
                }.items(),
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(controller)),
        ]
    )
