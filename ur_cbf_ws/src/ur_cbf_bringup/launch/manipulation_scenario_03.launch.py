"""Cena e controlador do cenário diagonal de manipulação."""

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
        [FindPackageShare("ur_cbf_control"), "launch", "manipulation_scenario_03.launch.py"]
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation),
                launch_arguments={
                    "table_x": "-0.18", "table_y": "0.26",
                    "table_radius": "0.08", "table_height": "0.25",
                    "cube_x": "-0.18", "cube_y": "0.26",
                    "drop_x": "0.16", "drop_y": "-0.27",
                    "show_cbf_volumes": "true",
                    "show_cbf_volumes_gazebo": "false",
                }.items(),
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(controller)),
        ]
    )
