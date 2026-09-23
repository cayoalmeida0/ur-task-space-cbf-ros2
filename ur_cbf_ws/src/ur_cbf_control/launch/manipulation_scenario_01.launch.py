"""Ensaio base com três alvos diretos e CBF do cilindro."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller = PathJoinSubstitution(
        [FindPackageShare("ur_cbf_control"), "launch", "cartesian_position.launch.py"]
    )
    arguments = {
        "execute_test": "true",
        "task_type": "manipulation",
        "trajectory_profile": "challenging",
        "task_control_mode": "position_vertical",
        "orientation_target_mode": "vertical",
        "manipulation_home_mode": "initial",
        "manipulation_object_frame": "base_link",
        "cube_position": "[-0.35, 0.0, 0.17]",
        "drop_position": "[-0.30, 0.18]",
        "cylinder_position": "[-0.35, 0.0]",
        "cylinder_radius": "0.08",
        "cylinder_height": "0.15",
        "cylinder_safe_distance": "0.01",
        "self_collision_cbf_mode": "enforce",
        "workspace_cbf_mode": "enforce",
        "cylinder_cbf_mode": "enforce",
        "experiment_id": "manipulation_scenario_01_base",
    }
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(controller),
                launch_arguments=arguments.items(),
            )
        ]
    )
