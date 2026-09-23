"""Ensaio lateral com cilindro mais alto e caixa deslocada."""

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
        "task_control_mode": "position",
        "manipulation_object_frame": "base_link",
        "cube_position": "[-0.28, -0.20, 0.22]",
        "drop_position": "[-0.10, 0.33]",
        "cylinder_position": "[-0.28, -0.20]",
        "cylinder_radius": "0.08",
        "cylinder_height": "0.20",
        "cylinder_safe_distance": "0.01",
        "self_collision_cbf_mode": "enforce",
        "workspace_cbf_mode": "enforce",
        "cylinder_cbf_mode": "enforce",
        "experiment_id": "manipulation_scenario_02_lateral",
    }
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(controller),
                launch_arguments=arguments.items(),
            )
        ]
    )
