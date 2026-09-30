"""Ensaio lateral com três alvos diretos e mesa mais alta."""

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
        "task_control_mode": "pose",
        "orientation_target_mode": "vertical_yaw",
        "manipulation_grasp_yaw": "0.0",
        "manipulation_home_mode": "initial",
        "manipulation_home_height": "0.30",
        "manipulation_object_frame": "base_link",
        "cube_position": "[-0.28, -0.20, 0.22]",
        "drop_position": "[-0.10, 0.33]",
        "cylinder_position": "[-0.28, -0.20]",
        "cylinder_radius": "0.08",
        "cylinder_height": "0.20",
        "cylinder_safe_distance": "0.01",
        "cube_size": "0.04",
        "cube_cbf_mode": "enforce",
        "cube_safe_distance": "0.005",
        "cube_cbf_gain": "5.0",
        "cube_cbf_contact_activation_distance": "0.06",
        "cube_cbf_excluded_pairs": (
            "['link_5_obj_4', 'link_5_obj_5', 'link_5_obj_6', 'link_5_obj_7']"
        ),
        "cube_witness_mode": "closest",
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
