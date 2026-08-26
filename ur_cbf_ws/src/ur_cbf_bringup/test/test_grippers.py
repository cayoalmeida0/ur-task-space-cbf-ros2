import xml.etree.ElementTree as ET
import math
from pathlib import Path

import numpy as np
import pytest

from ur_cbf_bringup.grippers import (
    GRIPPER_SPECS,
    SUPPORTED_ONROBOT_TYPES,
    get_gripper_spec,
)
from ur_cbf_bringup.visualization import resolve_cbf_visibility


PACKAGE_ROOT = Path(__file__).parents[1]
REPOSITORY_ROOT = Path(__file__).parents[4]


def test_supported_models_are_explicit_and_stable():
    assert SUPPORTED_ONROBOT_TYPES == ("rg2", "rg6")


@pytest.mark.parametrize(
    ("model", "maximum_width_m", "tcp_offset_m", "maximum_effort_n"),
    [
        ("rg2", 0.110, 0.218, 40.0),
        ("rg6", 0.160, 0.268, 120.0),
    ],
)
def test_model_parameters_match_upstream_description(
    model, maximum_width_m, tcp_offset_m, maximum_effort_n
):
    spec = get_gripper_spec(model)

    assert spec.maximum_width_m == maximum_width_m
    assert spec.tcp_offset_m == tcp_offset_m
    assert spec.maximum_effort_n == maximum_effort_n
    assert (PACKAGE_ROOT / "urdf" / spec.description_file).is_file()
    assert (PACKAGE_ROOT / "urdf" / spec.visual_description_file).is_file()
    positions_closed = spec.joint_positions(0.0)
    positions_open = spec.joint_positions(maximum_width_m)
    assert positions_closed[0] == 0.0
    assert positions_open[0] == maximum_width_m
    assert len(positions_open) == len(spec.joint_names) == 7
    assert spec.simulated_joint_names == spec.joint_names[1:]
    assert len(spec.simulated_joint_positions(maximum_width_m)) == 6


def test_unknown_gripper_model_is_rejected():
    with pytest.raises(ValueError, match="Modelo OnRobot nao suportado"):
        get_gripper_spec("unknown")


@pytest.mark.parametrize("model", SUPPORTED_ONROBOT_TYPES)
def test_width_outside_physical_range_is_rejected(model):
    spec = get_gripper_spec(model)
    with pytest.raises(ValueError, match="fora do intervalo"):
        spec.joint_positions(-0.001)
    with pytest.raises(ValueError, match="fora do intervalo"):
        spec.joint_positions(spec.maximum_width_m + 0.001)


def test_every_model_has_a_unique_description():
    descriptions = [spec.description_file for spec in GRIPPER_SPECS.values()]
    assert len(descriptions) == len(set(descriptions))
    visual_descriptions = [
        spec.visual_description_file for spec in GRIPPER_SPECS.values()
    ]
    assert len(visual_descriptions) == len(set(visual_descriptions))


@pytest.mark.parametrize("model", SUPPORTED_ONROBOT_TYPES)
def test_combined_xacro_is_well_formed_and_uses_harmonic(model):
    description = PACKAGE_ROOT / "urdf" / get_gripper_spec(model).description_file
    root = ET.parse(description).getroot()
    text = description.read_text(encoding="utf-8")

    assert root.tag == "robot"
    assert "gz_ros2_control/GazeboSimSystem" in text
    assert "gz_ros2_control::GazeboSimROS2ControlPlugin" in text
    assert "gazebo_ros2_control/GazeboSystem" not in text
    assert "libgazebo_ros2_control.so" not in text
    spec = get_gripper_spec(model)
    for joint_name in spec.simulated_joint_names:
        assert f'joint="{joint_name}"' in text
    assert 'joint="finger_width"' not in text


def test_gripper_controller_contains_all_explicit_joints():
    controller_config = (
        PACKAGE_ROOT / "config" / "ur_onrobot_controllers.yaml"
    ).read_text(encoding="utf-8")

    assert "onrobot_joint_position_controller:" in controller_config
    assert "position_controllers/JointGroupPositionController" in controller_config
    for spec in GRIPPER_SPECS.values():
        for joint_name in spec.simulated_joint_names:
            assert f"- {joint_name}" in controller_config
    assert "- finger_width" not in controller_config


def test_simulation_exposes_onrobot_meshes_to_gazebo():
    launch_text = (PACKAGE_ROOT / "launch" / "simulation.launch.py").read_text(
        encoding="utf-8"
    )

    assert "AppendEnvironmentVariable" in launch_text
    assert 'name="GZ_SIM_RESOURCE_PATH"' in launch_text
    assert "str(onrobot_share.parent)" in launch_text


@pytest.mark.parametrize("model", SUPPORTED_ONROBOT_TYPES)
def test_real_visual_xacro_attaches_gripper_to_tool0(model):
    visual = PACKAGE_ROOT / "urdf" / get_gripper_spec(model).visual_description_file
    text = visual.read_text(encoding="utf-8")

    assert '<parent link="$(arg tf_prefix)tool0"/>' in text
    assert '<child link="$(arg tf_prefix)onrobot_base_link"/>' in text
    assert "gz_ros2_control" not in text
    assert "gazebo_ros2_control" not in text


def test_cbf_visual_volumes_use_project_model_and_remain_visual_only():
    volumes = PACKAGE_ROOT / "urdf" / "cbf_visual_volumes.urdf.xacro"
    root = ET.parse(volumes).getroot()
    text = volumes.read_text(encoding="utf-8")

    assert root.tag == "robot"
    assert "<collision" not in text
    assert "<inertial" not in text
    assert text.count("<xacro:cbf_sphere_visual") == 1
    assert text.count("<xacro:cbf_cylinder_visual") == 12
    assert "cbf_box_visual" not in text
    assert "cbf_capsule_visual" in text
    assert "1acb5ed637738aca4ea05945e6c065c3757bc13d" in text
    assert "<visibility_flags>" not in text
    assert "gazebo_visible" not in text
    for parent in (
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    ):
        assert f'parent="${{prefix}}{parent}"' in text

    for object_name in (
        "c0", "c11", "c12", "c13", "c21", "c22", "c23", "c31", "c32",
        "c41", "c42", "c51", "c52",
    ):
        assert f'name="${{prefix}}uaibot_{object_name}"' in text


def test_uaibot_visual_primitives_preserve_converted_origins_and_sizes():
    volumes = PACKAGE_ROOT / "urdf" / "cbf_visual_volumes.urdf.xacro"
    root = ET.parse(volumes).getroot()
    namespace = "{http://wiki.ros.org/xacro}"
    calls = {
        element.attrib["name"].replace("${prefix}uaibot_", ""): (
            element.tag.removeprefix(namespace).replace("cbf_", "").replace(
                "_visual", ""
            ),
            element.attrib,
        )
        for element in root.iter()
        if element.tag in {
            f"{namespace}cbf_sphere_visual",
            f"{namespace}cbf_cylinder_visual",
        }
    }
    expected = {
        "c0": ("cylinder", "shoulder_link", "0 0 -0.0469", "0 0 0", "0.067", "0.21"),
        "c11": ("cylinder", "upper_arm_link", "0.00185 0 0.118", "0 0 1.570796326795", "0.052", "0.13"),
        "c12": ("cylinder", "upper_arm_link", "-0.11815 0 0.12", "-1.570796326795 0 1.570796326795", "0.05", "0.2"),
        "c13": ("cylinder", "upper_arm_link", "-0.24315 0 0.118", "0 0 1.570796326795", "0.05", "0.12"),
        "c21": ("sphere", "forearm_link", "0 0 0.027", None, "0.06", None),
        "c22": ("cylinder", "forearm_link", "-0.1046 0 0.027", "-1.570796326795 0 1.570796326795", "0.04", "0.2"),
        "c23": ("cylinder", "forearm_link", "-0.2132 0 0.079025", "0 0 1.570796326795", "0.04", "0.10405"),
        "c31": ("cylinder", "wrist_1_link", "0 0 0", "0 0 3.14159265359", "0.045", "0.09"),
        "c32": ("cylinder", "wrist_1_link", "0 -0.042675 0", "-1.570796326795 0 -3.14159265359", "0.045", "0.08535"),
        "c41": ("cylinder", "wrist_2_link", "0 0 0", "0 0 -1.570796326795", "0.038", "0.09"),
        "c42": ("cylinder", "wrist_2_link", "0.0011 -0.0025 -0.004", "1.570796326795 1.570796326795 0", "0.038", "0.098"),
        "c51": ("cylinder", "wrist_3_link", "0.0011 0.004 -0.0231", "3.14159265359 0 1.570796326795", "0.038", "0.046"),
        "c52": ("cylinder", "wrist_3_link", "0.0011 -0.021 -0.0201", "1.570796326795 1.570796326795 0", "0.01", "0.028"),
    }

    assert set(calls) == set(expected)
    for name, (primitive, parent, xyz, rpy, dimension_a, dimension_b) in expected.items():
        actual_primitive, attributes = calls[name]
        assert actual_primitive == primitive
        assert attributes["parent"] == f"${{prefix}}{parent}"
        assert attributes["xyz"] == xyz
        if rpy is not None:
            assert attributes["rpy"] == rpy
        if primitive == "sphere":
            assert attributes["radius"] == dimension_a
        elif primitive == "box":
            assert attributes["size"] == dimension_a
        else:
            assert attributes["radius"] == dimension_a
            assert attributes["length"] == dimension_b


def test_uaibot_dh_to_urdf_link_maps_are_configuration_independent():
    def rotation_x(angle):
        cosine, sine = math.cos(angle), math.sin(angle)
        result = np.eye(4)
        result[:3, :3] = (
            (1.0, 0.0, 0.0),
            (0.0, cosine, -sine),
            (0.0, sine, cosine),
        )
        return result

    def rotation_z(angle):
        cosine, sine = math.cos(angle), math.sin(angle)
        result = np.eye(4)
        result[:3, :3] = (
            (cosine, -sine, 0.0),
            (sine, cosine, 0.0),
            (0.0, 0.0, 1.0),
        )
        return result

    def rotation_y(angle):
        cosine, sine = math.cos(angle), math.sin(angle)
        result = np.eye(4)
        result[:3, :3] = (
            (cosine, 0.0, sine),
            (0.0, 1.0, 0.0),
            (-sine, 0.0, cosine),
        )
        return result

    def translation(x, y, z):
        result = np.eye(4)
        result[:3, 3] = (x, y, z)
        return result

    def standard_dh(theta, d, a, alpha):
        return (
            rotation_z(theta)
            @ translation(0.0, 0.0, d)
            @ translation(a, 0.0, 0.0)
            @ rotation_x(alpha)
        )

    pi = math.pi
    d_values = (0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921)
    a_values = (0.0, -0.24355, -0.2132, 0.0, 0.0, 0.0)
    alpha_values = (pi / 2.0, 0.0, 0.0, pi / 2.0, -pi / 2.0, 0.0)
    urdf_origins = (
        translation(0.0, 0.0, 0.15185),
        rotation_x(pi / 2.0),
        translation(-0.24355, 0.0, 0.0),
        translation(-0.2132, 0.0, 0.13105),
        translation(0.0, -0.08535, 0.0) @ rotation_x(pi / 2.0),
        translation(0.0, 0.0921, 0.0)
        @ rotation_z(pi)
        @ rotation_y(pi)
        @ rotation_x(pi / 2.0),
    )
    expected_link_from_dh = (
        rotation_x(pi / 2.0),
        translation(-0.24355, 0.0, 0.0),
        translation(-0.2132, 0.0, 0.0),
        rotation_x(pi / 2.0),
        rotation_x(-pi / 2.0),
        np.eye(4),
    )

    for joint_positions in (
        (0.0, -pi / 2.0, 0.0, -pi / 2.0, -pi / 2.0, 0.0),
        (0.2, -0.7, 0.4, 1.1, -0.6, 0.3),
    ):
        uaibot_transform = np.eye(4)
        urdf_transform = np.eye(4)
        for index in range(6):
            uaibot_transform = uaibot_transform @ standard_dh(
                joint_positions[index],
                d_values[index],
                a_values[index],
                alpha_values[index],
            )
            urdf_transform = (
                urdf_transform
                @ urdf_origins[index]
                @ rotation_z(joint_positions[index])
            )
            actual_link_from_dh = (
                np.linalg.inv(urdf_transform) @ uaibot_transform
            )
            np.testing.assert_allclose(
                actual_link_from_dh,
                expected_link_from_dh[index],
                atol=5e-10,
                rtol=0.0,
            )


def test_rg2_uses_project_capsule_and_rg6_remains_without_unvalidated_tool_volume():
    rg2 = (PACKAGE_ROOT / "urdf" / "ur_rg2_gz.urdf.xacro").read_text(
        encoding="utf-8"
    )
    rg6 = (PACKAGE_ROOT / "urdf" / "ur_rg6_gz.urdf.xacro").read_text(
        encoding="utf-8"
    )

    for description in (rg2, rg6):
        assert "cbf_visual_volumes.urdf.xacro" in description
        assert 'name="show_cbf_volumes"' in description
        assert "<xacro:ur3e_cbf_visual_volumes" in description
        assert "selected_ur_type == 'ur3e'" in description
    assert "<xacro:rg2_cbf_visual_volume" in rg2
    assert "rg2_cbf_visual_volume" not in rg6


def test_rg2_capsule_matches_project_collision_dimensions():
    volumes = (PACKAGE_ROOT / "urdf" / "cbf_visual_volumes.urdf.xacro").read_text(
        encoding="utf-8"
    )

    assert 'parent="${prefix}onrobot_base_link"' in volumes
    assert 'center="0 0 0.110"' in volumes
    assert 'cap_a="0 0 0.055" cap_b="0 0 0.165"' in volumes
    assert 'radius="0.090" length="0.110"' in volumes


def test_cbf_visual_volumes_can_be_toggled_without_editing_env():
    compose = (REPOSITORY_ROOT / "docker" / "compose.yaml").read_text(
        encoding="utf-8"
    )
    makefile = (REPOSITORY_ROOT / "Makefile").read_text(encoding="utf-8")
    env_example = (REPOSITORY_ROOT / ".env.example").read_text(encoding="utf-8")
    simulation_launch = (PACKAGE_ROOT / "launch" / "simulation.launch.py").read_text(
        encoding="utf-8"
    )

    assert "CBF_VOLUMES: ${CBF_VOLUMES:-true}" in compose
    assert "CBF_VOLUMES_GAZEBO: ${CBF_VOLUMES_GAZEBO:-true}" in compose
    assert "CBF_VOLUMES ?= true" in makefile
    assert "CBF_VOLUMES_GAZEBO ?= true" in makefile
    assert 'CBF_VOLUMES="$(CBF_VOLUMES)" \\' in makefile
    assert 'CBF_VOLUMES_GAZEBO="$(CBF_VOLUMES_GAZEBO)" \\' in makefile
    assert "test-cbf-motion: init" in makefile
    assert "./scripts/test_cbf_volume_motion.sh" in makefile
    assert "CBF_VOLUMES=true" in env_example
    assert "CBF_VOLUMES_GAZEBO=true" in env_example
    assert "make sim CBF_VOLUMES=false" in env_example
    assert "make sim CBF_VOLUMES_GAZEBO=false" in env_example
    assert "rviz_description_content = _description_command(" in simulation_launch
    assert "gazebo_description_content = _description_command(" in simulation_launch
    assert '"robot_description": ParameterValue(' in simulation_launch
    assert '"-string",\n            gazebo_description_content,' in simulation_launch


@pytest.mark.parametrize(
    ("show_volumes", "show_volumes_gazebo", "expected"),
    [
        ("true", "true", ("true", "true")),
        ("true", "false", ("true", "false")),
        ("false", "true", ("false", "false")),
        ("false", "false", ("false", "false")),
    ],
)
def test_cbf_visibility_is_resolved_independently(
    show_volumes, show_volumes_gazebo, expected
):
    assert resolve_cbf_visibility(show_volumes, show_volumes_gazebo) == expected


def test_cbf_visibility_rejects_invalid_values():
    with pytest.raises(ValueError, match="show_cbf_volumes"):
        resolve_cbf_visibility("yes", "true")
    with pytest.raises(ValueError, match="show_cbf_volumes_gazebo"):
        resolve_cbf_visibility("true", "yes")


def test_visual_motion_test_is_simulation_only_and_returns_each_joint():
    script = (
        REPOSITORY_ROOT / "scripts" / "test_cbf_volume_motion.sh"
    ).read_text(encoding="utf-8")

    assert 'grep -qx "/gz_ros_control"' in script
    assert script.count("run_pulse shoulder_pan_joint") == 2
    assert script.count("run_pulse elbow_joint") == 2
    assert script.count("run_pulse wrist_1_joint") == 2
    assert "pulse_duration" in script
    assert "max_abs_velocity:=0.35" in script
    assert "state_timeout:=1.0" in script
