import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from ur_cbf_bringup.grippers import (
    GRIPPER_SPECS,
    SUPPORTED_ONROBOT_TYPES,
    get_gripper_spec,
)


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


def test_cbf_visual_volumes_are_visual_only_and_cover_ur3e_rg2():
    volumes = PACKAGE_ROOT / "urdf" / "cbf_visual_volumes.urdf.xacro"
    root = ET.parse(volumes).getroot()
    text = volumes.read_text(encoding="utf-8")

    assert root.tag == "robot"
    assert "<collision" not in text
    assert "<inertial" not in text
    assert text.count("<xacro:cbf_sphere_visual") == 6
    assert text.count("<xacro:cbf_capsule_visual") == 3
    for parent in (
        "base_link",
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
        "onrobot_base_link",
    ):
        assert f'parent="${{prefix}}{parent}"' in text

    assert 'name="${prefix}rg2"' in text
    assert 'radius="0.090" length="0.110"' in text
    assert "rg2_finger" not in text


def test_rg2_uses_one_gripper_capsule_and_rg6_does_not_reuse_it():
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
    assert rg2.count("<xacro:rg2_cbf_visual_volume") == 1
    assert "<xacro:rg2_cbf_visual_volume" not in rg6


def test_cbf_visual_volumes_can_be_toggled_without_editing_env():
    compose = (REPOSITORY_ROOT / "docker" / "compose.yaml").read_text(
        encoding="utf-8"
    )
    makefile = (REPOSITORY_ROOT / "Makefile").read_text(encoding="utf-8")
    env_example = (REPOSITORY_ROOT / ".env.example").read_text(encoding="utf-8")

    assert "CBF_VOLUMES: ${CBF_VOLUMES:-true}" in compose
    assert "CBF_VOLUMES ?= true" in makefile
    assert 'CBF_VOLUMES="$(CBF_VOLUMES)" $(COMPOSE)' in makefile
    assert "CBF_VOLUMES=true" in env_example
    assert "make sim CBF_VOLUMES=false" in env_example
