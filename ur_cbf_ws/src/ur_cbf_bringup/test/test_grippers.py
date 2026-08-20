import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from ur_cbf_bringup.grippers import (
    GRIPPER_SPECS,
    SUPPORTED_ONROBOT_TYPES,
    get_gripper_spec,
)


PACKAGE_ROOT = Path(__file__).parents[1]


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
    positions_closed = spec.joint_positions(0.0)
    positions_open = spec.joint_positions(maximum_width_m)
    assert positions_closed[0] == 0.0
    assert positions_open[0] == maximum_width_m
    assert len(positions_open) == len(spec.joint_names) == 7


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
    for joint_name in get_gripper_spec(model).joint_names:
        assert f'joint="{joint_name}"' in text


def test_gripper_controller_contains_all_explicit_joints():
    controller_config = (
        PACKAGE_ROOT / "config" / "ur_onrobot_controllers.yaml"
    ).read_text(encoding="utf-8")

    assert "onrobot_joint_position_controller:" in controller_config
    assert "position_controllers/JointGroupPositionController" in controller_config
    for spec in GRIPPER_SPECS.values():
        for joint_name in spec.joint_names:
            assert f"- {joint_name}" in controller_config
