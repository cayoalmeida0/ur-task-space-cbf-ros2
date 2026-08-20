import subprocess
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
REPOSITORY_ROOT = Path(__file__).parents[4]


def test_real_launch_uses_separate_onrobot_controller_manager():
    launch_text = (PACKAGE_ROOT / "launch" / "real.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'FindPackageShare("onrobot_driver")' in launch_text
    assert '"ns": "onrobot"' in launch_text
    assert '"use_tool_communication"' in launch_text
    assert '"tool_baud_rate": "1000000"' in launch_text
    assert '"tool_parity": "2"' in launch_text
    assert '"tool_voltage": "24"' in launch_text
    assert "TimerAction" in launch_text


def test_real_adapter_preserves_common_command_topic():
    adapter_text = (
        PACKAGE_ROOT / "ur_cbf_bringup" / "onrobot_real_adapter.py"
    ).read_text(encoding="utf-8")

    assert '"/finger_width_controller/commands"' in adapter_text
    assert '"/onrobot/finger_width_controller/commands"' in adapter_text
    assert '"/onrobot/joint_states"' in adapter_text
    assert '"/onrobot/visual_joint_states"' in adapter_text


def test_docker_pins_onrobot_driver_revision():
    dockerfile = (REPOSITORY_ROOT / "docker" / "Dockerfile").read_text(
        encoding="utf-8"
    )

    assert (
        "ONROBOT_DRIVER_COMMIT="
        "b99abaccfbbe90f2096feff833f4c0849757a587" in dockerfile
    )
    assert "libnet1-dev" in dockerfile
    assert "git submodule update --init --recursive" in dockerfile


def test_environment_migration_preserves_local_values(tmp_path):
    target = tmp_path / ".env"
    target.write_text(
        "IMAGE_TAG=0.1.9\n"
        "ONROBOT_TYPE=rg6\n"
        "ROBOT_IP=10.0.0.20\n"
        "ROS_DOMAIN_ID=77\n",
        encoding="utf-8",
    )

    subprocess.run(
        [
            str(REPOSITORY_ROOT / "scripts" / "sync_env.sh"),
            str(target),
            str(REPOSITORY_ROOT / ".env.example"),
        ],
        check=True,
    )
    values = dict(
        line.split("=", 1)
        for line in target.read_text(encoding="utf-8").splitlines()
        if line and not line.startswith("#")
    )

    assert values["IMAGE_TAG"] == "0.2.0"
    assert values["ONROBOT_TYPE"] == "rg2"
    assert values["ROBOT_IP"] == "10.0.0.20"
    assert values["ROS_DOMAIN_ID"] == "77"
    assert values["ONROBOT_CONNECTION_TYPE"] == "serial"
