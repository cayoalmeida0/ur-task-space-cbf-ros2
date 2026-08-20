"""Parametros invariantes dos modelos OnRobot suportados."""

from dataclasses import dataclass


@dataclass(frozen=True)
class GripperSpec:
    description_file: str
    visual_description_file: str
    maximum_width_m: float
    tcp_offset_m: float
    maximum_effort_n: float
    finger_angle_offset_rad: float
    finger_angle_per_width_rad_m: float

    @property
    def joint_names(self) -> tuple[str, ...]:
        return (
            "finger_width",
            "finger_joint",
            "left_inner_knuckle_joint",
            "left_inner_finger_joint",
            "right_outer_knuckle_joint",
            "right_inner_knuckle_joint",
            "right_inner_finger_joint",
        )

    def joint_positions(self, width_m: float) -> tuple[float, ...]:
        """Converte a largura total para as sete juntas do mecanismo paralelo."""

        if not 0.0 <= width_m <= self.maximum_width_m:
            raise ValueError(
                f"Largura {width_m} fora do intervalo "
                f"[0, {self.maximum_width_m}] m."
            )
        angle = (
            self.finger_angle_offset_rad
            + self.finger_angle_per_width_rad_m * width_m
        )
        return (width_m, angle, -angle, angle, -angle, -angle, angle)


GRIPPER_SPECS = {
    "rg2": GripperSpec(
        description_file="ur_rg2_gz.urdf.xacro",
        visual_description_file="onrobot_rg2_visual.urdf.xacro",
        maximum_width_m=0.110,
        tcp_offset_m=0.218,
        maximum_effort_n=40.0,
        finger_angle_offset_rad=0.785398,
        finger_angle_per_width_rad_m=0.85 * ((-0.558505 - 0.785398) / 0.110),
    ),
    "rg6": GripperSpec(
        description_file="ur_rg6_gz.urdf.xacro",
        visual_description_file="onrobot_rg6_visual.urdf.xacro",
        maximum_width_m=0.160,
        tcp_offset_m=0.268,
        maximum_effort_n=120.0,
        finger_angle_offset_rad=0.628319,
        finger_angle_per_width_rad_m=0.88 * ((-0.628319 - 0.628319) / 0.160),
    ),
}

SUPPORTED_ONROBOT_TYPES = tuple(GRIPPER_SPECS)


def get_gripper_spec(onrobot_type: str) -> GripperSpec:
    """Retorna a especificacao ou rejeita um modelo nao parametrizado."""

    try:
        return GRIPPER_SPECS[onrobot_type]
    except KeyError as error:
        supported = ", ".join(SUPPORTED_ONROBOT_TYPES)
        raise ValueError(
            f"Modelo OnRobot nao suportado: {onrobot_type!r}; use {supported}."
        ) from error
