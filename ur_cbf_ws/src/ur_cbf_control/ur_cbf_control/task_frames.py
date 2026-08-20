"""Transformacoes rigidas dos efetuadores suportados para o frame de tarefa."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TaskFrameSpec:
    """Transformacao do ultimo frame DH/tool0 ao ponto controlado."""

    controlled_frame: str
    eef_offset_xyz: tuple[float, float, float]
    eef_offset_rpy: tuple[float, float, float]


TASK_FRAME_SPECS = {
    "rg2": TaskFrameSpec(
        controlled_frame="gripper_tcp",
        eef_offset_xyz=(0.0, 0.0, 0.218),
        eef_offset_rpy=(0.0, 0.0, -math.pi / 2.0),
    ),
    "rg6": TaskFrameSpec(
        controlled_frame="gripper_tcp",
        eef_offset_xyz=(0.0, 0.0, 0.268),
        eef_offset_rpy=(0.0, 0.0, -math.pi / 2.0),
    ),
}

SUPPORTED_ONROBOT_TYPES = tuple(TASK_FRAME_SPECS)


def get_task_frame_spec(onrobot_type: str) -> TaskFrameSpec:
    """Retorna a transformacao do TCP ou rejeita uma ferramenta desconhecida."""

    try:
        return TASK_FRAME_SPECS[onrobot_type]
    except KeyError as error:
        supported = ", ".join(SUPPORTED_ONROBOT_TYPES)
        raise ValueError(
            f"Modelo OnRobot nao suportado: {onrobot_type!r}; use {supported}."
        ) from error
