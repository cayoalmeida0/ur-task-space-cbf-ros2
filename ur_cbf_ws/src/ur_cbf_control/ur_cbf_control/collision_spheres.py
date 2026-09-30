"""Esferas envolventes para os volumes de colisao do modelo UAIbot."""

from dataclasses import dataclass
import math
from typing import Any, Sequence

import numpy as np


class RobotCollisionSpheresError(ValueError):
    """Indica geometria de colisao invalida para witnesses visuais."""


@dataclass(frozen=True)
class RobotCollisionSpheres:
    """Centros, raios e identificadores das primitivas no frame base."""

    centers: np.ndarray
    radii: np.ndarray
    labels: tuple[str, ...]


def primitive_bounding_radius(primitive: Any) -> float:
    """Retorna o raio da esfera que envolve Ball, Cylinder ou Box."""

    primitive_type = type(primitive).__name__.lower()
    if primitive_type == "ball":
        radius = float(primitive.radius)
    elif primitive_type == "cylinder":
        radius = math.hypot(
            float(primitive.radius),
            0.5 * float(primitive.height),
        )
    elif primitive_type == "box":
        radius = 0.5 * math.sqrt(
            float(primitive.width) ** 2
            + float(primitive.depth) ** 2
            + float(primitive.height) ** 2
        )
    else:
        raise RobotCollisionSpheresError(
            f"Primitiva de colisao nao suportada: {type(primitive).__name__}."
        )
    if not math.isfinite(radius) or radius <= 0.0:
        raise RobotCollisionSpheresError(
            f"Raio envolvente invalido para {type(primitive).__name__}."
        )
    return radius


def evaluate_robot_collision_spheres(
    robot: Any,
    configuration: Sequence[float],
) -> RobotCollisionSpheres:
    """Transforma todos os proxies de colisao para o frame DH base."""

    positions = np.asarray(configuration, dtype=float).reshape(-1)
    links = getattr(robot, "links", None)
    if (
        links is None
        or positions.size != len(links)
        or not np.all(np.isfinite(positions))
    ):
        raise RobotCollisionSpheresError("Configuracao articular invalida.")

    try:
        _, dh_transforms = robot.jac_geo(positions, "dh", mode="python")
    except Exception as error:
        raise RobotCollisionSpheresError(
            f"Falha ao transformar volumes de colisao: {error}"
        ) from error
    if len(dh_transforms) != len(links):
        raise RobotCollisionSpheresError(
            "Numero de frames DH difere do numero de elos."
        )

    centers: list[np.ndarray] = []
    radii: list[float] = []
    labels: list[str] = []
    for link_index, link in enumerate(links):
        dh = np.asarray(dh_transforms[link_index], dtype=float)
        if dh.shape != (4, 4) or not np.all(np.isfinite(dh)):
            raise RobotCollisionSpheresError("Frame DH invalido.")
        for object_index, item in enumerate(link.col_objects):
            if len(item) != 2:
                raise RobotCollisionSpheresError(
                    f"Anexo invalido em link_{link_index}_obj_{object_index}."
                )
            primitive, attached = item
            attached_transform = np.asarray(attached, dtype=float)
            if (
                attached_transform.shape != (4, 4)
                or not np.all(np.isfinite(attached_transform))
            ):
                raise RobotCollisionSpheresError(
                    f"Transformacao invalida em link_{link_index}_obj_{object_index}."
                )
            world = dh @ attached_transform
            center = world[:3, 3]
            if not np.all(np.isfinite(center)):
                raise RobotCollisionSpheresError(
                    f"Centro invalido em link_{link_index}_obj_{object_index}."
                )
            centers.append(center.copy())
            radii.append(primitive_bounding_radius(primitive))
            labels.append(f"link_{link_index}_obj_{object_index}")

    if not centers:
        raise RobotCollisionSpheresError(
            "O modelo nao possui primitivas de colisao."
        )
    return RobotCollisionSpheres(
        centers=np.vstack(centers),
        radii=np.asarray(radii, dtype=float),
        labels=tuple(labels),
    )
