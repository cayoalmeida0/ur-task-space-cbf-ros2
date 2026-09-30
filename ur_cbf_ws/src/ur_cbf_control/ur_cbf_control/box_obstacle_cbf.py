"""CBF diferencial para um obstáculo prismático axis-aligned.

O cubo da tarefa é descrito como uma caixa fixa durante a aproximação no frame
DH ``base``. Cada
primitiva de colisao do robo e conservadoramente envolvida por uma esfera; a
distancia assinada entre o centro dessa esfera e a caixa, subtraindo o raio da
esfera, fornece a restricao diferencial
``J_d qdot >= -gamma (d - d_safe)``.

Os quatro volumes móveis das falanges e pontas da RG2 podem ser excluidos por
configuracao somente na janela final, pois o contato lateral dessas partes com
o cubo e intencional na pega. A palma, os punhos e o braço continuam
protegidos.
"""

from dataclasses import dataclass
import math
import time
from typing import Any, Sequence

import numpy as np


class BoxObstacleCbfError(RuntimeError):
    """Indica geometria ou parametros invalidos do obstaculo em caixa."""


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=float).reshape(-1)
    return np.array(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)))


@dataclass(frozen=True)
class BoxObstacleDistances:
    """Distancias assinadas entre primitivas do robo e a caixa."""

    distances: np.ndarray
    jacobian: np.ndarray
    pair_labels: tuple[str, ...]
    first_witness_points: np.ndarray
    second_witness_points: np.ndarray
    geometry_source: str
    evaluation_time: float = 0.0

    @property
    def count(self) -> int:
        return int(self.distances.size)

    @property
    def minimum_distance(self) -> float:
        return float(np.min(self.distances)) if self.count else math.inf


@dataclass(frozen=True)
class BoxObstacleCbfConstraints:
    """Restricoes ``matrix @ qdot >= lower_bound`` da caixa."""

    matrix: np.ndarray
    lower_bound: np.ndarray
    barrier_values: np.ndarray
    distances: np.ndarray
    pair_labels: tuple[str, ...]
    first_witness_points: np.ndarray
    second_witness_points: np.ndarray
    safe_distance: float
    gain: float
    geometry_source: str
    evaluation_time: float

    @property
    def count(self) -> int:
        return int(self.distances.size)

    @property
    def minimum_distance(self) -> float:
        return float(np.min(self.distances)) if self.count else math.inf

    @property
    def minimum_barrier(self) -> float:
        return float(np.min(self.barrier_values)) if self.count else math.inf

    def to_record(self) -> dict[str, object]:
        return {
            "geometry_source": self.geometry_source,
            "constraint_count": self.count,
            "safe_distance_m": self.safe_distance,
            "gain_per_s": self.gain,
            "minimum_distance_m": self.minimum_distance,
            "minimum_barrier_m": self.minimum_barrier,
            "closest_pair": (
                None
                if not self.count
                else self.pair_labels[int(np.argmin(self.distances))]
            ),
            "evaluation_time_s": self.evaluation_time,
        }


def _primitive_bounding_radius(primitive: Any) -> float:
    primitive_type = type(primitive).__name__.lower()
    if primitive_type == "ball":
        return float(primitive.radius)
    if primitive_type == "cylinder":
        return math.sqrt(
            float(primitive.radius) ** 2 + (0.5 * float(primitive.height)) ** 2
        )
    if primitive_type == "box":
        return 0.5 * math.sqrt(
            float(primitive.width) ** 2
            + float(primitive.depth) ** 2
            + float(primitive.height) ** 2
        )
    raise BoxObstacleCbfError(
        f"Primitiva de colisao nao suportada: {type(primitive).__name__}."
    )


def _signed_distance_and_gradient(
    point: np.ndarray,
    *,
    center: np.ndarray,
    half_extents: np.ndarray,
) -> tuple[float, np.ndarray, np.ndarray]:
    """Retorna SDF, gradiente e ponto mais proximo na superficie da caixa."""

    relative = np.asarray(point, dtype=float) - center
    absolute = np.abs(relative)
    gap = absolute - half_extents
    outside = np.maximum(gap, 0.0)
    outside_norm = float(np.linalg.norm(outside))

    if outside_norm > 1e-12:
        signs = np.where(relative >= 0.0, 1.0, -1.0)
        gradient = signs * outside / outside_norm
        closest = center + np.clip(relative, -half_extents, half_extents)
        return outside_norm, gradient, closest

    # Dentro da caixa, a distancia assinada e a menor penetracao ate uma face.
    active_axis = int(np.argmax(gap))
    gradient = np.zeros(3, dtype=float)
    gradient[active_axis] = (
        1.0 if relative[active_axis] >= 0.0 else -1.0
    )
    closest_relative = relative.copy()
    closest_relative[active_axis] = (
        gradient[active_axis] * half_extents[active_axis]
    )
    return float(gap[active_axis]), gradient, center + closest_relative


def evaluate_box_obstacle_distances(
    robot: Any,
    configuration: Sequence[float],
    *,
    center: Sequence[float],
    size: Sequence[float],
    geometry_source: str,
    excluded_pair_labels: Sequence[str] = (),
) -> BoxObstacleDistances:
    """Avalia distancias e Jacobianos para todas as primitivas selecionadas."""

    positions = np.asarray(configuration, dtype=float).reshape(-1)
    obstacle_center = np.asarray(center, dtype=float).reshape(-1)
    obstacle_size = np.asarray(size, dtype=float).reshape(-1)
    if positions.size != len(robot.links) or not np.all(np.isfinite(positions)):
        raise BoxObstacleCbfError("Configuracao articular invalida.")
    if obstacle_center.size != 3 or not np.all(np.isfinite(obstacle_center)):
        raise BoxObstacleCbfError("center deve conter tres valores finitos.")
    if (
        obstacle_size.size != 3
        or not np.all(np.isfinite(obstacle_size))
        or np.any(obstacle_size <= 0.0)
    ):
        raise BoxObstacleCbfError(
            "size deve conter tres dimensoes finitas e positivas."
        )

    excluded = {str(label) for label in excluded_pair_labels}
    half_extents = 0.5 * obstacle_size
    start = time.perf_counter()
    try:
        jacobians, dh_transforms = robot.jac_geo(positions, "dh", mode="python")
    except Exception as error:
        raise BoxObstacleCbfError(
            f"Falha na cinematica DH do obstaculo em caixa: {error}"
        ) from error

    distances: list[float] = []
    jacobian_rows: list[np.ndarray] = []
    robot_witnesses: list[np.ndarray] = []
    obstacle_witnesses: list[np.ndarray] = []
    labels: list[str] = []
    for link_index, link in enumerate(robot.links):
        dh = np.asarray(dh_transforms[link_index], dtype=float)
        link_jacobian = np.asarray(jacobians[link_index], dtype=float)
        if dh.shape != (4, 4) or link_jacobian.shape != (6, positions.size):
            raise BoxObstacleCbfError("Frame ou Jacobiano DH invalido.")
        for object_index, item in enumerate(link.col_objects):
            pair_id = f"link_{link_index}_obj_{object_index}"
            label = f"{pair_id}__cube"
            if pair_id in excluded or label in excluded:
                continue
            try:
                primitive, attached = item
                attached_transform = np.asarray(attached, dtype=float)
            except (TypeError, ValueError) as error:
                raise BoxObstacleCbfError(
                    f"Primitiva invalida em {pair_id}."
                ) from error
            if attached_transform.shape != (4, 4):
                raise BoxObstacleCbfError(
                    f"Transformacao invalida em {pair_id}."
                )
            world = dh @ attached_transform
            point = world[:3, 3]
            primitive_radius = _primitive_bounding_radius(primitive)
            signed_distance, gradient, obstacle_point = (
                _signed_distance_and_gradient(
                    point,
                    center=obstacle_center,
                    half_extents=half_extents,
                )
            )
            point_jacobian = (
                link_jacobian[:3, :]
                - _skew(point - dh[:3, 3]) @ link_jacobian[3:6, :]
            )
            distance_jacobian = gradient.reshape(1, 3) @ point_jacobian
            distances.append(float(signed_distance - primitive_radius))
            jacobian_rows.append(distance_jacobian.reshape(-1))
            robot_witnesses.append(point.copy())
            obstacle_witnesses.append(obstacle_point.copy())
            labels.append(label)

    if not distances:
        raise BoxObstacleCbfError(
            "O modelo nao possui primitivas de colisao selecionadas."
        )
    return BoxObstacleDistances(
        distances=np.asarray(distances, dtype=float),
        jacobian=np.vstack(jacobian_rows),
        pair_labels=tuple(labels),
        first_witness_points=np.vstack(robot_witnesses),
        second_witness_points=np.vstack(obstacle_witnesses),
        geometry_source=str(geometry_source),
        evaluation_time=time.perf_counter() - start,
    )


def formulate_box_obstacle_cbf(
    distances: BoxObstacleDistances,
    *,
    safe_distance: float,
    gain: float,
) -> BoxObstacleCbfConstraints:
    """Formula uma CBF de primeira ordem para as distancias avaliadas."""

    if not math.isfinite(safe_distance) or safe_distance <= 0.0:
        raise BoxObstacleCbfError("safe_distance deve ser finita e positiva.")
    if not math.isfinite(gain) or gain <= 0.0:
        raise BoxObstacleCbfError("gain deve ser finito e positivo.")
    values = np.asarray(distances.distances, dtype=float).reshape(-1)
    matrix = np.asarray(distances.jacobian, dtype=float)
    if (
        matrix.ndim != 2
        or matrix.shape[0] != values.size
        or not np.all(np.isfinite(matrix))
    ):
        raise BoxObstacleCbfError("Jacobiano do obstaculo invalido.")
    if not np.all(np.isfinite(values)):
        raise BoxObstacleCbfError("Distancias do obstaculo nao finitas.")
    barrier = values - float(safe_distance)
    return BoxObstacleCbfConstraints(
        matrix=matrix.copy(),
        lower_bound=-float(gain) * barrier,
        barrier_values=barrier,
        distances=values.copy(),
        pair_labels=tuple(distances.pair_labels),
        first_witness_points=np.asarray(
            distances.first_witness_points, dtype=float
        ).copy(),
        second_witness_points=np.asarray(
            distances.second_witness_points, dtype=float
        ).copy(),
        safe_distance=float(safe_distance),
        gain=float(gain),
        geometry_source=str(distances.geometry_source),
        evaluation_time=float(distances.evaluation_time),
    )
