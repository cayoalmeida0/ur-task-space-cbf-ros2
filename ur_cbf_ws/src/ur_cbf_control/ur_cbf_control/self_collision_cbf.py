"""Formulacao cinemática da CBF de evitacao de autocolisao."""

from dataclasses import dataclass
import math
from typing import Any, Sequence

import numpy as np


class SelfCollisionCbfError(RuntimeError):
    """Indica geometria, parametro ou resultado de distancia invalido."""


@dataclass(frozen=True)
class SelfCollisionDistances:
    """Distancias de superficie e seus Jacobianos na ordem do modelo."""

    distances: np.ndarray
    jacobian: np.ndarray
    pair_labels: tuple[str, ...]
    geometry_source: str
    evaluation_time: float = 0.0

    @property
    def count(self) -> int:
        return int(self.distances.size)

    @property
    def minimum_distance(self) -> float:
        if self.count == 0:
            return math.inf
        return float(np.min(self.distances))

    @property
    def closest_pair(self) -> str | None:
        if self.count == 0:
            return None
        return self.pair_labels[int(np.argmin(self.distances))]


@dataclass(frozen=True)
class SelfCollisionCbfConstraints:
    """Restricoes ``matrix @ qdot >= lower_bound`` para o OSQP."""

    matrix: np.ndarray
    lower_bound: np.ndarray
    barrier_values: np.ndarray
    distances: np.ndarray
    pair_labels: tuple[str, ...]
    safe_distance: float
    gain: float
    geometry_source: str
    evaluation_time: float

    @property
    def count(self) -> int:
        return int(self.distances.size)

    @property
    def minimum_distance(self) -> float:
        if self.count == 0:
            return math.inf
        return float(np.min(self.distances))

    @property
    def minimum_barrier(self) -> float:
        if self.count == 0:
            return math.inf
        return float(np.min(self.barrier_values))

    @property
    def closest_pair(self) -> str | None:
        if self.count == 0:
            return None
        return self.pair_labels[int(np.argmin(self.distances))]

    def to_record(self) -> dict[str, object]:
        """Resume a avaliacao sem gravar matrizes grandes no resultado."""

        return {
            "geometry_source": self.geometry_source,
            "constraint_count": self.count,
            "safe_distance_m": self.safe_distance,
            "gain_per_s": self.gain,
            "minimum_distance_m": self.minimum_distance,
            "minimum_barrier_m": self.minimum_barrier,
            "closest_pair": self.closest_pair,
            "evaluation_time_s": self.evaluation_time,
        }


def formulate_self_collision_cbf(
    distances: SelfCollisionDistances,
    *,
    safe_distance: float,
    gain: float,
) -> SelfCollisionCbfConstraints:
    """Aplica ``J_d qdot >= -gain (d - safe_distance)`` a cada par."""

    if not math.isfinite(safe_distance) or safe_distance <= 0.0:
        raise SelfCollisionCbfError(
            "self_collision_safe_distance deve ser finita e positiva."
        )
    if not math.isfinite(gain) or gain <= 0.0:
        raise SelfCollisionCbfError(
            "self_collision_cbf_gain deve ser finito e positivo."
        )

    distance_vector = np.asarray(distances.distances, dtype=float).reshape(-1)
    distance_jacobian = np.asarray(distances.jacobian, dtype=float)
    if distance_jacobian.ndim != 2:
        raise SelfCollisionCbfError(
            "Jacobiano das distancias deve ser uma matriz bidimensional."
        )
    if distance_jacobian.shape[0] != distance_vector.size:
        raise SelfCollisionCbfError(
            "Quantidade de distancias difere das linhas do Jacobiano."
        )
    if distance_jacobian.shape[1] == 0:
        raise SelfCollisionCbfError(
            "Jacobiano das distancias nao possui velocidades articulares."
        )
    if len(distances.pair_labels) != distance_vector.size:
        raise SelfCollisionCbfError(
            "Quantidade de rotulos difere da quantidade de distancias."
        )
    if not np.all(np.isfinite(distance_vector)) or np.any(distance_vector < 0.0):
        raise SelfCollisionCbfError(
            "Distancias de autocolisao devem ser finitas e nao negativas."
        )
    if not np.all(np.isfinite(distance_jacobian)):
        raise SelfCollisionCbfError(
            "Jacobiano das distancias contem NaN ou infinito."
        )
    if not math.isfinite(distances.evaluation_time) or distances.evaluation_time < 0.0:
        raise SelfCollisionCbfError(
            "Tempo de avaliacao das distancias deve ser finito e nao negativo."
        )

    barrier_values = distance_vector - float(safe_distance)
    lower_bound = -float(gain) * barrier_values
    return SelfCollisionCbfConstraints(
        matrix=distance_jacobian.copy(),
        lower_bound=lower_bound,
        barrier_values=barrier_values,
        distances=distance_vector.copy(),
        pair_labels=tuple(distances.pair_labels),
        safe_distance=float(safe_distance),
        gain=float(gain),
        geometry_source=str(distances.geometry_source),
        evaluation_time=float(distances.evaluation_time),
    )


def distances_from_uaibot_structure(
    distance_structure: Any,
    *,
    joint_count: int,
    evaluation_time: float = 0.0,
    geometry_source: str = "uaibot_internal_collision_objects",
) -> SelfCollisionDistances:
    """Converte ``DistStructRobotAuto`` sem depender de classes privadas."""

    try:
        count = int(distance_structure.no_items)
        distance_vector = np.asarray(
            distance_structure.dist_vect,
            dtype=float,
        ).reshape(-1)
        distance_jacobian = np.asarray(
            distance_structure.jac_dist_mat,
            dtype=float,
        )
    except Exception as error:
        raise SelfCollisionCbfError(
            f"Estrutura de distancias UAIbot invalida: {error}"
        ) from error

    if count <= 0:
        raise SelfCollisionCbfError(
            "UAIbot nao retornou pares nao adjacentes de colisao."
        )
    if distance_vector.size != count or distance_jacobian.shape != (
        count,
        joint_count,
    ):
        raise SelfCollisionCbfError(
            "Dimensoes da estrutura de distancias UAIbot sao inconsistentes."
        )

    labels: list[str] = []
    for index in range(count):
        try:
            item = distance_structure[index]
            labels.append(
                "link_"
                f"{int(item.link_number_1)}_obj_{int(item.link_col_obj_number_1)}"
                "__link_"
                f"{int(item.link_number_2)}_obj_{int(item.link_col_obj_number_2)}"
            )
        except Exception as error:
            raise SelfCollisionCbfError(
                f"Nao foi possivel identificar o par UAIbot {index}: {error}"
            ) from error

    return SelfCollisionDistances(
        distances=distance_vector,
        jacobian=distance_jacobian,
        pair_labels=tuple(labels),
        geometry_source=str(geometry_source),
        evaluation_time=float(evaluation_time),
    )
