"""Formulacao cinemática da CBF de evitacao de autocolisao."""

from dataclasses import dataclass
import math
import time
from typing import Any, Sequence

import numpy as np


class SelfCollisionCbfError(RuntimeError):
    """Indica geometria, parametro ou resultado de distancia invalido."""


def _skew(vector: np.ndarray) -> np.ndarray:
    """Retorna a matriz antissimetrica de um vetor tridimensional."""

    values = np.asarray(vector, dtype=float).reshape(-1)
    if values.size != 3 or not np.all(np.isfinite(values)):
        raise SelfCollisionCbfError(
            "Ponto testemunha UAIbot deve conter tres valores finitos."
        )
    x, y, z = values
    return np.array(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)))


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


def evaluate_uaibot_nonadjacent_distances(
    robot: Any,
    configuration: Sequence[float],
    *,
    distance_utils: Any,
    tolerance: float,
    max_iterations: int,
    geometry_source: str,
) -> SelfCollisionDistances:
    """Avalia os pares nao adjacentes usando a distancia publica do UAIbot.

    O ``compute_dist_auto`` Python do UAIbot 1.2.7 tenta desempacotar tres
    valores de ``Utils.compute_dist``, embora essa funcao documente e retorne
    quatro. Este avaliador reproduz a cinemática diferencial do algoritmo
    fixado, preserva a exclusao de elos adjacentes e trata explicitamente o
    quarto retorno, sem modificar a dependencia instalada.
    """

    positions = np.asarray(configuration, dtype=float).reshape(-1)
    links = getattr(robot, "links", None)
    if links is None or len(links) != positions.size:
        raise SelfCollisionCbfError(
            "Quantidade de elos UAIbot difere da configuracao articular."
        )
    if not np.all(np.isfinite(positions)):
        raise SelfCollisionCbfError("Configuracao contém NaN ou infinito.")
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise SelfCollisionCbfError(
            "Tolerancia de distancia deve ser finita e positiva."
        )
    if max_iterations <= 0:
        raise SelfCollisionCbfError(
            "Numero maximo de iteracoes de distancia deve ser positivo."
        )
    compute_distance = getattr(distance_utils, "compute_dist", None)
    if not callable(compute_distance):
        raise SelfCollisionCbfError(
            "UAIbot nao expoe Utils.compute_dist para o backend do projeto."
        )

    start = time.perf_counter()
    try:
        jacobians, dh_transforms = robot.jac_geo(
            positions,
            "dh",
            mode="python",
        )
    except Exception as error:
        raise SelfCollisionCbfError(
            f"Falha na cinemática DH usada pela autocolisao: {error}"
        ) from error
    if len(jacobians) != len(links) or len(dh_transforms) != len(links):
        raise SelfCollisionCbfError(
            "UAIbot retornou quantidade invalida de frames ou Jacobianos DH."
        )

    world_objects: list[list[Any]] = []
    for link_index, link in enumerate(links):
        dh_transform = np.asarray(dh_transforms[link_index], dtype=float)
        jacobian = np.asarray(jacobians[link_index], dtype=float)
        if dh_transform.shape != (4, 4) or not np.all(np.isfinite(dh_transform)):
            raise SelfCollisionCbfError(
                f"Transformacao DH invalida no elo {link_index}."
            )
        if jacobian.shape != (6, positions.size) or not np.all(
            np.isfinite(jacobian)
        ):
            raise SelfCollisionCbfError(
                f"Jacobiano DH invalido no elo {link_index}."
            )

        link_world_objects: list[Any] = []
        for object_index, item in enumerate(link.col_objects):
            try:
                primitive, attached_transform = item
                attached_array = np.asarray(attached_transform, dtype=float)
                if attached_array.shape != (4, 4):
                    raise ValueError("transformacao anexada nao e 4x4")
                primitive_copy = primitive.copy()
                primitive_copy.set_ani_frame(dh_transform @ attached_array)
            except Exception as error:
                raise SelfCollisionCbfError(
                    "Objeto de colisao UAIbot invalido em "
                    f"link_{link_index}_obj_{object_index}: {error}"
                ) from error
            link_world_objects.append(primitive_copy)
        world_objects.append(link_world_objects)

    distance_values: list[float] = []
    distance_jacobians: list[np.ndarray] = []
    labels: list[str] = []
    for first_link in range(len(links)):
        for second_link in range(first_link + 2, len(links)):
            for first_object, first_primitive in enumerate(
                world_objects[first_link]
            ):
                for second_object, second_primitive in enumerate(
                    world_objects[second_link]
                ):
                    initial_point = np.random.uniform(-100.0, 100.0, (3, 1))
                    try:
                        result = compute_distance(
                            first_primitive,
                            second_primitive,
                            p_a_init=initial_point,
                            tol=float(tolerance),
                            no_iter_max=int(max_iterations),
                            h=0.0,
                            eps=0.0,
                            mode="python",
                        )
                    except Exception as error:
                        raise SelfCollisionCbfError(
                            "Falha na distancia UAIbot do par "
                            f"link_{first_link}_obj_{first_object}__"
                            f"link_{second_link}_obj_{second_object}: {error}"
                        ) from error
                    if not isinstance(result, (tuple, list)) or len(result) != 4:
                        raise SelfCollisionCbfError(
                            "Utils.compute_dist do UAIbot deve retornar quatro "
                            "valores: dois pontos, distancia e historico."
                        )
                    point_first, point_second, distance, _history = result
                    first_point = np.asarray(point_first, dtype=float).reshape(-1)
                    second_point = np.asarray(point_second, dtype=float).reshape(-1)
                    distance_value = float(distance)
                    if (
                        first_point.size != 3
                        or second_point.size != 3
                        or not np.all(np.isfinite(first_point))
                        or not np.all(np.isfinite(second_point))
                        or not math.isfinite(distance_value)
                        or distance_value <= 0.0
                    ):
                        raise SelfCollisionCbfError(
                            "UAIbot retornou pontos ou distancia invalidos para "
                            f"link_{first_link}_obj_{first_object}__"
                            f"link_{second_link}_obj_{second_object}."
                        )

                    first_dh_position = np.asarray(
                        dh_transforms[first_link], dtype=float
                    )[:3, 3]
                    second_dh_position = np.asarray(
                        dh_transforms[second_link], dtype=float
                    )[:3, 3]
                    first_jacobian = np.asarray(
                        jacobians[first_link], dtype=float
                    )
                    second_jacobian = np.asarray(
                        jacobians[second_link], dtype=float
                    )
                    first_point_jacobian = (
                        first_jacobian[:3, :]
                        - _skew(first_point - first_dh_position)
                        @ first_jacobian[3:6, :]
                    )
                    second_point_jacobian = (
                        second_jacobian[:3, :]
                        - _skew(second_point - second_dh_position)
                        @ second_jacobian[3:6, :]
                    )
                    delta = first_point - second_point
                    jacobian_distance = (
                        delta.reshape(1, 3)
                        @ (first_point_jacobian - second_point_jacobian)
                        / distance_value
                    ).reshape(-1)
                    if not np.all(np.isfinite(jacobian_distance)):
                        raise SelfCollisionCbfError(
                            "Jacobiano de distancia nao finito para "
                            f"link_{first_link}_obj_{first_object}__"
                            f"link_{second_link}_obj_{second_object}."
                        )

                    distance_values.append(distance_value)
                    distance_jacobians.append(jacobian_distance)
                    labels.append(
                        f"link_{first_link}_obj_{first_object}__"
                        f"link_{second_link}_obj_{second_object}"
                    )

    if not distance_values:
        raise SelfCollisionCbfError(
            "UAIbot nao retornou pares nao adjacentes de colisao."
        )
    return SelfCollisionDistances(
        distances=np.asarray(distance_values, dtype=float),
        jacobian=np.vstack(distance_jacobians),
        pair_labels=tuple(labels),
        geometry_source=str(geometry_source),
        evaluation_time=time.perf_counter() - start,
    )
