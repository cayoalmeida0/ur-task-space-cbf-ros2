"""CBF diferencial para um obstáculo cilíndrico estático.

O obstáculo é descrito como um cilindro vertical finito. Cada primitiva de
colisão do robô é conservadoramente envolvida por uma esfera; a distância
assinada entre essa esfera e o cilindro fornece uma restrição diferencial
``J_d qdot >= -gamma (d - d_safe)``. A aproximação é deliberadamente
conservadora e evita depender de uma API privada de distância da biblioteca
cinemática.
"""

from dataclasses import dataclass
import math
import time
from typing import Any, Sequence

import numpy as np


class CylinderObstacleCbfError(RuntimeError):
    """Indica geometria ou parâmetros inválidos do obstáculo cilíndrico."""


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=float).reshape(-1)
    return np.array(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)))


@dataclass(frozen=True)
class CylinderObstacleDistances:
    """Distâncias assinadas entre primitivas do robô e o cilindro."""

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
class CylinderObstacleCbfConstraints:
    """Restrições ``matrix @ qdot >= lower_bound`` do obstáculo."""

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
    raise CylinderObstacleCbfError(
        f"Primitiva de colisão não suportada: {type(primitive).__name__}."
    )


def _signed_distance_and_gradient(
    point: np.ndarray,
    *,
    center_xy: np.ndarray,
    radius: float,
    z_min: float,
    z_max: float,
) -> tuple[float, np.ndarray]:
    """Distância assinada à superfície do cilindro e seu gradiente em ``p``."""

    relative_xy = point[:2] - center_xy
    radial_norm = float(np.linalg.norm(relative_xy))
    if radial_norm > 1e-12:
        radial_gradient = np.array(
            (relative_xy[0] / radial_norm, relative_xy[1] / radial_norm, 0.0)
        )
    else:
        radial_gradient = np.array((1.0, 0.0, 0.0))
    radial_gap = radial_norm - radius

    if radial_gap >= 0.0 and z_min <= point[2] <= z_max:
        return radial_gap, radial_gradient
    if radial_gap < 0.0 and point[2] < z_min:
        return z_min - point[2], np.array((0.0, 0.0, -1.0))
    if radial_gap < 0.0 and point[2] > z_max:
        return point[2] - z_max, np.array((0.0, 0.0, 1.0))
    if radial_gap >= 0.0 and point[2] < z_min:
        delta_z = z_min - point[2]
        distance = math.hypot(radial_gap, delta_z)
        return distance, (radial_gap * radial_gradient - delta_z * np.array((0.0, 0.0, 1.0))) / distance
    if radial_gap >= 0.0 and point[2] > z_max:
        delta_z = point[2] - z_max
        distance = math.hypot(radial_gap, delta_z)
        return distance, (radial_gap * radial_gradient + delta_z * np.array((0.0, 0.0, 1.0))) / distance

    margins = np.array((radial_gap, point[2] - z_min, z_max - point[2]))
    active = int(np.argmin(margins))
    if active == 0:
        return float(radial_gap), radial_gradient
    if active == 1:
        return float(margins[active]), np.array((0.0, 0.0, 1.0))
    return float(margins[active]), np.array((0.0, 0.0, -1.0))


def evaluate_cylinder_obstacle_distances(
    robot: Any,
    configuration: Sequence[float],
    *,
    center_xy: Sequence[float],
    radius: float,
    height: float,
    geometry_source: str,
) -> CylinderObstacleDistances:
    """Avalia distâncias e Jacobianos para todas as primitivas do robô."""

    positions = np.asarray(configuration, dtype=float).reshape(-1)
    center = np.asarray(center_xy, dtype=float).reshape(-1)
    if positions.size != len(robot.links) or not np.all(np.isfinite(positions)):
        raise CylinderObstacleCbfError("Configuração articular inválida.")
    if center.size != 2 or not np.all(np.isfinite(center)):
        raise CylinderObstacleCbfError("center_xy deve conter dois valores finitos.")
    if not math.isfinite(radius) or radius <= 0.0:
        raise CylinderObstacleCbfError("radius deve ser finito e positivo.")
    if not math.isfinite(height) or height <= 0.0:
        raise CylinderObstacleCbfError("height deve ser finito e positivo.")

    start = time.perf_counter()
    try:
        jacobians, dh_transforms = robot.jac_geo(positions, "dh", mode="python")
    except Exception as error:
        raise CylinderObstacleCbfError(
            f"Falha na cinemática DH do obstáculo cilíndrico: {error}"
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
            raise CylinderObstacleCbfError("Frame ou Jacobiano DH inválido.")
        for object_index, item in enumerate(link.col_objects):
            primitive, attached = item
            world = dh @ np.asarray(attached, dtype=float)
            point = world[:3, 3]
            primitive_radius = _primitive_bounding_radius(primitive)
            distance, gradient = _signed_distance_and_gradient(
                point,
                center_xy=center,
                radius=float(radius) + primitive_radius,
                z_min=0.0,
                z_max=float(height),
            )
            point_jacobian = (
                link_jacobian[:3, :]
                - _skew(point - dh[:3, 3]) @ link_jacobian[3:6, :]
            )
            distance_jacobian = gradient.reshape(1, 3) @ point_jacobian
            obstacle_point = point - gradient * distance
            distances.append(float(distance))
            jacobian_rows.append(distance_jacobian.reshape(-1))
            robot_witnesses.append(point.copy())
            obstacle_witnesses.append(obstacle_point)
            labels.append(f"link_{link_index}_obj_{object_index}__cylinder")

    if not distances:
        raise CylinderObstacleCbfError("O modelo não possui primitivas de colisão.")
    return CylinderObstacleDistances(
        distances=np.asarray(distances, dtype=float),
        jacobian=np.vstack(jacobian_rows),
        pair_labels=tuple(labels),
        first_witness_points=np.vstack(robot_witnesses),
        second_witness_points=np.vstack(obstacle_witnesses),
        geometry_source=str(geometry_source),
        evaluation_time=time.perf_counter() - start,
    )


def formulate_cylinder_obstacle_cbf(
    distances: CylinderObstacleDistances,
    *,
    safe_distance: float,
    gain: float,
) -> CylinderObstacleCbfConstraints:
    """Formula a CBF de primeira ordem para as distâncias avaliadas."""

    if not math.isfinite(safe_distance) or safe_distance <= 0.0:
        raise CylinderObstacleCbfError("safe_distance deve ser finita e positiva.")
    if not math.isfinite(gain) or gain <= 0.0:
        raise CylinderObstacleCbfError("gain deve ser finito e positivo.")
    values = np.asarray(distances.distances, dtype=float).reshape(-1)
    matrix = np.asarray(distances.jacobian, dtype=float)
    if (
        matrix.ndim != 2
        or matrix.shape[0] != values.size
        or not np.all(np.isfinite(matrix))
    ):
        raise CylinderObstacleCbfError("Jacobiano do obstáculo inválido.")
    if not np.all(np.isfinite(values)):
        raise CylinderObstacleCbfError("Distâncias do obstáculo não finitas.")
    barrier = values - float(safe_distance)
    return CylinderObstacleCbfConstraints(
        matrix=matrix.copy(),
        lower_bound=-float(gain) * barrier,
        barrier_values=barrier,
        distances=values.copy(),
        pair_labels=tuple(distances.pair_labels),
        first_witness_points=np.asarray(distances.first_witness_points, dtype=float).copy(),
        second_witness_points=np.asarray(distances.second_witness_points, dtype=float).copy(),
        safe_distance=float(safe_distance),
        gain=float(gain),
        geometry_source=str(distances.geometry_source),
        evaluation_time=float(distances.evaluation_time),
    )
