"""CBF e geometria visual para um envelope cartesiano axis-aligned."""

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


class WorkspaceBoundaryCbfError(ValueError):
    """Indica limites ou dados invalidos do envelope cartesiano."""


WORKSPACE_BOUNDARY_LABELS = (
    "x_min",
    "x_max",
    "y_min",
    "y_max",
    "z_min",
    "z_max",
)


@dataclass(frozen=True)
class WorkspaceBoundaryCbfConstraints:
    """Restricoes ``matrix @ qdot >= lower_bound`` para seis planos."""

    matrix: np.ndarray
    lower_bound: np.ndarray
    barrier_values: np.ndarray
    position: np.ndarray
    bounds: np.ndarray
    safety_margin: float
    gain: float
    labels: tuple[str, ...] = WORKSPACE_BOUNDARY_LABELS

    @property
    def count(self) -> int:
        return int(self.barrier_values.size)

    @property
    def minimum_barrier(self) -> float:
        return float(np.min(self.barrier_values))

    @property
    def closest_boundary(self) -> str:
        return self.labels[int(np.argmin(self.barrier_values))]

    def to_record(self) -> dict[str, object]:
        """Resume a avaliacao sem gravar as matrizes do QP."""

        return {
            "constraint_count": self.count,
            "minimum_barrier_m": self.minimum_barrier,
            "closest_boundary": self.closest_boundary,
            "position_m": self.position.tolist(),
            "bounds_m": self.bounds.reshape(-1).tolist(),
            "safety_margin_m": self.safety_margin,
            "gain_per_s": self.gain,
        }


def _validate_inputs(
    position: Sequence[float],
    translational_jacobian: Sequence[Sequence[float]],
    bounds: Sequence[float],
    safety_margin: float,
    gain: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    point = np.asarray(position, dtype=float).reshape(-1)
    if point.shape != (3,) or not np.all(np.isfinite(point)):
        raise WorkspaceBoundaryCbfError(
            "Posicao do efetuador deve conter tres valores finitos."
        )
    jacobian = np.asarray(translational_jacobian, dtype=float)
    if jacobian.ndim != 2 or jacobian.shape[0] != 3 or jacobian.shape[1] == 0:
        raise WorkspaceBoundaryCbfError(
            "Jacobiano translacional deve possuir forma 3 x n."
        )
    if not np.all(np.isfinite(jacobian)):
        raise WorkspaceBoundaryCbfError(
            "Jacobiano translacional contem NaN ou infinito."
        )
    raw_bounds = np.asarray(bounds, dtype=float).reshape(-1)
    if raw_bounds.size != 6 or not np.all(np.isfinite(raw_bounds)):
        raise WorkspaceBoundaryCbfError(
            "workspace_bounds deve conter [xmin, xmax, ymin, ymax, zmin, zmax]."
        )
    envelope = raw_bounds.reshape(3, 2)
    if np.any(envelope[:, 0] >= envelope[:, 1]):
        raise WorkspaceBoundaryCbfError(
            "Cada limite inferior do workspace deve ser menor que o superior."
        )
    if not math.isfinite(safety_margin) or safety_margin < 0.0:
        raise WorkspaceBoundaryCbfError(
            "workspace_safe_margin deve ser finita e nao negativa."
        )
    if np.any(envelope[:, 1] - envelope[:, 0] <= 2.0 * safety_margin):
        raise WorkspaceBoundaryCbfError(
            "A margem de seguranca deve caber dentro do envelope."
        )
    if not math.isfinite(gain) or gain <= 0.0:
        raise WorkspaceBoundaryCbfError(
            "workspace_cbf_gain deve ser finito e positivo."
        )
    return point, jacobian, envelope


def formulate_workspace_boundary_cbf(
    position: Sequence[float],
    translational_jacobian: Sequence[Sequence[float]],
    *,
    bounds: Sequence[float],
    safety_margin: float,
    gain: float,
) -> WorkspaceBoundaryCbfConstraints:
    """Formula seis CBFs de primeira ordem no frame cartesiano da cinemática.

    Para cada eixo, as barreiras sao ``p_i - p_min - margin`` e
    ``p_max - p_i - margin``. Como ``p_dot = J_p q_dot``, cada linha e
    inserida diretamente no QP na forma ``A q_dot >= -gamma h``.
    """

    point, jacobian, envelope = _validate_inputs(
        position,
        translational_jacobian,
        bounds,
        safety_margin,
        gain,
    )
    lower_barriers = point - envelope[:, 0] - float(safety_margin)
    upper_barriers = envelope[:, 1] - point - float(safety_margin)
    barriers = np.empty(6, dtype=float)
    barriers[0::2] = lower_barriers
    barriers[1::2] = upper_barriers

    rows = np.empty((6, jacobian.shape[1]), dtype=float)
    rows[0::2] = jacobian
    rows[1::2] = -jacobian
    return WorkspaceBoundaryCbfConstraints(
        matrix=rows,
        lower_bound=-float(gain) * barriers,
        barrier_values=barriers,
        position=point.copy(),
        bounds=envelope.copy(),
        safety_margin=float(safety_margin),
        gain=float(gain),
    )


def workspace_boundary_vertices(bounds: Sequence[float]) -> np.ndarray:
    """Retorna os oito vertices do envelope em ordem deterministica."""

    raw_bounds = np.asarray(bounds, dtype=float).reshape(-1)
    if raw_bounds.size != 6 or not np.all(np.isfinite(raw_bounds)):
        raise WorkspaceBoundaryCbfError(
            "workspace_bounds deve conter seis valores finitos."
        )
    xmin, xmax, ymin, ymax, zmin, zmax = raw_bounds
    if xmin >= xmax or ymin >= ymax or zmin >= zmax:
        raise WorkspaceBoundaryCbfError("Limites do workspace sao invalidos.")
    return np.asarray(
        (
            (xmin, ymin, zmin),
            (xmax, ymin, zmin),
            (xmax, ymax, zmin),
            (xmin, ymax, zmin),
            (xmin, ymin, zmax),
            (xmax, ymin, zmax),
            (xmax, ymax, zmax),
            (xmin, ymax, zmax),
        ),
        dtype=float,
    )


def build_workspace_boundary_marker_array(
    bounds: Sequence[float],
    *,
    frame_id: str,
    stamp,
    line_width: float = 0.005,
):
    """Cria um ``MarkerArray`` com as doze arestas do envelope."""

    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point

    if not frame_id.strip():
        raise WorkspaceBoundaryCbfError("workspace_boundary_frame nao pode ser vazio.")
    if not math.isfinite(line_width) or line_width <= 0.0:
        raise WorkspaceBoundaryCbfError("workspace_boundary_line_width deve ser positivo.")
    vertices = workspace_boundary_vertices(bounds)
    edges = (
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    )
    markers = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    markers.markers.append(clear)

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "workspace_boundary"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = float(line_width)
    marker.color.r = 0.15
    marker.color.g = 0.90
    marker.color.b = 0.25
    marker.color.a = 0.95
    for first, second in edges:
        for index in (first, second):
            point = Point()
            point.x, point.y, point.z = (float(value) for value in vertices[index])
            marker.points.append(point)
    markers.markers.append(marker)
    return markers
