"""Marcadores RViz para os witness points das CBFs."""

from typing import Any, Sequence

import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

WITNESS_VISUALIZATION_MODES = ("off", "closest", "all")


def _color(distance: float, safe_distance: float) -> ColorRGBA:
    color = ColorRGBA()
    color.a = 1.0
    if distance < safe_distance:
        color.r, color.g, color.b = 1.0, 0.1, 0.1
    elif distance < 1.5 * safe_distance:
        color.r, color.g, color.b = 1.0, 0.75, 0.0
    else:
        color.r, color.g, color.b = 0.1, 0.9, 0.2
    return color


def _point(values: Any) -> Point:
    point = Point()
    point.x, point.y, point.z = (float(value) for value in values)
    return point


def build_witness_marker_array(
    constraints: Any,
    *,
    mode: str,
    frame_id: str,
    stamp: Any,
    namespace: str = "self_collision",
    line_width: float = 0.004,
    point_size: float = 0.012,
) -> MarkerArray:
    """Cria pontos e segmentos dos pares avaliados em um frame comum.

    A funcao aceita as estruturas de autocolisao, mesa, cubo e caixa porque
    todas expõem ``distances``, ``safe_distance`` e os dois conjuntos de
    witness points. O ``namespace`` separa visualmente as familias mesmo
    quando um consumidor reutiliza o mesmo display RViz.
    """

    normalized_mode = str(mode).lower()
    if normalized_mode not in WITNESS_VISUALIZATION_MODES:
        raise ValueError(
            "witness_mode deve ser off, closest ou all."
        )
    if not frame_id.strip():
        raise ValueError("witness_frame nao pode ser vazio.")
    if not namespace.strip():
        raise ValueError("namespace nao pode ser vazio.")
    if line_width <= 0.0 or not np.isfinite(line_width):
        raise ValueError("line_width deve ser positivo e finito.")
    if point_size <= 0.0 or not np.isfinite(point_size):
        raise ValueError("point_size deve ser positivo e finito.")

    markers = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    markers.markers.append(clear)
    if normalized_mode == "off" or constraints.count == 0:
        return markers

    if normalized_mode == "closest":
        indices = (int(constraints.distances.argmin()),)
    else:
        indices = tuple(range(constraints.count))

    for marker_id, index in enumerate(indices):
        color = _color(
            float(constraints.distances[index]), constraints.safe_distance
        )
        first = _point(constraints.first_witness_points[index])
        second = _point(constraints.second_witness_points[index])

        line = Marker()
        line.header.frame_id = frame_id
        line.header.stamp = stamp
        line.ns = f"{namespace}_lines"
        line.id = marker_id
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.scale.x = float(line_width)
        line.color = color
        line.points = [first, second]
        markers.markers.append(line)

        points = Marker()
        points.header.frame_id = frame_id
        points.header.stamp = stamp
        points.ns = f"{namespace}_points"
        points.id = marker_id
        points.type = Marker.SPHERE_LIST
        points.action = Marker.ADD
        points.scale.x = points.scale.y = points.scale.z = float(point_size)
        points.color = color
        points.points = [first, second]
        markers.markers.append(points)

    return markers


def build_single_witness_marker_array(
    first: Sequence[float],
    second: Sequence[float],
    *,
    frame_id: str,
    stamp: Any,
    namespace: str,
    color: tuple[float, float, float] = (0.25, 0.85, 1.0),
    line_width: float = 0.005,
    point_size: float = 0.014,
) -> MarkerArray:
    """Cria uma linha única para uma relação sem lista de pares.

    É usada pela fronteira cartesiana, que tem seis planos de CBF mas precisa
    mostrar somente o plano atualmente mais próximo do efetuador.
    """

    first_array = np.asarray(first, dtype=float).reshape(-1)
    second_array = np.asarray(second, dtype=float).reshape(-1)
    if (
        first_array.size != 3
        or second_array.size != 3
        or not np.all(np.isfinite(first_array))
        or not np.all(np.isfinite(second_array))
    ):
        raise ValueError("witness points devem conter tres valores finitos.")
    if not frame_id.strip():
        raise ValueError("frame_id nao pode ser vazio.")
    if not namespace.strip():
        raise ValueError("namespace nao pode ser vazio.")
    if len(color) != 3 or not all(np.isfinite(value) for value in color):
        raise ValueError("color deve conter tres valores finitos.")
    if line_width <= 0.0 or point_size <= 0.0:
        raise ValueError("line_width e point_size devem ser positivos.")

    markers = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    markers.markers.append(clear)

    line = Marker()
    line.header.frame_id = frame_id
    line.header.stamp = stamp
    line.ns = f"{namespace}_line"
    line.id = 0
    line.type = Marker.LINE_LIST
    line.action = Marker.ADD
    line.scale.x = float(line_width)
    line.color.r, line.color.g, line.color.b, line.color.a = (
        float(color[0]),
        float(color[1]),
        float(color[2]),
        1.0,
    )
    line.points = [_point(first_array), _point(second_array)]
    markers.markers.append(line)

    points = Marker()
    points.header.frame_id = frame_id
    points.header.stamp = stamp
    points.ns = f"{namespace}_points"
    points.id = 0
    points.type = Marker.SPHERE_LIST
    points.action = Marker.ADD
    points.scale.x = points.scale.y = points.scale.z = float(point_size)
    points.color = line.color
    points.points = [_point(first_array), _point(second_array)]
    markers.markers.append(points)
    return markers
