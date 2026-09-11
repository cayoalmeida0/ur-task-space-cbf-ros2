"""Marcadores RViz para os witness points da CBF de autocolisao."""

from typing import Any

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from ur_cbf_control.self_collision_cbf import SelfCollisionCbfConstraints


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
    constraints: SelfCollisionCbfConstraints,
    *,
    mode: str,
    frame_id: str,
    stamp: Any,
) -> MarkerArray:
    """Cria pontos e segmentos dos pares avaliados no frame do UAIbot."""

    normalized_mode = str(mode).lower()
    if normalized_mode not in WITNESS_VISUALIZATION_MODES:
        raise ValueError(
            "self_collision_witness_mode deve ser off, closest ou all."
        )
    if not frame_id.strip():
        raise ValueError("self_collision_witness_frame nao pode ser vazio.")

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
        line.ns = "self_collision_witness_lines"
        line.id = marker_id
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.scale.x = 0.004
        line.color = color
        line.points = [first, second]
        markers.markers.append(line)

        points = Marker()
        points.header.frame_id = frame_id
        points.header.stamp = stamp
        points.ns = "self_collision_witness_points"
        points.id = marker_id
        points.type = Marker.SPHERE_LIST
        points.action = Marker.ADD
        points.scale.x = points.scale.y = points.scale.z = 0.012
        points.color = color
        points.points = [first, second]
        markers.markers.append(points)

    return markers
