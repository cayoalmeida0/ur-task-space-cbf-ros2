"""Marcadores RViz para obstaculos geometricos externos da CBF."""

from typing import Any, Sequence

import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def _point(values: Sequence[float]) -> Point:
    point = Point()
    point.x, point.y, point.z = (float(value) for value in values)
    return point


def _box_edge_points(center: np.ndarray, size: np.ndarray) -> list[Point]:
    half = 0.5 * size
    corners = np.array(
        [
            (-half[0], -half[1], -half[2]),
            (half[0], -half[1], -half[2]),
            (half[0], half[1], -half[2]),
            (-half[0], half[1], -half[2]),
            (-half[0], -half[1], half[2]),
            (half[0], -half[1], half[2]),
            (half[0], half[1], half[2]),
            (-half[0], half[1], half[2]),
        ],
        dtype=float,
    ) + center
    edges = (
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    )
    points: list[Point] = []
    for first, second in edges:
        points.extend((_point(corners[first]), _point(corners[second])))
    return points


def build_box_obstacle_marker_array(
    center: Sequence[float],
    size: Sequence[float],
    *,
    safe_distance: float,
    frame_id: str,
    stamp: Any,
    namespace: str = "cube_cbf_obstacle",
) -> MarkerArray:
    """Cria a caixa fisica e a linha da margem CBF para o RViz."""

    center_array = np.asarray(center, dtype=float).reshape(-1)
    size_array = np.asarray(size, dtype=float).reshape(-1)
    if center_array.size != 3 or not np.all(np.isfinite(center_array)):
        raise ValueError("center deve conter tres valores finitos.")
    if (
        size_array.size != 3
        or not np.all(np.isfinite(size_array))
        or np.any(size_array <= 0.0)
    ):
        raise ValueError("size deve conter tres dimensoes positivas.")
    if not np.isfinite(safe_distance) or safe_distance < 0.0:
        raise ValueError("safe_distance deve ser finita e nao negativa.")
    if not str(frame_id).strip():
        raise ValueError("frame_id nao pode ser vazio.")
    if not str(namespace).strip():
        raise ValueError("namespace nao pode ser vazio.")

    markers = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    markers.markers.append(clear)

    cube = Marker()
    cube.header.frame_id = str(frame_id)
    cube.header.stamp = stamp
    cube.ns = str(namespace)
    cube.id = 0
    cube.type = Marker.CUBE
    cube.action = Marker.ADD
    cube.pose.position = _point(center_array)
    cube.pose.orientation.w = 1.0
    cube.scale.x, cube.scale.y, cube.scale.z = (
        float(value) for value in size_array
    )
    cube.color = ColorRGBA()
    cube.color.r, cube.color.g, cube.color.b, cube.color.a = (
        1.0,
        0.25,
        0.05,
        0.22,
    )
    markers.markers.append(cube)

    if safe_distance > 0.0:
        shell = Marker()
        shell.header.frame_id = str(frame_id)
        shell.header.stamp = stamp
        shell.ns = f"{namespace}_safe_margin"
        shell.id = 1
        shell.type = Marker.LINE_LIST
        shell.action = Marker.ADD
        shell.scale.x = 0.003
        shell.color = ColorRGBA()
        shell.color.r, shell.color.g, shell.color.b, shell.color.a = (
            1.0,
            0.85,
            0.0,
            0.95,
        )
        shell.points = _box_edge_points(
            center_array,
            size_array + 2.0 * float(safe_distance),
        )
        markers.markers.append(shell)
    return markers
