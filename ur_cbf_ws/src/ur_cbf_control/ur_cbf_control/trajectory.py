"""Perfis reproduziveis de trajetoria cartesiana."""

import math
from typing import Sequence


SUPPORTED_TRAJECTORY_PROFILES = ("simple", "complex")

COMPLEX_WAYPOINT_OFFSETS_M = (
    (0.0, 0.0, 0.05),
    (0.05, 0.0, 0.05),
    (0.05, 0.05, 0.02),
    (-0.03, 0.05, 0.06),
    (0.0, 0.0, 0.0),
)


def resolve_trajectory_waypoints(
    profile: str,
    target_offset: Sequence[float],
) -> tuple[tuple[float, float, float], ...]:
    """Retorna offsets cartesianos relativos à posição inicial."""

    normalized = str(profile).strip().lower()
    if normalized not in SUPPORTED_TRAJECTORY_PROFILES:
        raise ValueError(
            "trajectory_profile deve ser simple ou complex."
        )

    simple_offset = tuple(float(value) for value in target_offset)
    if len(simple_offset) != 3 or not all(
        math.isfinite(value) for value in simple_offset
    ):
        raise ValueError("target_offset deve conter tres valores finitos.")

    if normalized == "simple":
        return (simple_offset,)
    return COMPLEX_WAYPOINT_OFFSETS_M
