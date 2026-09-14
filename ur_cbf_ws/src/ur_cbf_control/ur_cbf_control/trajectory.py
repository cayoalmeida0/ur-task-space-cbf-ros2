"""Perfis reproduziveis de trajetoria cartesiana."""

import math
from typing import Sequence


SUPPORTED_TRAJECTORY_PROFILES = ("simple", "complex", "challenging")

COMPLEX_WAYPOINT_OFFSETS_M = (
    (0.0, 0.0, 0.05),
    (0.05, 0.0, 0.05),
    (0.05, 0.05, 0.02),
    (-0.03, 0.05, 0.06),
    (0.0, 0.0, 0.0),
)

# Regioes mais baixas e proximas ao eixo da base, com alternancia entre X+ e X-.
CHALLENGING_WAYPOINT_OFFSETS_M = (
    (0.12, 0.05, -0.15),
    (0.18, 0.25, -0.28),
    (-0.18, 0.28, -0.32),
    (-0.20, -0.02, -0.12),
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
            "trajectory_profile deve ser simple, complex ou challenging."
        )

    simple_offset = tuple(float(value) for value in target_offset)
    if len(simple_offset) != 3 or not all(
        math.isfinite(value) for value in simple_offset
    ):
        raise ValueError("target_offset deve conter tres valores finitos.")

    if normalized == "simple":
        return (simple_offset,)
    if normalized == "complex":
        return COMPLEX_WAYPOINT_OFFSETS_M
    return CHALLENGING_WAYPOINT_OFFSETS_M
