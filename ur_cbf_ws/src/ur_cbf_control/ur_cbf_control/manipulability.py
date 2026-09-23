"""Métricas numéricas de manipulabilidade para registros experimentais."""

import math
from typing import Sequence

import numpy as np


def evaluate_manipulability(jacobian: Sequence[Sequence[float]]) -> dict[str, float]:
    """Retorna valores singulares, ``sigma_min`` e índice de Yoshikawa.

    O índice é calculado sobre o Jacobiano da tarefa efetivamente controlada:
    ``Jv`` no modo posicional ou ``[Jv; Jw]`` no modo de pose. Isso evita
    misturar unidades sem que o experimento declare explicitamente a tarefa.
    """

    matrix = np.asarray(jacobian, dtype=float)
    if matrix.ndim != 2 or matrix.size == 0 or not np.all(np.isfinite(matrix)):
        raise ValueError("O Jacobiano deve ser uma matriz finita não vazia.")
    singular_values = np.linalg.svd(matrix, compute_uv=False)
    sigma_min = float(np.min(singular_values))
    sigma_max = float(np.max(singular_values))
    yoshikawa = float(np.prod(singular_values))
    condition_number = (
        math.inf if sigma_min <= np.finfo(float).eps else sigma_max / sigma_min
    )
    return {
        "sigma_min": sigma_min,
        "sigma_max": sigma_max,
        "condition_number": condition_number,
        "yoshikawa_index": yoshikawa,
    }
