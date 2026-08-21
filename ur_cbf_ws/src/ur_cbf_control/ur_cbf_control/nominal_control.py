"""Nucleo matematico do controlador cartesiano nominal de posicao."""

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


class NominalControlError(ValueError):
    """Indica entrada invalida ou falha numerica no controle nominal."""


@dataclass(frozen=True)
class PositionControlResult:
    """Resultado validado de uma iteracao do controle cartesiano."""

    controller_velocity: tuple[float, ...]
    model_velocity: tuple[float, ...]
    cartesian_velocity: tuple[float, ...]
    error: tuple[float, ...]
    error_norm: float
    cartesian_saturated: bool
    joint_saturated: bool


def _finite_vector(values: Sequence[float], label: str) -> np.ndarray:
    vector = np.asarray(values, dtype=float).reshape(-1)
    if vector.size == 0:
        raise NominalControlError(f"{label} esta vazio.")
    if not np.all(np.isfinite(vector)):
        raise NominalControlError(f"{label} contem NaN ou infinito.")
    return vector


def _unique_names(names: Sequence[str], label: str) -> tuple[str, ...]:
    normalized = tuple(str(name) for name in names)
    if not normalized:
        raise NominalControlError(f"{label} esta vazio.")
    if any(not name for name in normalized):
        raise NominalControlError(f"{label} contem nome vazio.")
    if len(set(normalized)) != len(normalized):
        raise NominalControlError(f"{label} contem nomes duplicados.")
    return normalized


def reorder_vector(
    values: Sequence[float],
    source_names: Sequence[str],
    target_names: Sequence[str],
) -> tuple[float, ...]:
    """Reordena um vetor usando nomes, sem assumir a ordem das juntas."""

    source = _unique_names(source_names, "Nomes de origem")
    target = _unique_names(target_names, "Nomes de destino")
    vector = _finite_vector(values, "Vetor a reordenar")
    if vector.size != len(source):
        raise NominalControlError(
            "Dimensao do vetor difere da quantidade de nomes de origem."
        )
    if set(source) != set(target):
        missing = sorted(set(target) - set(source))
        extra = sorted(set(source) - set(target))
        details = []
        if missing:
            details.append("ausentes=" + ",".join(missing))
        if extra:
            details.append("extras=" + ",".join(extra))
        raise NominalControlError(
            "Conjuntos de juntas incompativeis: " + "; ".join(details)
        )
    source_index = {name: index for index, name in enumerate(source)}
    return tuple(float(vector[source_index[name]]) for name in target)


def limit_vector_norm(
    values: Sequence[float],
    max_norm: float,
) -> tuple[np.ndarray, bool]:
    """Limita a norma euclidiana preservando a direcao do vetor."""

    vector = _finite_vector(values, "Vetor")
    if not math.isfinite(max_norm) or max_norm <= 0.0:
        raise NominalControlError("Limite de norma deve ser finito e positivo.")
    norm = float(np.linalg.norm(vector))
    if norm <= max_norm:
        return vector, False
    return vector * (max_norm / norm), True


def damped_least_squares(
    jacobian: Sequence[Sequence[float]],
    task_velocity: Sequence[float],
    damping: float,
) -> np.ndarray:
    """Resolve J*qdot=v pela inversa de minimo quadrado amortecida."""

    matrix = np.asarray(jacobian, dtype=float)
    velocity = _finite_vector(task_velocity, "Velocidade da tarefa")
    if matrix.ndim != 2 or matrix.shape[0] == 0 or matrix.shape[1] == 0:
        raise NominalControlError("Jacobiano deve ser uma matriz bidimensional.")
    if matrix.shape[0] != velocity.size:
        raise NominalControlError(
            "Numero de linhas do Jacobiano difere da dimensao da tarefa."
        )
    if not np.all(np.isfinite(matrix)):
        raise NominalControlError("Jacobiano contem NaN ou infinito.")
    if not math.isfinite(damping) or damping <= 0.0:
        raise NominalControlError("Amortecimento deve ser finito e positivo.")

    regularized = matrix @ matrix.T + (damping**2) * np.eye(matrix.shape[0])
    try:
        solution = matrix.T @ np.linalg.solve(regularized, velocity)
    except np.linalg.LinAlgError as error:
        raise NominalControlError(
            f"Falha ao resolver a inversa amortecida: {error}"
        ) from error
    if not np.all(np.isfinite(solution)):
        raise NominalControlError("Solucao articular contem NaN ou infinito.")
    return solution.reshape(-1)


def saturate_joint_velocity(
    values: Sequence[float],
    max_abs_velocity: float | Sequence[float],
) -> tuple[np.ndarray, bool]:
    """Aplica limites simetricos escalares ou individuais por junta."""

    vector = _finite_vector(values, "Velocidade articular")
    limits = np.asarray(max_abs_velocity, dtype=float).reshape(-1)
    if limits.size == 1:
        limits = np.full(vector.size, float(limits[0]))
    if limits.size != vector.size:
        raise NominalControlError(
            "Quantidade de limites difere da dimensao da velocidade articular."
        )
    if not np.all(np.isfinite(limits)) or np.any(limits <= 0.0):
        raise NominalControlError(
            "Limites articulares devem ser finitos e positivos."
        )
    saturated = np.clip(vector, -limits, limits)
    changed = not np.allclose(saturated, vector, rtol=0.0, atol=1e-15)
    return saturated, changed


def compute_position_control(
    *,
    error: Sequence[float],
    translational_jacobian: Sequence[Sequence[float]],
    model_joint_names: Sequence[str],
    controller_joint_names: Sequence[str],
    gains: float | Sequence[float],
    damping: float,
    max_cartesian_speed: float,
    max_abs_joint_velocity: float | Sequence[float],
) -> PositionControlResult:
    """Calcula qdot nominal e o organiza na ordem exigida pelo controlador."""

    error_vector = _finite_vector(error, "Erro cartesiano")
    model_names = _unique_names(model_joint_names, "Juntas do modelo")
    controller_names = _unique_names(
        controller_joint_names,
        "Juntas do controlador",
    )
    if set(model_names) != set(controller_names):
        reorder_vector(
            np.zeros(len(model_names)),
            model_names,
            controller_names,
        )

    gain_vector = np.asarray(gains, dtype=float).reshape(-1)
    if gain_vector.size == 1:
        gain_vector = np.full(error_vector.size, float(gain_vector[0]))
    if gain_vector.size != error_vector.size:
        raise NominalControlError(
            "Quantidade de ganhos difere da dimensao do erro cartesiano."
        )
    if not np.all(np.isfinite(gain_vector)) or np.any(gain_vector <= 0.0):
        raise NominalControlError("Ganhos devem ser finitos e positivos.")

    desired_cartesian = gain_vector * error_vector
    cartesian_velocity, cartesian_saturated = limit_vector_norm(
        desired_cartesian,
        max_cartesian_speed,
    )
    model_velocity = damped_least_squares(
        translational_jacobian,
        cartesian_velocity,
        damping,
    )
    if model_velocity.size != len(model_names):
        raise NominalControlError(
            "Numero de colunas do Jacobiano difere da quantidade de juntas do modelo."
        )
    model_velocity, joint_saturated = saturate_joint_velocity(
        model_velocity,
        max_abs_joint_velocity,
    )
    controller_velocity = reorder_vector(
        model_velocity,
        model_names,
        controller_names,
    )
    return PositionControlResult(
        controller_velocity=controller_velocity,
        model_velocity=tuple(float(value) for value in model_velocity),
        cartesian_velocity=tuple(float(value) for value in cartesian_velocity),
        error=tuple(float(value) for value in error_vector),
        error_norm=float(np.linalg.norm(error_vector)),
        cartesian_saturated=cartesian_saturated,
        joint_saturated=joint_saturated,
    )
