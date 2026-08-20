"""Funcoes puras usadas para validar estados e construir comandos articulares."""

from dataclasses import dataclass
import math
from typing import Sequence


class JointStateValidationError(ValueError):
    """Indica que uma mensagem de estado nao pode ser usada com seguranca."""


@dataclass(frozen=True)
class OrderedJointState:
    """Estado articular organizado na ordem exigida pelo controlador."""

    names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]


@dataclass(frozen=True)
class VelocityCommand:
    """Comando validado e limitado antes da publicacao."""

    values: tuple[float, ...]
    applied_velocity: float
    saturated: bool


def _unique_names(names: Sequence[str], label: str) -> tuple[str, ...]:
    normalized = tuple(str(name) for name in names)
    if not normalized:
        raise JointStateValidationError(f"{label} esta vazio.")
    if any(not name for name in normalized):
        raise JointStateValidationError(f"{label} contem nome vazio.")
    if len(set(normalized)) != len(normalized):
        raise JointStateValidationError(f"{label} contem nomes duplicados.")
    return normalized


def reorder_joint_state(
    controller_joints: Sequence[str],
    message_names: Sequence[str],
    positions: Sequence[float],
    velocities: Sequence[float],
) -> OrderedJointState:
    """Reordena ``JointState`` sem depender da ordem recebida no topico."""

    ordered_names = _unique_names(controller_joints, "Ordem do controlador")
    received_names = _unique_names(message_names, "Nomes de /joint_states")

    if len(received_names) != len(positions):
        raise JointStateValidationError(
            "Quantidade de posicoes difere da quantidade de nomes."
        )
    if len(received_names) != len(velocities):
        raise JointStateValidationError(
            "Quantidade de velocidades difere da quantidade de nomes."
        )

    index_by_name = {name: index for index, name in enumerate(received_names)}
    missing = [name for name in ordered_names if name not in index_by_name]
    if missing:
        raise JointStateValidationError(
            "Juntas ausentes em /joint_states: " + ", ".join(missing)
        )

    ordered_positions = tuple(
        float(positions[index_by_name[name]]) for name in ordered_names
    )
    ordered_velocities = tuple(
        float(velocities[index_by_name[name]]) for name in ordered_names
    )
    values = ordered_positions + ordered_velocities
    if not all(math.isfinite(value) for value in values):
        raise JointStateValidationError("Estado articular contem NaN ou infinito.")

    return OrderedJointState(
        names=ordered_names,
        positions=ordered_positions,
        velocities=ordered_velocities,
    )


def build_velocity_command(
    controller_joints: Sequence[str],
    target_joint: str,
    requested_velocity: float,
    max_abs_velocity: float,
) -> VelocityCommand:
    """Cria um pulso monoarticular limitado e na ordem do controlador."""

    ordered_names = _unique_names(controller_joints, "Ordem do controlador")
    if target_joint not in ordered_names:
        raise ValueError(f"Junta alvo desconhecida: {target_joint!r}.")
    if not math.isfinite(requested_velocity):
        raise ValueError("Velocidade solicitada deve ser finita.")
    if not math.isfinite(max_abs_velocity) or max_abs_velocity <= 0.0:
        raise ValueError("Limite de velocidade deve ser finito e positivo.")

    applied_velocity = max(
        -max_abs_velocity,
        min(max_abs_velocity, float(requested_velocity)),
    )
    values = [0.0] * len(ordered_names)
    values[ordered_names.index(target_joint)] = applied_velocity
    return VelocityCommand(
        values=tuple(values),
        applied_velocity=applied_velocity,
        saturated=not math.isclose(applied_velocity, requested_velocity),
    )


def zero_velocity_command(controller_joints: Sequence[str]) -> tuple[float, ...]:
    """Cria um comando nulo com dimensao derivada da configuracao."""

    ordered_names = _unique_names(controller_joints, "Ordem do controlador")
    return (0.0,) * len(ordered_names)


def is_state_stale(
    receipt_monotonic: float,
    now_monotonic: float,
    timeout: float,
) -> bool:
    """Verifica obsolescencia usando relogio monotonicamente crescente."""

    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError("Timeout deve ser finito e positivo.")
    if now_monotonic < receipt_monotonic:
        return True
    return now_monotonic - receipt_monotonic > timeout
