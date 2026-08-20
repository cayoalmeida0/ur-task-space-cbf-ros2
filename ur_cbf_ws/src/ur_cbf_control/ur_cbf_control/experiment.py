"""Temporizacao e persistencia reprodutivel dos ensaios de controle."""

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import tempfile
from typing import Any, Mapping


class ExperimentDataError(ValueError):
    """Indica tempo invalido ou resultado experimental nao serializavel."""


@dataclass(frozen=True)
class ControlTiming:
    """Tempos decorridos e estado dos dois limites independentes."""

    simulated_seconds: float
    wall_seconds: float
    simulated_limit_reached: bool
    wall_limit_reached: bool


def _finite(value: float, label: str) -> float:
    normalized = float(value)
    if not math.isfinite(normalized):
        raise ExperimentDataError(f"{label} deve ser finito.")
    return normalized


def evaluate_control_timing(
    *,
    start_simulated: float,
    current_simulated: float,
    start_wall: float,
    current_wall: float,
    max_simulated: float,
    max_wall: float,
) -> ControlTiming:
    """Compara duracao dinamica simulada e limite absoluto de tempo real."""

    start_simulated = _finite(start_simulated, "Tempo simulado inicial")
    current_simulated = _finite(current_simulated, "Tempo simulado atual")
    start_wall = _finite(start_wall, "Tempo real inicial")
    current_wall = _finite(current_wall, "Tempo real atual")
    max_simulated = _finite(max_simulated, "Limite simulado")
    max_wall = _finite(max_wall, "Limite real")
    if max_simulated <= 0.0 or max_wall <= 0.0:
        raise ExperimentDataError("Limites de tempo devem ser positivos.")

    simulated_seconds = current_simulated - start_simulated
    wall_seconds = current_wall - start_wall
    tolerance = 1e-9
    if simulated_seconds < -tolerance:
        raise ExperimentDataError("Relogio simulado retrocedeu.")
    if wall_seconds < -tolerance:
        raise ExperimentDataError("Relogio monotonicamente crescente retrocedeu.")
    simulated_seconds = max(0.0, simulated_seconds)
    wall_seconds = max(0.0, wall_seconds)
    return ControlTiming(
        simulated_seconds=simulated_seconds,
        wall_seconds=wall_seconds,
        simulated_limit_reached=simulated_seconds >= max_simulated,
        wall_limit_reached=wall_seconds >= max_wall,
    )


def _safe_experiment_id(experiment_id: str) -> str:
    normalized = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(experiment_id)).strip(
        "_.-"
    )
    if not normalized:
        raise ExperimentDataError("experiment_id nao produz um nome de arquivo valido.")
    return normalized


def write_experiment_record(
    *,
    record: Mapping[str, Any],
    directory: str | Path,
    experiment_id: str,
    recorded_at: datetime | None = None,
) -> Path:
    """Grava um resultado JSON de forma atomica e rejeita NaN ou infinito."""

    destination = Path(directory).expanduser()
    if not str(destination):
        raise ExperimentDataError("Diretorio de resultados esta vazio.")
    timestamp = recorded_at or datetime.now(timezone.utc)
    if timestamp.tzinfo is None:
        raise ExperimentDataError("Timestamp do experimento deve possuir fuso horario.")
    timestamp = timestamp.astimezone(timezone.utc)
    payload = dict(record)
    payload["recorded_at_utc"] = timestamp.isoformat().replace("+00:00", "Z")
    try:
        serialized = json.dumps(
            payload,
            allow_nan=False,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
    except (TypeError, ValueError) as error:
        raise ExperimentDataError(
            f"Resultado experimental nao e JSON valido: {error}"
        ) from error

    destination.mkdir(parents=True, exist_ok=True)
    stamp = timestamp.strftime("%Y%m%dT%H%M%S.%fZ")
    output = destination / f"{_safe_experiment_id(experiment_id)}_{stamp}.json"
    temporary_name = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=destination,
            prefix=".ur_cbf_result_",
            suffix=".tmp",
            delete=False,
        ) as temporary:
            temporary_name = temporary.name
            temporary.write(serialized)
            temporary.write("\n")
            temporary.flush()
            os.fsync(temporary.fileno())
        os.replace(temporary_name, output)
    except OSError as error:
        if temporary_name is not None:
            try:
                Path(temporary_name).unlink(missing_ok=True)
            except OSError:
                pass
        raise ExperimentDataError(
            f"Falha ao gravar resultado experimental: {error}"
        ) from error
    return output
