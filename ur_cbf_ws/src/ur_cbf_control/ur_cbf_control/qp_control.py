"""Nucleo QP do controlador cartesiano com limites e restricoes CBF."""

from dataclasses import dataclass
import math
from typing import Any, Callable, Sequence

import numpy as np
import osqp
from scipy import sparse

from ur_cbf_control.nominal_control import limit_vector_norm
from ur_cbf_control.nominal_control import NominalControlError
from ur_cbf_control.nominal_control import reorder_vector


class QpControlError(RuntimeError):
    """Indica entrada invalida, falha numerica ou insucesso do resolvedor."""


@dataclass(frozen=True)
class QpDiagnostics:
    """Diagnostico validado de uma solucao OSQP."""

    status: str
    status_value: int
    iterations: int
    objective: float
    primal_residual: float
    dual_residual: float
    setup_time: float
    solve_time: float
    update_time: float
    polish_time: float
    run_time: float
    rho_updates: int
    active_lower: tuple[int, ...]
    active_upper: tuple[int, ...]
    active_cbf: tuple[int, ...]
    max_bound_violation: float
    max_cbf_violation: float
    reused_workspace: bool

    @property
    def constraint_active(self) -> bool:
        """Indica ativacao de algum limite articular."""

        return bool(self.active_lower or self.active_upper)

    @property
    def cbf_constraint_active(self) -> bool:
        """Indica ativacao de alguma desigualdade CBF."""

        return bool(self.active_cbf)

    def to_record(self) -> dict[str, object]:
        """Converte o diagnostico para tipos JSON estritos."""

        return {
            "status": self.status,
            "status_value": self.status_value,
            "iterations": self.iterations,
            "objective": self.objective,
            "primal_residual": self.primal_residual,
            "dual_residual": self.dual_residual,
            "setup_time": self.setup_time,
            "solve_time": self.solve_time,
            "update_time": self.update_time,
            "polish_time": self.polish_time,
            "run_time": self.run_time,
            "rho_updates": self.rho_updates,
            "active_lower": list(self.active_lower),
            "active_upper": list(self.active_upper),
            "active_cbf": list(self.active_cbf),
            "max_bound_violation": self.max_bound_violation,
            "max_cbf_violation": self.max_cbf_violation,
            "reused_workspace": self.reused_workspace,
        }


@dataclass(frozen=True)
class QpPositionControlResult:
    """Comando cartesiano QP e telemetria de uma iteracao."""

    controller_velocity: tuple[float, ...]
    model_velocity: tuple[float, ...]
    cartesian_velocity: tuple[float, ...]
    error: tuple[float, ...]
    error_norm: float
    cartesian_saturated: bool
    joint_constraint_active: bool
    cbf_constraint_active: bool
    diagnostics: QpDiagnostics


def _finite_vector(values: Sequence[float], label: str) -> np.ndarray:
    vector = np.asarray(values, dtype=float).reshape(-1)
    if vector.size == 0:
        raise QpControlError(f"{label} esta vazio.")
    if not np.all(np.isfinite(vector)):
        raise QpControlError(f"{label} contem NaN ou infinito.")
    return vector


def _finite_matrix(values: Sequence[Sequence[float]], label: str) -> np.ndarray:
    matrix = np.asarray(values, dtype=float)
    if matrix.ndim != 2 or matrix.shape[0] == 0 or matrix.shape[1] == 0:
        raise QpControlError(f"{label} deve ser uma matriz bidimensional.")
    if not np.all(np.isfinite(matrix)):
        raise QpControlError(f"{label} contem NaN ou infinito.")
    return matrix


def _positive_limits(
    values: float | Sequence[float],
    dimension: int,
) -> np.ndarray:
    limits = np.asarray(values, dtype=float).reshape(-1)
    if limits.size == 1:
        limits = np.full(dimension, float(limits[0]))
    if limits.size != dimension:
        raise QpControlError(
            "Quantidade de limites difere da dimensao da velocidade articular."
        )
    if not np.all(np.isfinite(limits)) or np.any(limits <= 0.0):
        raise QpControlError(
            "Limites articulares devem ser finitos e positivos."
        )
    return limits


def _finite_info(info: Any, name: str, default: float = 0.0) -> float:
    value = float(getattr(info, name, default))
    if not math.isfinite(value):
        raise QpControlError(f"OSQP retornou {name} nao finito.")
    return value


class BoxConstrainedQpSolver:
    """Resolve o QP cinetico e reutiliza a fatoracao entre iteracoes."""

    def __init__(
        self,
        *,
        absolute_tolerance: float = 1e-6,
        relative_tolerance: float = 1e-6,
        max_iterations: int = 4000,
        time_limit: float = 0.01,
        polishing: bool = False,
        solver_factory: Callable[[], Any] | None = None,
    ) -> None:
        positive_values = {
            "absolute_tolerance": absolute_tolerance,
            "relative_tolerance": relative_tolerance,
            "time_limit": time_limit,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise QpControlError(f"{name} deve ser finito e positivo.")
        if max_iterations <= 0:
            raise QpControlError("max_iterations deve ser positivo.")

        self.absolute_tolerance = float(absolute_tolerance)
        self.relative_tolerance = float(relative_tolerance)
        self.max_iterations = int(max_iterations)
        self.time_limit = float(time_limit)
        self.polishing = bool(polishing)
        self._solver_factory = solver_factory or osqp.OSQP
        self._solver: Any | None = None
        self._dimension: int | None = None
        self._cbf_count: int | None = None

    @property
    def solver_version(self) -> str:
        return str(getattr(osqp, "__version__", "unknown"))

    @staticmethod
    def _upper_triangular(matrix: np.ndarray) -> sparse.csc_matrix:
        rows, columns = np.triu_indices(matrix.shape[0])
        result = sparse.csc_matrix(
            (matrix[rows, columns], (rows, columns)),
            shape=matrix.shape,
        )
        expected_entries = matrix.shape[0] * (matrix.shape[0] + 1) // 2
        if result.nnz != expected_entries:
            raise QpControlError("Padrao esparso superior do QP foi alterado.")
        return result

    def _setup_or_update(
        self,
        *,
        quadratic: sparse.csc_matrix,
        linear: np.ndarray,
        constraint_matrix: sparse.csc_matrix,
        lower: np.ndarray,
        upper: np.ndarray,
        cbf_count: int,
    ) -> bool:
        dimension = linear.size
        reused = (
            self._solver is not None
            and self._dimension == dimension
            and self._cbf_count == cbf_count
        )
        if not reused:
            self._solver = self._solver_factory()
            self._solver.setup(
                P=quadratic,
                q=linear,
                A=constraint_matrix,
                l=lower,
                u=upper,
                verbose=False,
                eps_abs=self.absolute_tolerance,
                eps_rel=self.relative_tolerance,
                max_iter=self.max_iterations,
                time_limit=self.time_limit,
                polishing=self.polishing,
                warm_starting=True,
            )
            self._dimension = dimension
            self._cbf_count = cbf_count
            return False

        self._solver.update(
            Px=quadratic.data,
            Ax=constraint_matrix.data,
            q=linear,
            l=lower,
            u=upper,
        )
        return True

    @staticmethod
    def _stack_constraint_matrix(
        dimension: int,
        cbf_matrix: np.ndarray,
    ) -> sparse.csc_matrix:
        """Cria um padrao CSC fixo para limites e linhas CBF densas."""

        cbf_count = cbf_matrix.shape[0]
        rows = np.concatenate(
            (
                np.arange(dimension),
                dimension + np.repeat(np.arange(cbf_count), dimension),
            )
        )
        columns = np.concatenate(
            (
                np.arange(dimension),
                np.tile(np.arange(dimension), cbf_count),
            )
        )
        values = np.concatenate((np.ones(dimension), cbf_matrix.reshape(-1)))
        result = sparse.csc_matrix(
            (values, (rows, columns)),
            shape=(dimension + cbf_count, dimension),
        )
        expected_entries = dimension * (1 + cbf_count)
        if result.nnz != expected_entries:
            raise QpControlError("Padrao esparso das restricoes foi alterado.")
        return result

    def solve(
        self,
        *,
        jacobian: Sequence[Sequence[float]],
        task_velocity: Sequence[float],
        damping: float,
        max_abs_joint_velocity: float | Sequence[float],
        cbf_matrix: Sequence[Sequence[float]] | None = None,
        cbf_lower_bound: Sequence[float] | None = None,
    ) -> tuple[np.ndarray, QpDiagnostics]:
        """Minimiza erro cartesiano sujeito a limites e ``C qdot >= b``."""

        matrix = _finite_matrix(jacobian, "Jacobiano")
        velocity = _finite_vector(task_velocity, "Velocidade da tarefa")
        if matrix.shape[0] != velocity.size:
            raise QpControlError(
                "Numero de linhas do Jacobiano difere da dimensao da tarefa."
            )
        if not math.isfinite(damping) or damping <= 0.0:
            raise QpControlError("Amortecimento deve ser finito e positivo.")

        dimension = matrix.shape[1]
        limits = _positive_limits(max_abs_joint_velocity, dimension)
        if cbf_matrix is None and cbf_lower_bound is None:
            cbf = np.empty((0, dimension), dtype=float)
            cbf_lower = np.empty(0, dtype=float)
        elif cbf_matrix is None or cbf_lower_bound is None:
            raise QpControlError(
                "Matriz e limite inferior CBF devem ser fornecidos juntos."
            )
        else:
            cbf = np.asarray(cbf_matrix, dtype=float)
            cbf_lower = np.asarray(cbf_lower_bound, dtype=float).reshape(-1)
            if cbf.ndim != 2 or cbf.shape[1] != dimension:
                raise QpControlError(
                    "Matriz CBF deve ter uma coluna por velocidade articular."
                )
            if cbf.shape[0] != cbf_lower.size:
                raise QpControlError(
                    "Quantidade de limites CBF difere das linhas da matriz."
                )
            if not np.all(np.isfinite(cbf)) or not np.all(np.isfinite(cbf_lower)):
                raise QpControlError("Restricoes CBF contem NaN ou infinito.")

        lower = np.concatenate((-limits, cbf_lower))
        upper = np.concatenate(
            (limits, np.full(cbf_lower.size, np.inf, dtype=float))
        )
        constraint_matrix = self._stack_constraint_matrix(dimension, cbf)
        hessian = matrix.T @ matrix + (damping**2) * np.eye(dimension)
        linear = -(matrix.T @ velocity)
        quadratic = self._upper_triangular(hessian)

        try:
            reused = self._setup_or_update(
                quadratic=quadratic,
                linear=linear,
                constraint_matrix=constraint_matrix,
                lower=lower,
                upper=upper,
                cbf_count=cbf.shape[0],
            )
            result = self._solver.solve(raise_error=False)
        except Exception as error:
            raise QpControlError(f"Falha ao executar OSQP: {error}") from error

        info = result.info
        status = str(getattr(info, "status", "unknown"))
        status_value = int(getattr(info, "status_val", -1))
        if status_value not in (1, 2):
            raise QpControlError(
                f"OSQP nao encontrou solucao utilizavel: {status} "
                f"(codigo {status_value})."
            )
        solution = np.asarray(result.x, dtype=float).reshape(-1)
        if solution.size != dimension or not np.all(np.isfinite(solution)):
            raise QpControlError("OSQP retornou solucao articular invalida.")

        constraint_values = np.asarray(constraint_matrix @ solution).reshape(-1)
        lower_violation = np.maximum(lower - constraint_values, 0.0)
        upper_violation = np.maximum(constraint_values - upper, 0.0)
        max_bound_violation = float(
            max(
                np.max(lower_violation[:dimension]),
                np.max(upper_violation[:dimension]),
            )
        )
        max_constraint_violation = float(
            max(np.max(lower_violation), np.max(upper_violation))
        )
        feasibility_tolerance = max(
            10.0 * self.absolute_tolerance,
            10.0
            * self.relative_tolerance
            * max(float(np.max(limits)), float(np.max(np.abs(lower))), 1.0),
        )
        if max_constraint_violation > feasibility_tolerance:
            raise QpControlError(
                "OSQP violou alguma restricao acima da tolerancia: "
                f"{max_constraint_violation:.3e}."
            )
        solution = np.clip(solution, -limits, limits)
        active_tolerance = max(feasibility_tolerance, 1e-8)
        active_lower = tuple(
            int(index)
            for index in np.flatnonzero(
                np.abs(solution + limits) <= active_tolerance
            )
        )
        active_upper = tuple(
            int(index)
            for index in np.flatnonzero(
                np.abs(solution - limits) <= active_tolerance
            )
        )
        if cbf.shape[0]:
            cbf_values = np.asarray(cbf @ solution).reshape(-1)
            cbf_violation = np.maximum(cbf_lower - cbf_values, 0.0)
            max_cbf_violation = float(np.max(cbf_violation))
            active_cbf = tuple(
                int(index)
                for index in np.flatnonzero(
                    np.abs(cbf_values - cbf_lower) <= active_tolerance
                )
            )
        else:
            max_cbf_violation = 0.0
            active_cbf = ()
        if max_cbf_violation > feasibility_tolerance:
            raise QpControlError(
                "A projecao numerica violou alguma restricao CBF acima da "
                f"tolerancia: {max_cbf_violation:.3e}."
            )
        diagnostics = QpDiagnostics(
            status=status,
            status_value=status_value,
            iterations=int(getattr(info, "iter", 0)),
            objective=_finite_info(info, "obj_val"),
            primal_residual=_finite_info(info, "prim_res"),
            dual_residual=_finite_info(info, "dual_res"),
            setup_time=_finite_info(info, "setup_time"),
            solve_time=_finite_info(info, "solve_time"),
            update_time=_finite_info(info, "update_time"),
            polish_time=_finite_info(info, "polish_time"),
            run_time=_finite_info(info, "run_time"),
            rho_updates=int(getattr(info, "rho_updates", 0)),
            active_lower=active_lower,
            active_upper=active_upper,
            active_cbf=active_cbf,
            max_bound_violation=max_bound_violation,
            max_cbf_violation=max_cbf_violation,
            reused_workspace=reused,
        )
        return solution, diagnostics


def compute_qp_position_control(
    *,
    error: Sequence[float],
    translational_jacobian: Sequence[Sequence[float]],
    model_joint_names: Sequence[str],
    controller_joint_names: Sequence[str],
    gains: float | Sequence[float],
    damping: float,
    max_cartesian_speed: float,
    max_abs_joint_velocity: float | Sequence[float],
    solver: BoxConstrainedQpSolver,
    cbf_matrix: Sequence[Sequence[float]] | None = None,
    cbf_lower_bound: Sequence[float] | None = None,
) -> QpPositionControlResult:
    """Calcula o comando QP e o organiza na ordem do controlador ROS."""

    error_vector = _finite_vector(error, "Erro cartesiano")
    gain_vector = np.asarray(gains, dtype=float).reshape(-1)
    if gain_vector.size == 1:
        gain_vector = np.full(error_vector.size, float(gain_vector[0]))
    if gain_vector.size != error_vector.size:
        raise QpControlError(
            "Quantidade de ganhos difere da dimensao do erro cartesiano."
        )
    if not np.all(np.isfinite(gain_vector)) or np.any(gain_vector <= 0.0):
        raise QpControlError("Ganhos devem ser finitos e positivos.")

    desired_cartesian = gain_vector * error_vector
    try:
        cartesian_velocity, cartesian_saturated = limit_vector_norm(
            desired_cartesian,
            max_cartesian_speed,
        )
        model_velocity, diagnostics = solver.solve(
            jacobian=translational_jacobian,
            task_velocity=cartesian_velocity,
            damping=damping,
            max_abs_joint_velocity=max_abs_joint_velocity,
            cbf_matrix=cbf_matrix,
            cbf_lower_bound=cbf_lower_bound,
        )
        if model_velocity.size != len(model_joint_names):
            raise QpControlError(
                "Numero de colunas do Jacobiano difere da quantidade de juntas "
                "do modelo."
            )
        controller_velocity = reorder_vector(
            model_velocity,
            model_joint_names,
            controller_joint_names,
        )
    except NominalControlError as error:
        raise QpControlError(str(error)) from error

    return QpPositionControlResult(
        controller_velocity=controller_velocity,
        model_velocity=tuple(float(value) for value in model_velocity),
        cartesian_velocity=tuple(float(value) for value in cartesian_velocity),
        error=tuple(float(value) for value in error_vector),
        error_norm=float(np.linalg.norm(error_vector)),
        cartesian_saturated=cartesian_saturated,
        joint_constraint_active=diagnostics.constraint_active,
        cbf_constraint_active=diagnostics.cbf_constraint_active,
        diagnostics=diagnostics,
    )
