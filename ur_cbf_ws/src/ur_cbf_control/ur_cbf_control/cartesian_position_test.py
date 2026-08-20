"""Ensaio protegido de regulacao cartesiana nominal de posicao."""

from enum import Enum, auto
import math
import sys
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from ur_cbf_control.kinematics import KinematicState
from ur_cbf_control.kinematics import KinematicsError
from ur_cbf_control.kinematics import UaibotKinematics
from ur_cbf_control.nominal_control import compute_position_control
from ur_cbf_control.nominal_control import NominalControlError
from ur_cbf_control.nominal_control import reorder_vector
from ur_cbf_control.safety import JointStateValidationError
from ur_cbf_control.safety import OrderedJointState
from ur_cbf_control.safety import is_state_stale
from ur_cbf_control.safety import reorder_joint_state
from ur_cbf_control.safety import zero_velocity_command


class Phase(Enum):
    WAITING = auto()
    SETTLING = auto()
    CONTROLLING = auto()
    HOLDING = auto()
    STOPPING = auto()


class CartesianPositionTest(Node):
    """Regula a posicao do efetuador e garante parada em qualquer falha."""

    def __init__(self) -> None:
        super().__init__("cartesian_position_test")
        self.declare_parameter("execute_test", False)
        self.declare_parameter("controller_node", "/forward_velocity_controller")
        self.declare_parameter(
            "command_topic",
            "/forward_velocity_controller/commands",
        )
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("required_simulation_node", "/gz_ros_control")
        self.declare_parameter("ur_type", "ur3e")
        self.declare_parameter(
            "model_joint_names",
            Parameter.Type.STRING_ARRAY,
        )
        self.declare_parameter("uaibot_mode", "auto")
        self.declare_parameter("eef_offset_xyz", [0.0, 0.0, 0.2])
        self.declare_parameter("target_offset", [0.0, 0.0, 0.01])
        self.declare_parameter("position_gains", [1.0, 1.0, 1.0])
        self.declare_parameter("damping", 0.05)
        self.declare_parameter("max_cartesian_speed", 0.01)
        self.declare_parameter("max_abs_joint_velocity", 0.10)
        self.declare_parameter("position_tolerance", 0.001)
        self.declare_parameter("settle_duration", 0.5)
        self.declare_parameter("success_hold_duration", 0.5)
        self.declare_parameter("zero_hold_duration", 0.5)
        self.declare_parameter("max_control_duration", 8.0)
        self.declare_parameter("state_timeout", 0.25)
        self.declare_parameter("startup_timeout", 15.0)
        self.declare_parameter("command_rate", 50.0)
        self.declare_parameter("experiment_id", "cartesian_position_ur3e_001")
        self.declare_parameter("random_seed", 0)

        self.execute_test = bool(self.get_parameter("execute_test").value)
        self.controller_node = str(self.get_parameter("controller_node").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.joint_states_topic = str(
            self.get_parameter("joint_states_topic").value
        )
        self.required_simulation_node = str(
            self.get_parameter("required_simulation_node").value
        )
        self.ur_type = str(self.get_parameter("ur_type").value)
        model_names_value = self.get_parameter("model_joint_names").value
        self.model_joint_names = tuple(model_names_value or ())
        self.uaibot_mode = str(self.get_parameter("uaibot_mode").value)
        self.eef_offset_xyz = tuple(
            float(value) for value in self.get_parameter("eef_offset_xyz").value
        )
        self.target_offset = np.asarray(
            self.get_parameter("target_offset").value,
            dtype=float,
        ).reshape(-1)
        self.position_gains = tuple(
            float(value) for value in self.get_parameter("position_gains").value
        )
        self.damping = float(self.get_parameter("damping").value)
        self.max_cartesian_speed = float(
            self.get_parameter("max_cartesian_speed").value
        )
        self.max_abs_joint_velocity = float(
            self.get_parameter("max_abs_joint_velocity").value
        )
        self.position_tolerance = float(
            self.get_parameter("position_tolerance").value
        )
        self.settle_duration = float(self.get_parameter("settle_duration").value)
        self.success_hold_duration = float(
            self.get_parameter("success_hold_duration").value
        )
        self.zero_hold_duration = float(
            self.get_parameter("zero_hold_duration").value
        )
        self.max_control_duration = float(
            self.get_parameter("max_control_duration").value
        )
        self.state_timeout = float(self.get_parameter("state_timeout").value)
        self.startup_timeout = float(self.get_parameter("startup_timeout").value)
        self.command_rate = float(self.get_parameter("command_rate").value)
        self.experiment_id = str(self.get_parameter("experiment_id").value)
        self.random_seed = int(self.get_parameter("random_seed").value)

        self._validate_parameters()
        self.kinematics = UaibotKinematics.create(
            ur_type=self.ur_type,
            model_joint_names=self.model_joint_names,
            eef_offset_xyz=self.eef_offset_xyz,
            mode=self.uaibot_mode,
        )

        self.finished = False
        self.exit_code = 1
        self.phase = Phase.WAITING
        self.start_monotonic = time.monotonic()
        self.phase_start_monotonic = self.start_monotonic
        self.controller_joints: tuple[str, ...] | None = None
        self.latest_state: OrderedJointState | None = None
        self.latest_state_receipt: float | None = None
        self.initial_position: np.ndarray | None = None
        self.target_position: np.ndarray | None = None
        self.pending_failure: str | None = None
        self._joint_parameter_future = None
        self._last_state_error: str | None = None
        self._last_progress_log = self.start_monotonic
        self._maximum_command = 0.0

        command_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            self.command_topic,
            command_qos,
        )
        self.create_subscription(
            JointState,
            self.joint_states_topic,
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        controller_name = "/" + self.controller_node.strip("/")
        self.parameter_client = self.create_client(
            GetParameters,
            f"{controller_name}/get_parameters",
        )
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            1.0 / self.command_rate,
            self._tick,
            clock=steady_clock,
        )

        if not self.execute_test:
            self.get_logger().error(
                "Teste desarmado. Execute com execute_test:=true no Gazebo."
            )
            self.finished = True
            self.exit_code = 2
        else:
            self.get_logger().info(
                f"Ensaio {self.experiment_id} armado; ur_type={self.ur_type}; "
                f"seed={self.random_seed}; imagem esperada=ur-cbf-jazzy:0.1.7."
            )

    def _validate_parameters(self) -> None:
        positive_values = {
            "damping": self.damping,
            "max_cartesian_speed": self.max_cartesian_speed,
            "max_abs_joint_velocity": self.max_abs_joint_velocity,
            "position_tolerance": self.position_tolerance,
            "settle_duration": self.settle_duration,
            "success_hold_duration": self.success_hold_duration,
            "zero_hold_duration": self.zero_hold_duration,
            "max_control_duration": self.max_control_duration,
            "state_timeout": self.state_timeout,
            "startup_timeout": self.startup_timeout,
            "command_rate": self.command_rate,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} deve ser finito e positivo.")
        if len(self.model_joint_names) == 0:
            raise ValueError("model_joint_names deve ser configurado explicitamente.")
        if len(set(self.model_joint_names)) != len(self.model_joint_names):
            raise ValueError("model_joint_names contem nomes duplicados.")
        if len(self.eef_offset_xyz) != 3 or not all(
            math.isfinite(value) for value in self.eef_offset_xyz
        ):
            raise ValueError("eef_offset_xyz deve conter tres valores finitos.")
        if self.target_offset.size != 3 or not np.all(
            np.isfinite(self.target_offset)
        ):
            raise ValueError("target_offset deve conter tres valores finitos.")
        if len(self.position_gains) != 3 or not all(
            math.isfinite(value) and value > 0.0
            for value in self.position_gains
        ):
            raise ValueError("position_gains deve conter tres valores positivos.")
        if self.execute_test and np.linalg.norm(self.target_offset) <= (
            2.0 * self.position_tolerance
        ):
            raise ValueError("target_offset deve superar duas vezes a tolerancia.")

    def _request_joint_order(self) -> None:
        if self._joint_parameter_future is not None:
            if not self._joint_parameter_future.done():
                return
            try:
                response = self._joint_parameter_future.result()
            except Exception as error:  # pragma: no cover - depende do middleware
                self._finish_failure(
                    f"Falha ao consultar a ordem das juntas: {error}"
                )
                return
            self._joint_parameter_future = None
            if response is None or len(response.values) != 1:
                self._finish_failure("Resposta invalida do parametro joints.")
                return
            value = response.values[0]
            if value.type != ParameterType.PARAMETER_STRING_ARRAY:
                self._finish_failure("Parametro joints nao e uma lista de strings.")
                return
            self.controller_joints = tuple(value.string_array_value)
            try:
                reorder_vector(
                    np.zeros(len(self.model_joint_names)),
                    self.model_joint_names,
                    self.controller_joints,
                )
            except NominalControlError as error:
                self._finish_failure(str(error))
                return
            self.get_logger().info(
                "Ordem do controlador: " + ", ".join(self.controller_joints)
            )
            self.get_logger().info(
                "Ordem do modelo UAIbot: " + ", ".join(self.model_joint_names)
            )
            return

        if self.parameter_client.service_is_ready():
            request = GetParameters.Request()
            request.names = ["joints"]
            self._joint_parameter_future = self.parameter_client.call_async(request)

    def _joint_state_callback(self, message: JointState) -> None:
        if self.controller_joints is None:
            return
        try:
            ordered = reorder_joint_state(
                self.controller_joints,
                message.name,
                message.position,
                message.velocity,
            )
        except JointStateValidationError as error:
            reason = str(error)
            if reason != self._last_state_error:
                self.get_logger().error(reason)
                self._last_state_error = reason
            return
        self._last_state_error = None
        self.latest_state = ordered
        self.latest_state_receipt = time.monotonic()

    def _state_is_stale(self, now: float) -> bool:
        if self.latest_state_receipt is None:
            return True
        return is_state_stale(
            self.latest_state_receipt,
            now,
            self.state_timeout,
        )

    def _simulation_is_available(self) -> bool:
        required = "/" + self.required_simulation_node.strip("/")
        available = {
            (namespace.rstrip("/") + "/" + name).replace("//", "/")
            for name, namespace in self.get_node_names_and_namespaces()
        }
        return required in available

    def _publish(self, values: tuple[float, ...]) -> None:
        message = Float64MultiArray()
        message.data = list(values)
        self.command_publisher.publish(message)

    def _publish_zero(self) -> None:
        if self.controller_joints is not None:
            self._publish(zero_velocity_command(self.controller_joints))

    def _model_positions(self) -> tuple[float, ...]:
        if self.latest_state is None or self.controller_joints is None:
            raise NominalControlError("Estado articular ainda nao esta disponivel.")
        return reorder_vector(
            self.latest_state.positions,
            self.controller_joints,
            self.model_joint_names,
        )

    def _evaluate_kinematics(self) -> KinematicState:
        return self.kinematics.evaluate(self._model_positions())

    def request_abort(self, reason: str) -> None:
        if self.finished or self.phase is Phase.STOPPING:
            return
        self.pending_failure = reason
        self.phase = Phase.STOPPING
        self.phase_start_monotonic = time.monotonic()
        self.get_logger().error(reason)

    def _finish_failure(self, reason: str) -> None:
        self._publish_zero()
        self.get_logger().error(f"ENSAIO CARTESIANO REPROVADO: {reason}")
        self.finished = True
        self.exit_code = 1

    def _finish_success(self, state: KinematicState) -> None:
        self._publish_zero()
        error = self.target_position - np.asarray(state.position)
        self.get_logger().info(
            "ENSAIO CARTESIANO APROVADO: "
            f"erro_final={np.linalg.norm(error):.6f} m; "
            f"max_qdot={self._maximum_command:.6f} rad/s."
        )
        self.finished = True
        self.exit_code = 0

    def _tick(self) -> None:
        try:
            self._tick_impl()
        except (KinematicsError, NominalControlError, ValueError) as error:
            self.request_abort(f"Falha de controle: {error}")
        except Exception as error:  # pragma: no cover - protecao de ultima camada
            self.request_abort(f"Falha inesperada: {error}")

    def _tick_impl(self) -> None:
        if self.finished:
            return
        now = time.monotonic()
        if self.controller_joints is None:
            self._request_joint_order()
            if now - self.start_monotonic > self.startup_timeout:
                self._finish_failure("Timeout ao obter a ordem das juntas.")
            return
        if self.latest_state is None or self.command_publisher.get_subscription_count() < 1:
            self._publish_zero()
            if now - self.start_monotonic > self.startup_timeout:
                self._finish_failure("Timeout ao aguardar estado ou assinante.")
            return
        if not self._simulation_is_available():
            self._publish_zero()
            if now - self.start_monotonic > self.startup_timeout:
                self._finish_failure(
                    "No de simulacao ausente; o ensaio nao opera no robo real."
                )
            return
        if self.phase is not Phase.STOPPING and self._state_is_stale(now):
            self.request_abort("/joint_states ficou obsoleto; aplicando comando nulo.")

        if self.phase is Phase.WAITING:
            self.phase = Phase.SETTLING
            self.phase_start_monotonic = now
            self._publish_zero()
            self.get_logger().info("Estado valido recebido; estabilizando.")
            return

        if self.phase is Phase.SETTLING:
            self._publish_zero()
            if now - self.phase_start_monotonic >= self.settle_duration:
                state = self._evaluate_kinematics()
                self.initial_position = np.asarray(state.position)
                self.target_position = self.initial_position + self.target_offset
                self.phase = Phase.CONTROLLING
                self.phase_start_monotonic = now
                self.get_logger().info(
                    "Posicao inicial="
                    f"{self.initial_position.tolist()}; alvo={self.target_position.tolist()}; "
                    f"offset={self.target_offset.tolist()}."
                )
            return

        if self.phase is Phase.CONTROLLING:
            if now - self.phase_start_monotonic > self.max_control_duration:
                self.request_abort("Tempo maximo de controle excedido.")
                return
            state = self._evaluate_kinematics()
            error = self.target_position - np.asarray(state.position)
            error_norm = float(np.linalg.norm(error))
            if error_norm <= self.position_tolerance:
                self.phase = Phase.HOLDING
                self.phase_start_monotonic = now
                self._publish_zero()
                self.get_logger().info(
                    f"Tolerancia atingida: erro={error_norm:.6f} m; validando parada."
                )
                return
            result = compute_position_control(
                error=error,
                translational_jacobian=state.translational_jacobian,
                model_joint_names=self.model_joint_names,
                controller_joint_names=self.controller_joints,
                gains=self.position_gains,
                damping=self.damping,
                max_cartesian_speed=self.max_cartesian_speed,
                max_abs_joint_velocity=self.max_abs_joint_velocity,
            )
            self._publish(result.controller_velocity)
            self._maximum_command = max(
                self._maximum_command,
                max(abs(value) for value in result.controller_velocity),
            )
            if now - self._last_progress_log >= 1.0:
                self.get_logger().info(
                    f"erro={result.error_norm:.6f} m; "
                    f"cart_sat={result.cartesian_saturated}; "
                    f"joint_sat={result.joint_saturated}."
                )
                self._last_progress_log = now
            return

        if self.phase is Phase.HOLDING:
            self._publish_zero()
            state = self._evaluate_kinematics()
            error = self.target_position - np.asarray(state.position)
            error_norm = float(np.linalg.norm(error))
            if error_norm > 2.0 * self.position_tolerance:
                self.request_abort("Erro cresceu durante a validacao da parada.")
                return
            if now - self.phase_start_monotonic >= self.success_hold_duration:
                self._finish_success(state)
            return

        if self.phase is Phase.STOPPING:
            self._publish_zero()
            if now - self.phase_start_monotonic < self.zero_hold_duration:
                return
            self._finish_failure(self.pending_failure or "Falha nao especificada.")


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    exit_code = 1
    try:
        node = CartesianPositionTest()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if node is not None:
            node.request_abort("Interrupcao solicitada pelo operador.")
            deadline = time.monotonic() + node.zero_hold_duration + 0.5
            while rclpy.ok() and not node.finished and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
    except Exception as error:
        print(f"Falha ao iniciar ensaio cartesiano: {error}", file=sys.stderr)
    finally:
        if node is not None:
            exit_code = node.exit_code
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
