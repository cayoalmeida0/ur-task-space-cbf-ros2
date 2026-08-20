"""Ensaio monoarticular protegido para validar a interface de velocidades."""

from enum import Enum, auto
import math
import sys
import time

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from ur_cbf_control.safety import JointStateValidationError
from ur_cbf_control.safety import OrderedJointState
from ur_cbf_control.safety import build_velocity_command
from ur_cbf_control.safety import is_state_stale
from ur_cbf_control.safety import reorder_joint_state
from ur_cbf_control.safety import zero_velocity_command


class Phase(Enum):
    WAITING = auto()
    SETTLING = auto()
    PULSE = auto()
    STOPPING = auto()


class JointVelocityPulseTest(Node):
    """Executa um pulso curto e garante comando nulo ao finalizar ou abortar."""

    def __init__(self) -> None:
        super().__init__("joint_velocity_pulse_test")
        self.declare_parameter("execute_test", False)
        self.declare_parameter("controller_node", "/forward_velocity_controller")
        self.declare_parameter(
            "command_topic", "/forward_velocity_controller/commands"
        )
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("required_simulation_node", "/gz_ros_control")
        self.declare_parameter("target_joint", "")
        self.declare_parameter("pulse_velocity", 0.03)
        self.declare_parameter("max_abs_velocity", 0.05)
        self.declare_parameter("pulse_duration", 0.5)
        self.declare_parameter("settle_duration", 0.5)
        self.declare_parameter("zero_hold_duration", 0.5)
        self.declare_parameter("state_timeout", 0.25)
        self.declare_parameter("startup_timeout", 10.0)
        self.declare_parameter("command_rate", 50.0)
        self.declare_parameter("minimum_motion_ratio", 0.5)
        self.declare_parameter("maximum_motion_ratio", 1.5)
        self.declare_parameter("max_other_joint_displacement", 0.003)

        self.execute_test = bool(self.get_parameter("execute_test").value)
        self.controller_node = str(self.get_parameter("controller_node").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.joint_states_topic = str(
            self.get_parameter("joint_states_topic").value
        )
        self.required_simulation_node = str(
            self.get_parameter("required_simulation_node").value
        )
        self.target_joint = str(self.get_parameter("target_joint").value)
        self.pulse_velocity = float(self.get_parameter("pulse_velocity").value)
        self.max_abs_velocity = float(
            self.get_parameter("max_abs_velocity").value
        )
        self.pulse_duration = float(self.get_parameter("pulse_duration").value)
        self.settle_duration = float(self.get_parameter("settle_duration").value)
        self.zero_hold_duration = float(
            self.get_parameter("zero_hold_duration").value
        )
        self.state_timeout = float(self.get_parameter("state_timeout").value)
        self.startup_timeout = float(self.get_parameter("startup_timeout").value)
        self.command_rate = float(self.get_parameter("command_rate").value)
        self.minimum_motion_ratio = float(
            self.get_parameter("minimum_motion_ratio").value
        )
        self.maximum_motion_ratio = float(
            self.get_parameter("maximum_motion_ratio").value
        )
        self.max_other_joint_displacement = float(
            self.get_parameter("max_other_joint_displacement").value
        )

        self._validate_parameters()
        self.finished = False
        self.exit_code = 1
        self.phase = Phase.WAITING
        self.start_monotonic = time.monotonic()
        self.phase_start_monotonic = self.start_monotonic
        self.controller_joints: tuple[str, ...] | None = None
        self.latest_state: OrderedJointState | None = None
        self.latest_state_receipt: float | None = None
        self.initial_state: OrderedJointState | None = None
        self.pending_failure: str | None = None
        self._joint_parameter_future = None
        self._last_state_error: str | None = None

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
                "Teste desarmado. Execute com execute_test:=true e informe target_joint."
            )
            self.finished = True
            self.exit_code = 2
        else:
            self.get_logger().info(
                "Ensaio armado somente para simulacao; aguardando controlador e estado."
            )

    def _validate_parameters(self) -> None:
        positive_values = {
            "command_rate": self.command_rate,
            "pulse_duration": self.pulse_duration,
            "settle_duration": self.settle_duration,
            "zero_hold_duration": self.zero_hold_duration,
            "state_timeout": self.state_timeout,
            "startup_timeout": self.startup_timeout,
            "max_abs_velocity": self.max_abs_velocity,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} deve ser finito e positivo.")
        if not 0.0 < self.minimum_motion_ratio <= self.maximum_motion_ratio:
            raise ValueError("Intervalo de razao de movimento invalido.")
        if self.execute_test and not self.target_joint:
            raise ValueError("target_joint e obrigatorio quando execute_test=true.")

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
                command = build_velocity_command(
                    self.controller_joints,
                    self.target_joint,
                    self.pulse_velocity,
                    self.max_abs_velocity,
                )
            except ValueError as error:
                self._finish_failure(str(error))
                return
            self.pulse_velocity = command.applied_velocity
            if command.saturated:
                self.get_logger().warning(
                    f"Velocidade saturada para {self.pulse_velocity:.6f} rad/s."
                )
            self.get_logger().info(
                "Ordem do controlador: " + ", ".join(self.controller_joints)
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

    def _publish_pulse(self) -> None:
        if self.controller_joints is None:
            return
        command = build_velocity_command(
            self.controller_joints,
            self.target_joint,
            self.pulse_velocity,
            self.max_abs_velocity,
        )
        self._publish(command.values)

    def request_abort(self, reason: str) -> None:
        if self.finished or self.phase is Phase.STOPPING:
            return
        self.pending_failure = reason
        self.phase = Phase.STOPPING
        self.phase_start_monotonic = time.monotonic()
        self.get_logger().error(reason)

    def _finish_failure(self, reason: str) -> None:
        self._publish_zero()
        self.get_logger().error(f"ENSAIO REPROVADO: {reason}")
        self.finished = True
        self.exit_code = 1

    def _evaluate_result(self) -> None:
        if self.initial_state is None or self.latest_state is None:
            self._finish_failure("Estados inicial ou final indisponiveis.")
            return
        target_index = self.controller_joints.index(self.target_joint)
        displacements = tuple(
            final - initial
            for initial, final in zip(
                self.initial_state.positions,
                self.latest_state.positions,
            )
        )
        target_displacement = displacements[target_index]
        expected_displacement = self.pulse_velocity * self.pulse_duration
        expected_abs = abs(expected_displacement)
        measured_abs = abs(target_displacement)
        ratio = measured_abs / expected_abs
        other_max = max(
            (
                abs(value)
                for index, value in enumerate(displacements)
                if index != target_index
            ),
            default=0.0,
        )
        correct_direction = target_displacement * expected_displacement > 0.0
        ratio_ok = self.minimum_motion_ratio <= ratio <= self.maximum_motion_ratio
        other_ok = other_max <= self.max_other_joint_displacement

        self.get_logger().info(
            f"Deslocamento esperado={expected_displacement:.6f} rad; "
            f"medido={target_displacement:.6f} rad; razao={ratio:.3f}; "
            f"maior deslocamento nas demais juntas={other_max:.6f} rad."
        )
        if not correct_direction:
            self._finish_failure("A junta alvo moveu no sentido incorreto.")
            return
        if not ratio_ok:
            self._finish_failure("Deslocamento da junta alvo fora da tolerancia.")
            return
        if not other_ok:
            self._finish_failure("Movimento excessivo em junta nao comandada.")
            return

        self.get_logger().info("ENSAIO APROVADO: interface de velocidade validada.")
        self.finished = True
        self.exit_code = 0

    def _tick(self) -> None:
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
                self._finish_failure("Timeout ao aguardar estado ou assinante do comando.")
            return

        if not self._simulation_is_available():
            self._publish_zero()
            if now - self.start_monotonic > self.startup_timeout:
                self._finish_failure(
                    "No de simulacao nao encontrado; o ensaio nao opera no robo real."
                )
            return

        if self.phase is not Phase.STOPPING and self._state_is_stale(now):
            self.request_abort("/joint_states ficou obsoleto; aplicando comando nulo.")

        if self.phase is Phase.WAITING:
            self.initial_state = self.latest_state
            self.phase = Phase.SETTLING
            self.phase_start_monotonic = now
            self._publish_zero()
            self.get_logger().info("Estado valido recebido; estabilizando com comando nulo.")
            return

        if self.phase is Phase.SETTLING:
            self._publish_zero()
            if now - self.phase_start_monotonic >= self.settle_duration:
                self.initial_state = self.latest_state
                self.phase = Phase.PULSE
                self.phase_start_monotonic = now
                self.get_logger().info(
                    f"Aplicando {self.pulse_velocity:.6f} rad/s em "
                    f"{self.target_joint} por {self.pulse_duration:.3f} s."
                )
            return

        if self.phase is Phase.PULSE:
            self._publish_pulse()
            if now - self.phase_start_monotonic >= self.pulse_duration:
                self.phase = Phase.STOPPING
                self.phase_start_monotonic = now
                self.get_logger().info("Pulso concluido; mantendo comando nulo.")
            return

        if self.phase is Phase.STOPPING:
            self._publish_zero()
            if now - self.phase_start_monotonic < self.zero_hold_duration:
                return
            if self.pending_failure is not None:
                self._finish_failure(self.pending_failure)
                return
            if self._state_is_stale(now):
                self._finish_failure("Estado final obsoleto.")
                return
            self._evaluate_result()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = JointVelocityPulseTest()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.request_abort("Interrupcao solicitada pelo operador.")
        deadline = time.monotonic() + node.zero_hold_duration + 0.5
        while rclpy.ok() and not node.finished and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
