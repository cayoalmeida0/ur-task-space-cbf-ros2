"""Ensaio protegido de regulacao cartesiana nominal de posicao."""

from enum import Enum, auto
import math
import os
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
from visualization_msgs.msg import MarkerArray

from ur_cbf_control.experiment import ControlTiming
from ur_cbf_control.experiment import evaluate_control_timing
from ur_cbf_control.experiment import ExperimentDataError
from ur_cbf_control.experiment import write_experiment_record
from ur_cbf_control.kinematics import KinematicState
from ur_cbf_control.kinematics import KinematicsError
from ur_cbf_control.kinematics import UaibotKinematics
from ur_cbf_control.kinematics import homogeneous_transform_from_xyz_rpy
from ur_cbf_control.nominal_control import compute_position_control
from ur_cbf_control.nominal_control import NominalControlError
from ur_cbf_control.nominal_control import reorder_vector
from ur_cbf_control.nominal_control import rotation_error
from ur_cbf_control.qp_control import BoxConstrainedQpSolver
from ur_cbf_control.qp_control import compute_qp_position_control
from ur_cbf_control.qp_control import QpControlError
from ur_cbf_control.qp_control import QpDiagnostics
from ur_cbf_control.safety import JointStateValidationError
from ur_cbf_control.safety import OrderedJointState
from ur_cbf_control.safety import is_state_stale
from ur_cbf_control.safety import reorder_joint_state
from ur_cbf_control.safety import zero_velocity_command
from ur_cbf_control.self_collision_cbf import formulate_self_collision_cbf
from ur_cbf_control.self_collision_cbf import SelfCollisionCbfConstraints
from ur_cbf_control.self_collision_cbf import SelfCollisionCbfError
from ur_cbf_control.task_frames import get_task_frame_spec
from ur_cbf_control.trajectory import resolve_trajectory_waypoints
from ur_cbf_control.workspace_boundary import WorkspaceBoundaryCbfConstraints
from ur_cbf_control.workspace_boundary import WorkspaceBoundaryCbfError
from ur_cbf_control.workspace_boundary import build_workspace_boundary_marker_array
from ur_cbf_control.workspace_boundary import formulate_workspace_boundary_cbf
from ur_cbf_control.witness_visualization import build_witness_marker_array
from ur_cbf_control.witness_visualization import WITNESS_VISUALIZATION_MODES


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
        self.declare_parameter("onrobot_type", "rg2")
        self.declare_parameter("task_type", "cartesian")
        self.declare_parameter("trajectory_profile", "simple")
        self.declare_parameter("target_offset", [0.0, 0.0, 0.01])
        self.declare_parameter("position_gains", [1.0, 1.0, 1.0])
        self.declare_parameter("task_control_mode", "pose")
        self.declare_parameter("orientation_gains", [0.5, 0.5, 0.5])
        self.declare_parameter("orientation_target_mode", "initial")
        self.declare_parameter("target_orientation_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("cube_position", [-0.25, 0.0, 0.32])
        self.declare_parameter("drop_position", [0.20, 0.0])
        self.declare_parameter("manipulation_approach_height", 0.10)
        self.declare_parameter("manipulation_lift_height", 0.12)
        self.declare_parameter("manipulation_drop_approach_height", 0.14)
        self.declare_parameter("manipulation_release_height", 0.08)
        self.declare_parameter("manipulation_retract_height", 0.16)
        self.declare_parameter("gripper_open_width", 0.08)
        self.declare_parameter("gripper_grasp_width", 0.035)
        self.declare_parameter(
            "gripper_width_topic", "/finger_width_controller/commands"
        )
        self.declare_parameter("damping", 0.05)
        self.declare_parameter("controller_mode", "qp")
        self.declare_parameter("qp_absolute_tolerance", 1e-6)
        self.declare_parameter("qp_relative_tolerance", 1e-6)
        self.declare_parameter("qp_max_iterations", 4000)
        self.declare_parameter("qp_time_limit", 0.01)
        self.declare_parameter("qp_polishing", False)
        self.declare_parameter("self_collision_cbf_mode", "off")
        self.declare_parameter("self_collision_safe_distance", 0.03)
        self.declare_parameter("self_collision_cbf_gain", 5.0)
        self.declare_parameter("self_collision_distance_tolerance", 5e-4)
        self.declare_parameter("self_collision_distance_max_iterations", 20)
        self.declare_parameter(
            "self_collision_excluded_pairs",
            Parameter.Type.STRING_ARRAY,
        )
        self.declare_parameter("self_collision_witness_mode", "closest")
        self.declare_parameter(
            "self_collision_witness_topic",
            "/self_collision/witness_markers",
        )
        self.declare_parameter("self_collision_witness_frame", "base")
        self.declare_parameter("workspace_cbf_mode", "monitor")
        self.declare_parameter(
            "workspace_bounds",
            [-0.45, 0.45, -0.55, 0.55, 0.05, 0.90],
        )
        self.declare_parameter("workspace_safe_margin", 0.05)
        self.declare_parameter("workspace_cbf_gain", 7.0)
        self.declare_parameter(
            "workspace_boundary_topic", "/workspace/boundary_markers"
        )
        self.declare_parameter("workspace_boundary_frame", "base")
        self.declare_parameter("workspace_boundary_line_width", 0.005)
        self.declare_parameter("max_cartesian_speed", 0.01)
        self.declare_parameter("max_abs_joint_velocity", 0.10)
        self.declare_parameter("complex_max_cartesian_speed", 0.04)
        self.declare_parameter("complex_max_abs_joint_velocity", 0.30)
        self.declare_parameter("complex_max_control_duration", 120.0)
        self.declare_parameter("complex_max_wall_control_duration", 600.0)
        self.declare_parameter("position_tolerance", 0.001)
        self.declare_parameter("settle_duration", 0.5)
        self.declare_parameter("success_hold_duration", 0.5)
        self.declare_parameter("zero_hold_duration", 0.5)
        self.declare_parameter("max_control_duration", 30.0)
        self.declare_parameter("max_wall_control_duration", 180.0)
        self.declare_parameter("state_timeout", 0.25)
        self.declare_parameter("startup_timeout", 15.0)
        self.declare_parameter("command_rate", 50.0)
        self.declare_parameter("experiment_id", "cartesian_position_ur3e_001")
        self.declare_parameter("random_seed", 0)
        self.declare_parameter("result_directory", "/workspace/results")
        self.declare_parameter("require_result_record", True)

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
        self.onrobot_type = str(self.get_parameter("onrobot_type").value)
        self.task_type = str(self.get_parameter("task_type").value).lower()
        self.trajectory_profile = str(
            self.get_parameter("trajectory_profile").value
        ).lower()
        self.task_frame = get_task_frame_spec(self.onrobot_type)
        self.controlled_frame = self.task_frame.controlled_frame
        self.eef_offset_xyz = self.task_frame.eef_offset_xyz
        self.eef_offset_rpy = self.task_frame.eef_offset_rpy
        self.target_offset = np.asarray(
            self.get_parameter("target_offset").value,
            dtype=float,
        ).reshape(-1)
        self.position_gains = tuple(
            float(value) for value in self.get_parameter("position_gains").value
        )
        self.task_control_mode = str(
            self.get_parameter("task_control_mode").value
        ).lower()
        self.orientation_gains = tuple(
            float(value)
            for value in self.get_parameter("orientation_gains").value
        )
        self.orientation_target_mode = str(
            self.get_parameter("orientation_target_mode").value
        ).lower()
        self.target_orientation_rpy = tuple(
            float(value)
            for value in self.get_parameter("target_orientation_rpy").value
        )
        self.cube_position = np.asarray(
            self.get_parameter("cube_position").value, dtype=float
        ).reshape(-1)
        self.drop_position = np.asarray(
            self.get_parameter("drop_position").value, dtype=float
        ).reshape(-1)
        self.manipulation_approach_height = float(
            self.get_parameter("manipulation_approach_height").value
        )
        self.manipulation_lift_height = float(
            self.get_parameter("manipulation_lift_height").value
        )
        self.manipulation_drop_approach_height = float(
            self.get_parameter("manipulation_drop_approach_height").value
        )
        self.manipulation_release_height = float(
            self.get_parameter("manipulation_release_height").value
        )
        self.manipulation_retract_height = float(
            self.get_parameter("manipulation_retract_height").value
        )
        self.gripper_open_width = float(
            self.get_parameter("gripper_open_width").value
        )
        self.gripper_grasp_width = float(
            self.get_parameter("gripper_grasp_width").value
        )
        self.gripper_width_topic = str(
            self.get_parameter("gripper_width_topic").value
        )
        self.damping = float(self.get_parameter("damping").value)
        self.controller_mode = str(
            self.get_parameter("controller_mode").value
        ).lower()
        self.qp_absolute_tolerance = float(
            self.get_parameter("qp_absolute_tolerance").value
        )
        self.qp_relative_tolerance = float(
            self.get_parameter("qp_relative_tolerance").value
        )
        self.qp_max_iterations = int(
            self.get_parameter("qp_max_iterations").value
        )
        self.qp_time_limit = float(
            self.get_parameter("qp_time_limit").value
        )
        self.qp_polishing = bool(
            self.get_parameter("qp_polishing").value
        )
        self.self_collision_cbf_mode = str(
            self.get_parameter("self_collision_cbf_mode").value
        ).lower()
        self.self_collision_safe_distance = float(
            self.get_parameter("self_collision_safe_distance").value
        )
        self.self_collision_cbf_gain = float(
            self.get_parameter("self_collision_cbf_gain").value
        )
        self.self_collision_distance_tolerance = float(
            self.get_parameter("self_collision_distance_tolerance").value
        )
        self.self_collision_distance_max_iterations = int(
            self.get_parameter("self_collision_distance_max_iterations").value
        )
        self.self_collision_excluded_pairs = tuple(
            self.get_parameter("self_collision_excluded_pairs").value or ()
        )
        self.self_collision_witness_mode = str(
            self.get_parameter("self_collision_witness_mode").value
        ).lower()
        self.self_collision_witness_topic = str(
            self.get_parameter("self_collision_witness_topic").value
        )
        self.self_collision_witness_frame = str(
            self.get_parameter("self_collision_witness_frame").value
        )
        self.workspace_cbf_mode = str(
            self.get_parameter("workspace_cbf_mode").value
        ).lower()
        self.workspace_bounds = tuple(
            float(value) for value in self.get_parameter("workspace_bounds").value
        )
        self.workspace_safe_margin = float(
            self.get_parameter("workspace_safe_margin").value
        )
        self.workspace_cbf_gain = float(
            self.get_parameter("workspace_cbf_gain").value
        )
        self.workspace_boundary_topic = str(
            self.get_parameter("workspace_boundary_topic").value
        )
        self.workspace_boundary_frame = str(
            self.get_parameter("workspace_boundary_frame").value
        )
        self.workspace_boundary_line_width = float(
            self.get_parameter("workspace_boundary_line_width").value
        )
        self.max_cartesian_speed = float(
            self.get_parameter("max_cartesian_speed").value
        )
        self.max_abs_joint_velocity = float(
            self.get_parameter("max_abs_joint_velocity").value
        )
        self.complex_max_cartesian_speed = float(
            self.get_parameter("complex_max_cartesian_speed").value
        )
        self.complex_max_abs_joint_velocity = float(
            self.get_parameter("complex_max_abs_joint_velocity").value
        )
        self.complex_max_control_duration = float(
            self.get_parameter("complex_max_control_duration").value
        )
        self.complex_max_wall_control_duration = float(
            self.get_parameter("complex_max_wall_control_duration").value
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
        self.max_wall_control_duration = float(
            self.get_parameter("max_wall_control_duration").value
        )
        self.state_timeout = float(self.get_parameter("state_timeout").value)
        self.startup_timeout = float(self.get_parameter("startup_timeout").value)
        self.command_rate = float(self.get_parameter("command_rate").value)
        self.experiment_id = str(self.get_parameter("experiment_id").value)
        self.random_seed = int(self.get_parameter("random_seed").value)
        self.result_directory = str(
            self.get_parameter("result_directory").value
        )
        self.require_result_record = bool(
            self.get_parameter("require_result_record").value
        )
        self.waypoint_offsets = resolve_trajectory_waypoints(
            self.trajectory_profile,
            self.target_offset,
        )
        if self.trajectory_profile in {"complex", "challenging"}:
            self.max_cartesian_speed = self.complex_max_cartesian_speed
            self.max_abs_joint_velocity = self.complex_max_abs_joint_velocity
            self.max_control_duration = self.complex_max_control_duration
            self.max_wall_control_duration = self.complex_max_wall_control_duration

        self._validate_parameters()
        np.random.seed(self.random_seed)
        self.kinematics = UaibotKinematics.create(
            ur_type=self.ur_type,
            model_joint_names=self.model_joint_names,
            eef_offset_xyz=self.eef_offset_xyz,
            eef_offset_rpy=self.eef_offset_rpy,
            mode=self.uaibot_mode,
        )
        if self.kinematics.model_corrections:
            corrections = ", ".join(
                f"{item.parameter}: {item.upstream_value:.5f} -> "
                f"{item.corrected_value:.5f} m"
                for item in self.kinematics.model_corrections
            )
            self.get_logger().info(
                f"Correcoes do modelo cinemático aplicadas: {corrections}."
            )
        self.qp_solver = BoxConstrainedQpSolver(
            absolute_tolerance=self.qp_absolute_tolerance,
            relative_tolerance=self.qp_relative_tolerance,
            max_iterations=self.qp_max_iterations,
            time_limit=self.qp_time_limit,
            polishing=self.qp_polishing,
        )

        self.finished = False
        self.exit_code = 1
        self.phase = Phase.WAITING
        self.start_monotonic = time.monotonic()
        self.phase_start_monotonic = self.start_monotonic
        self.phase_start_sim_seconds: float | None = None
        self.control_start_monotonic: float | None = None
        self.control_start_sim_seconds: float | None = None
        self.controller_joints: tuple[str, ...] | None = None
        self.latest_state: OrderedJointState | None = None
        self.latest_state_receipt: float | None = None
        self.initial_position: np.ndarray | None = None
        self.target_position: np.ndarray | None = None
        self.initial_orientation: np.ndarray | None = None
        self.target_orientation: np.ndarray | None = None
        self.waypoint_index = 0
        self.manipulation_targets: tuple[np.ndarray, ...] = ()
        self.gripper_width = self.gripper_open_width
        self._waypoint_arrivals: list[dict[str, object]] = []
        self._last_position: np.ndarray | None = None
        self.pending_failure: str | None = None
        self._joint_parameter_future = None
        self._last_state_error: str | None = None
        self._last_progress_log = self.start_monotonic
        self._maximum_command = 0.0
        self._tolerance_simulated_seconds: float | None = None
        self._tolerance_wall_seconds: float | None = None
        self._trace_samples: list[dict[str, object]] = []
        self._result_path: str | None = None
        self._last_self_collision_cbf: SelfCollisionCbfConstraints | None = None
        self._last_workspace_cbf: WorkspaceBoundaryCbfConstraints | None = None

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
        self.gripper_width_publisher = self.create_publisher(
            Float64MultiArray,
            self.gripper_width_topic,
            10,
        )
        self.witness_publisher = self.create_publisher(
            MarkerArray,
            self.self_collision_witness_topic,
            10,
        )
        self.workspace_boundary_publisher = self.create_publisher(
            MarkerArray,
            self.workspace_boundary_topic,
            10,
        )
        if self.workspace_cbf_mode != "off":
            self.workspace_boundary_publisher.publish(
                build_workspace_boundary_marker_array(
                    self.workspace_bounds,
                    frame_id=self.workspace_boundary_frame,
                    stamp=self.get_clock().now().to_msg(),
                    line_width=self.workspace_boundary_line_width,
                )
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
            self._set_gripper_width(self.gripper_open_width)
            self.get_logger().info(
                f"Ensaio {self.experiment_id} armado; ur_type={self.ur_type}; "
                f"onrobot_type={self.onrobot_type}; "
                f"frame={self.controlled_frame}; "
                f"tarefa={self.task_type}; trajetoria={self.trajectory_profile}; "
                f"waypoints={len(self.waypoint_offsets)}; "
                f"modo={self.controller_mode}; "
                f"self_collision_cbf={self.self_collision_cbf_mode}; "
                f"workspace_cbf={self.workspace_cbf_mode}; "
                f"uaibot={self.kinematics.mode} "
                f"(solicitado={self.kinematics.requested_mode}); "
                f"seed={self.random_seed}; "
                "pacote=0.6.23; imagem esperada=ur-cbf-jazzy:0.2.0."
            )

    def _validate_parameters(self) -> None:
        positive_values = {
            "damping": self.damping,
            "qp_absolute_tolerance": self.qp_absolute_tolerance,
            "qp_relative_tolerance": self.qp_relative_tolerance,
            "qp_time_limit": self.qp_time_limit,
            "max_cartesian_speed": self.max_cartesian_speed,
            "max_abs_joint_velocity": self.max_abs_joint_velocity,
            "complex_max_cartesian_speed": self.complex_max_cartesian_speed,
            "complex_max_abs_joint_velocity": self.complex_max_abs_joint_velocity,
            "complex_max_control_duration": self.complex_max_control_duration,
            "complex_max_wall_control_duration": (
                self.complex_max_wall_control_duration
            ),
            "position_tolerance": self.position_tolerance,
            "settle_duration": self.settle_duration,
            "success_hold_duration": self.success_hold_duration,
            "zero_hold_duration": self.zero_hold_duration,
            "max_control_duration": self.max_control_duration,
            "max_wall_control_duration": self.max_wall_control_duration,
            "state_timeout": self.state_timeout,
            "startup_timeout": self.startup_timeout,
            "command_rate": self.command_rate,
            "self_collision_safe_distance": self.self_collision_safe_distance,
            "self_collision_cbf_gain": self.self_collision_cbf_gain,
            "self_collision_distance_tolerance": (
                self.self_collision_distance_tolerance
            ),
            "workspace_cbf_gain": self.workspace_cbf_gain,
            "workspace_boundary_line_width": self.workspace_boundary_line_width,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} deve ser finito e positivo.")
        if self.controller_mode not in {"dls", "qp"}:
            raise ValueError("controller_mode deve ser dls ou qp.")
        if self.task_control_mode not in {"position", "pose"}:
            raise ValueError("task_control_mode deve ser position ou pose.")
        if self.orientation_target_mode not in {"initial", "rpy", "vertical"}:
            raise ValueError(
                "orientation_target_mode deve ser initial, vertical ou rpy."
            )
        if self.task_type not in {"cartesian", "manipulation"}:
            raise ValueError("task_type deve ser cartesian ou manipulation.")
        if len(self.position_gains) not in {1, 3} or not all(
            math.isfinite(value) and value > 0.0
            for value in self.position_gains
        ):
            raise ValueError("position_gains deve ter 1 ou 3 valores positivos.")
        if len(self.orientation_gains) not in {1, 3} or not all(
            math.isfinite(value) and value > 0.0
            for value in self.orientation_gains
        ):
            raise ValueError(
                "orientation_gains deve ter 1 ou 3 valores positivos."
            )
        if len(self.target_orientation_rpy) != 3 or not all(
            math.isfinite(value) for value in self.target_orientation_rpy
        ):
            raise ValueError(
                "target_orientation_rpy deve conter tres valores finitos."
            )
        if self.task_type == "manipulation":
            if self.cube_position.size != 3 or not np.all(
                np.isfinite(self.cube_position)
            ):
                raise ValueError("cube_position deve conter tres valores finitos.")
            if self.drop_position.size != 2 or not np.all(
                np.isfinite(self.drop_position)
            ):
                raise ValueError("drop_position deve conter x e y finitos.")
            for name, value in {
                "manipulation_approach_height": self.manipulation_approach_height,
                "manipulation_lift_height": self.manipulation_lift_height,
                "manipulation_drop_approach_height": self.manipulation_drop_approach_height,
                "manipulation_release_height": self.manipulation_release_height,
                "manipulation_retract_height": self.manipulation_retract_height,
                "gripper_open_width": self.gripper_open_width,
                "gripper_grasp_width": self.gripper_grasp_width,
            }.items():
                if not math.isfinite(value) or value < 0.0:
                    raise ValueError(f"{name} deve ser finito e nao negativo.")
            if self.gripper_grasp_width > self.gripper_open_width:
                raise ValueError(
                    "gripper_grasp_width nao pode exceder gripper_open_width."
                )
            if not self.gripper_width_topic.strip():
                raise ValueError("gripper_width_topic nao pode ser vazio.")
        if self.self_collision_cbf_mode not in {"off", "monitor", "enforce"}:
            raise ValueError(
                "self_collision_cbf_mode deve ser off, monitor ou enforce."
            )
        if self.self_collision_witness_mode not in WITNESS_VISUALIZATION_MODES:
            raise ValueError(
                "self_collision_witness_mode deve ser off, closest ou all."
            )
        if not self.self_collision_witness_topic.strip():
            raise ValueError("self_collision_witness_topic nao pode ser vazio.")
        if not self.self_collision_witness_frame.strip():
            raise ValueError("self_collision_witness_frame nao pode ser vazio.")
        if self.workspace_cbf_mode not in {"off", "monitor", "enforce"}:
            raise ValueError(
                "workspace_cbf_mode deve ser off, monitor ou enforce."
            )
        if len(self.workspace_bounds) != 6:
            raise ValueError(
                "workspace_bounds deve conter [xmin, xmax, ymin, ymax, zmin, zmax]."
            )
        if not math.isfinite(self.workspace_safe_margin) or (
            self.workspace_safe_margin < 0.0
        ):
            raise ValueError("workspace_safe_margin deve ser finita e nao negativa.")
        if not self.workspace_boundary_topic.strip():
            raise ValueError("workspace_boundary_topic nao pode ser vazio.")
        if not self.workspace_boundary_frame.strip():
            raise ValueError("workspace_boundary_frame nao pode ser vazio.")
        try:
            formulate_workspace_boundary_cbf(
                (0.0, 0.0, 0.0),
                np.zeros((3, max(1, len(self.model_joint_names)))),
                bounds=self.workspace_bounds,
                safety_margin=self.workspace_safe_margin,
                gain=self.workspace_cbf_gain,
            )
        except WorkspaceBoundaryCbfError as error:
            raise ValueError(str(error)) from error
        if self.self_collision_cbf_mode != "off" and (
            self.ur_type,
            self.onrobot_type,
        ) != ("ur3e", "rg2"):
            raise ValueError(
                "A geometria de autocolisao corrigida suporta apenas ur3e + rg2."
            )
        if (
            self.self_collision_cbf_mode == "enforce"
            and self.controller_mode != "qp"
        ):
            raise ValueError(
                "self_collision_cbf_mode=enforce requer controller_mode=qp."
            )
        if self.qp_max_iterations <= 0:
            raise ValueError("qp_max_iterations deve ser positivo.")
        if self.self_collision_distance_max_iterations <= 0:
            raise ValueError(
                "self_collision_distance_max_iterations deve ser positivo."
            )
        if len(self.model_joint_names) == 0:
            raise ValueError("model_joint_names deve ser configurado explicitamente.")
        if len(set(self.model_joint_names)) != len(self.model_joint_names):
            raise ValueError("model_joint_names contem nomes duplicados.")
        if len(self.eef_offset_xyz) != 3 or not all(
            math.isfinite(value) for value in self.eef_offset_xyz
        ):
            raise ValueError("eef_offset_xyz deve conter tres valores finitos.")
        if len(self.eef_offset_rpy) != 3 or not all(
            math.isfinite(value) for value in self.eef_offset_rpy
        ):
            raise ValueError("eef_offset_rpy deve conter tres valores finitos.")
        if self.target_offset.size != 3 or not np.all(
            np.isfinite(self.target_offset)
        ):
            raise ValueError("target_offset deve conter tres valores finitos.")
        if len(self.position_gains) != 3 or not all(
            math.isfinite(value) and value > 0.0
            for value in self.position_gains
        ):
            raise ValueError("position_gains deve conter tres valores positivos.")
        if self.execute_test and all(
            np.linalg.norm(offset) <= 2.0 * self.position_tolerance
            for offset in self.waypoint_offsets
        ):
            raise ValueError(
                "A trajetoria deve conter ao menos um deslocamento superior "
                "a duas vezes a tolerancia."
            )
        if not self.result_directory.strip():
            raise ValueError("result_directory nao pode ser vazio.")

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

    def _evaluate_kinematics(
        self,
        model_positions: tuple[float, ...] | None = None,
    ) -> KinematicState:
        positions = (
            self._model_positions()
            if model_positions is None
            else model_positions
        )
        state = self.kinematics.evaluate(positions)
        self._last_position = np.asarray(state.position, dtype=float)
        return state

    def _task_error(self, state: KinematicState) -> np.ndarray:
        """Monta o erro cartesiano de posicao ou de pose completa."""

        if self.target_position is None:
            raise NominalControlError("Alvo cartesiano ainda nao foi definido.")
        position_error = self.target_position - np.asarray(state.position)
        if self.task_control_mode == "position":
            return position_error
        if self.target_orientation is None:
            raise NominalControlError("Alvo de orientacao ainda nao foi definido.")
        return np.concatenate(
            (position_error, rotation_error(state.orientation_matrix, self.target_orientation))
        )

    @staticmethod
    def _vertical_target_orientation(current: np.ndarray) -> np.ndarray:
        """Constroi uma orientacao com o eixo z do TCP apontando para baixo."""

        x_axis = np.asarray(current[:, 0], dtype=float).copy()
        x_axis[2] = 0.0
        norm = float(np.linalg.norm(x_axis))
        if norm < 1e-9:
            x_axis = np.array((1.0, 0.0, 0.0), dtype=float)
        else:
            x_axis /= norm
        z_axis = np.array((0.0, 0.0, -1.0), dtype=float)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        return np.column_stack((x_axis, y_axis, z_axis))

    def _manipulation_waypoints(self) -> tuple[np.ndarray, ...]:
        """Retorna a sequencia fixa de pick-and-place sem teste de factibilidade."""

        cube = self.cube_position.copy()
        drop = np.array((self.drop_position[0], self.drop_position[1]), dtype=float)
        return (
            cube + np.array((0.0, 0.0, self.manipulation_approach_height)),
            cube.copy(),
            cube + np.array((0.0, 0.0, self.manipulation_lift_height)),
            np.array((drop[0], drop[1], self.manipulation_drop_approach_height)),
            np.array((drop[0], drop[1], self.manipulation_release_height)),
            np.array((drop[0], drop[1], self.manipulation_retract_height)),
        )

    def _set_gripper_width(self, width: float) -> None:
        message = Float64MultiArray()
        message.data = [float(width)]
        self.gripper_width_publisher.publish(message)
        self.gripper_width = float(width)
        self.get_logger().info(f"Garra: largura comandada={width:.4f} m.")

    def _task_jacobian(self, state: KinematicState) -> np.ndarray:
        if self.task_control_mode == "position":
            return state.translational_jacobian
        return state.geometric_jacobian

    def _task_gains(self) -> tuple[float, ...]:
        if self.task_control_mode == "position":
            return self.position_gains
        position_gains = self.position_gains
        orientation_gains = self.orientation_gains
        if len(position_gains) == 1:
            position_gains = position_gains * 3
        if len(orientation_gains) == 1:
            orientation_gains = orientation_gains * 3
        return tuple(position_gains) + tuple(orientation_gains)

    def _evaluate_self_collision_cbf(
        self,
        model_positions: tuple[float, ...],
    ) -> SelfCollisionCbfConstraints | None:
        if self.self_collision_cbf_mode == "off":
            self._last_self_collision_cbf = None
            return None
        distances = self.kinematics.evaluate_self_collision(
            model_positions,
            tolerance=self.self_collision_distance_tolerance,
            max_iterations=self.self_collision_distance_max_iterations,
            excluded_pair_labels=self.self_collision_excluded_pairs,
        )
        constraints = formulate_self_collision_cbf(
            distances,
            safe_distance=self.self_collision_safe_distance,
            gain=self.self_collision_cbf_gain,
        )
        self.witness_publisher.publish(
            build_witness_marker_array(
                constraints,
                mode=self.self_collision_witness_mode,
                frame_id=self.self_collision_witness_frame,
                stamp=self.get_clock().now().to_msg(),
            )
        )
        self._last_self_collision_cbf = constraints
        return constraints

    def _evaluate_workspace_cbf(
        self,
        state: KinematicState,
    ) -> WorkspaceBoundaryCbfConstraints | None:
        if self.workspace_cbf_mode == "off":
            self._last_workspace_cbf = None
            return None
        constraints = formulate_workspace_boundary_cbf(
            state.position,
            state.translational_jacobian,
            bounds=self.workspace_bounds,
            safety_margin=self.workspace_safe_margin,
            gain=self.workspace_cbf_gain,
        )
        self.workspace_boundary_publisher.publish(
            build_workspace_boundary_marker_array(
                self.workspace_bounds,
                frame_id=self.workspace_boundary_frame,
                stamp=self.get_clock().now().to_msg(),
                line_width=self.workspace_boundary_line_width,
            )
        )
        self._last_workspace_cbf = constraints
        return constraints

    def _simulation_seconds(self) -> float:
        seconds = self.get_clock().now().nanoseconds * 1e-9
        if not math.isfinite(seconds):
            raise ExperimentDataError("Relogio ROS retornou tempo nao finito.")
        return seconds

    def _control_timing(
        self,
        *,
        now_wall: float | None = None,
        now_simulated: float | None = None,
    ) -> ControlTiming:
        if (
            self.control_start_monotonic is None
            or self.control_start_sim_seconds is None
        ):
            raise ExperimentDataError("Temporizacao de controle ainda nao iniciou.")
        return evaluate_control_timing(
            start_simulated=self.control_start_sim_seconds,
            current_simulated=(
                self._simulation_seconds()
                if now_simulated is None
                else now_simulated
            ),
            start_wall=self.control_start_monotonic,
            current_wall=time.monotonic() if now_wall is None else now_wall,
            max_simulated=self.max_control_duration,
            max_wall=self.max_wall_control_duration,
        )

    def _record_trace(
        self,
        *,
        timing: ControlTiming,
        error: np.ndarray,
        command: tuple[float, ...],
        phase: str,
        cartesian_saturated: bool,
        joint_saturated: bool = False,
        joint_constraint_active: bool = False,
        self_collision_constraint_active: bool = False,
        workspace_constraint_active: bool = False,
        qp_diagnostics: QpDiagnostics | None = None,
        self_collision_cbf: SelfCollisionCbfConstraints | None = None,
        workspace_cbf: WorkspaceBoundaryCbfConstraints | None = None,
    ) -> None:
        self._trace_samples.append(
            {
                "phase": phase,
                "waypoint_index": self.waypoint_index,
                "waypoint_count": len(self.waypoint_offsets),
                "simulated_seconds": timing.simulated_seconds,
                "wall_seconds": timing.wall_seconds,
                "error": [float(value) for value in error],
                "error_norm": float(np.linalg.norm(error)),
                "controller_velocity": [float(value) for value in command],
                "cartesian_saturated": bool(cartesian_saturated),
                "joint_saturated": bool(joint_saturated),
                "joint_constraint_active": bool(joint_constraint_active),
                "self_collision_constraint_active": bool(
                    self_collision_constraint_active
                ),
                "workspace_constraint_active": bool(workspace_constraint_active),
                "self_collision_cbf": (
                    None
                    if self_collision_cbf is None
                    else self_collision_cbf.to_record()
                ),
                "workspace_cbf": (
                    None if workspace_cbf is None else workspace_cbf.to_record()
                ),
                "qp": (
                    None
                    if qp_diagnostics is None
                    else qp_diagnostics.to_record()
                ),
            }
        )

    def _qp_summary(self) -> dict[str, object] | None:
        samples = [
            sample["qp"]
            for sample in self._trace_samples
            if sample.get("qp") is not None
        ]
        if not samples:
            return None
        statuses: dict[str, int] = {}
        for sample in samples:
            status = str(sample["status"])
            statuses[status] = statuses.get(status, 0) + 1
        iterations = [int(sample["iterations"]) for sample in samples]
        solve_times = [float(sample["solve_time"]) for sample in samples]
        run_times = [float(sample["run_time"]) for sample in samples]
        return {
            "solution_count": len(samples),
            "status_counts": statuses,
            "constraint_active_samples": sum(
                bool(sample["active_lower"] or sample["active_upper"])
                for sample in samples
            ),
            "cbf_active_samples": sum(
                bool(sample["active_cbf"]) for sample in samples
            ),
            "mean_iterations": float(np.mean(iterations)),
            "max_iterations": max(iterations),
            "mean_solve_time": float(np.mean(solve_times)),
            "max_solve_time": max(solve_times),
            "mean_run_time": float(np.mean(run_times)),
            "max_run_time": max(run_times),
            "max_bound_violation": max(
                float(sample["max_bound_violation"])
                for sample in samples
            ),
            "max_cbf_violation": max(
                float(sample["max_cbf_violation"])
                for sample in samples
            ),
        }

    def _result_record(
        self,
        *,
        result: str,
        reason: str,
        final_error_norm: float | None,
    ) -> dict[str, object]:
        simulated_seconds = None
        wall_seconds = None
        if self._trace_samples:
            simulated_seconds = self._trace_samples[-1]["simulated_seconds"]
            wall_seconds = self._trace_samples[-1]["wall_seconds"]
        return {
            "schema_version": "1.6",
            "experiment_id": self.experiment_id,
            "result": result,
            "reason": reason,
            "software": {
                "docker_image": "ur-cbf-jazzy:0.2.0",
                "control_package": "ur_cbf_control:0.6.23",
                "controller_mode": self.controller_mode,
                "self_collision_cbf_mode": self.self_collision_cbf_mode,
                "self_collision_witness_mode": self.self_collision_witness_mode,
                "workspace_cbf_mode": self.workspace_cbf_mode,
                "osqp": self.qp_solver.solver_version,
                "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
                "ur_type": self.ur_type,
                "onrobot_type": self.onrobot_type,
                "kinematic_model": self.kinematics.model_name,
                "uaibot_requested_mode": self.kinematics.requested_mode,
                "uaibot_effective_mode": self.kinematics.mode,
                "kinematic_model_corrections": [
                    correction.as_record()
                    for correction in self.kinematics.model_corrections
                ],
            },
            "random_seed": self.random_seed,
            "trajectory": {
                "profile": self.trajectory_profile,
                "waypoint_offsets_m": [
                    list(offset) for offset in self.waypoint_offsets
                ],
                "arrivals": list(self._waypoint_arrivals),
            },
            "joint_order": {
                "model": list(self.model_joint_names),
                "controller": list(self.controller_joints or ()),
            },
            "positions": {
                "initial": (
                    None
                    if self.initial_position is None
                    else self.initial_position.tolist()
                ),
                "target": (
                    None
                    if self.target_position is None
                    else self.target_position.tolist()
                ),
                "initial_orientation_matrix": (
                    None
                    if self.initial_orientation is None
                    else self.initial_orientation.tolist()
                ),
                "target_orientation_matrix": (
                    None
                    if self.target_orientation is None
                    else self.target_orientation.tolist()
                ),
                "final": (
                    None
                    if self._last_position is None
                    else self._last_position.tolist()
                ),
            },
            "metrics": {
                "initial_error_norm": (
                    None
                    if self.target_position is None
                    else float(np.linalg.norm(self.waypoint_offsets[0]))
                ),
                "final_error_norm": final_error_norm,
                "max_abs_joint_velocity": self._maximum_command,
                "simulated_seconds": simulated_seconds,
                "wall_seconds": wall_seconds,
                "time_to_tolerance_simulated": (
                    self._tolerance_simulated_seconds
                ),
                "time_to_tolerance_wall": self._tolerance_wall_seconds,
                "qp": self._qp_summary(),
                "self_collision_cbf": (
                    None
                    if self._last_self_collision_cbf is None
                    else self._last_self_collision_cbf.to_record()
                ),
                "workspace_cbf": (
                    None
                    if self._last_workspace_cbf is None
                    else self._last_workspace_cbf.to_record()
                ),
            },
            "parameters": {
                "trajectory_profile": self.trajectory_profile,
                "task_type": self.task_type,
                "target_offset": self.target_offset.tolist(),
                "position_gains": list(self.position_gains),
                "task_control_mode": self.task_control_mode,
                "orientation_gains": list(self.orientation_gains),
                "orientation_target_mode": self.orientation_target_mode,
                "target_orientation_rpy": list(self.target_orientation_rpy),
                "cube_position": self.cube_position.tolist(),
                "drop_position": self.drop_position.tolist(),
                "manipulation_approach_height": self.manipulation_approach_height,
                "manipulation_lift_height": self.manipulation_lift_height,
                "manipulation_drop_approach_height": self.manipulation_drop_approach_height,
                "manipulation_release_height": self.manipulation_release_height,
                "manipulation_retract_height": self.manipulation_retract_height,
                "gripper_open_width": self.gripper_open_width,
                "gripper_grasp_width": self.gripper_grasp_width,
                "damping": self.damping,
                "controller_mode": self.controller_mode,
                "qp_absolute_tolerance": self.qp_absolute_tolerance,
                "qp_relative_tolerance": self.qp_relative_tolerance,
                "qp_max_iterations": self.qp_max_iterations,
                "qp_time_limit": self.qp_time_limit,
                "qp_polishing": self.qp_polishing,
                "self_collision_cbf_mode": self.self_collision_cbf_mode,
                "self_collision_safe_distance": (
                    self.self_collision_safe_distance
                ),
                "self_collision_cbf_gain": self.self_collision_cbf_gain,
                "self_collision_distance_tolerance": (
                    self.self_collision_distance_tolerance
                ),
                "self_collision_distance_max_iterations": (
                    self.self_collision_distance_max_iterations
                ),
                "self_collision_excluded_pairs": list(
                    self.self_collision_excluded_pairs
                ),
                "workspace_cbf_mode": self.workspace_cbf_mode,
                "workspace_bounds": list(self.workspace_bounds),
                "workspace_safe_margin": self.workspace_safe_margin,
                "workspace_cbf_gain": self.workspace_cbf_gain,
                "workspace_boundary_topic": self.workspace_boundary_topic,
                "workspace_boundary_frame": self.workspace_boundary_frame,
                "workspace_boundary_line_width": self.workspace_boundary_line_width,
                "max_cartesian_speed": self.max_cartesian_speed,
                "max_abs_joint_velocity": self.max_abs_joint_velocity,
                "position_tolerance": self.position_tolerance,
                "max_simulated_control_duration": self.max_control_duration,
                "max_wall_control_duration": self.max_wall_control_duration,
                "command_rate": self.command_rate,
                "eef_offset_xyz": list(self.eef_offset_xyz),
                "eef_offset_rpy": list(self.eef_offset_rpy),
                "controlled_frame": self.controlled_frame,
                "uaibot_mode": self.uaibot_mode,
            },
            "samples": self._trace_samples,
        }

    def _save_result(
        self,
        *,
        result: str,
        reason: str,
        final_error_norm: float | None,
    ) -> bool:
        try:
            path = write_experiment_record(
                record=self._result_record(
                    result=result,
                    reason=reason,
                    final_error_norm=final_error_norm,
                ),
                directory=self.result_directory,
                experiment_id=self.experiment_id,
            )
        except ExperimentDataError as error:
            self.get_logger().error(str(error))
            return not self.require_result_record
        self._result_path = str(path)
        self.get_logger().info(f"Resultado experimental salvo em {path}.")
        return True

    def request_abort(self, reason: str) -> None:
        if self.finished or self.phase is Phase.STOPPING:
            return
        self._publish_zero()
        self.pending_failure = reason
        self.phase = Phase.STOPPING
        self.phase_start_monotonic = time.monotonic()
        self.get_logger().error(reason)

    def _finish_failure(self, reason: str) -> None:
        self._publish_zero()
        final_error_norm = None
        if self.target_position is not None and self._last_position is not None:
            state = self._evaluate_kinematics()
            final_error_norm = float(np.linalg.norm(self._task_error(state)))
        self._save_result(
            result="rejected",
            reason=reason,
            final_error_norm=final_error_norm,
        )
        self.get_logger().error(f"ENSAIO CARTESIANO REPROVADO: {reason}")
        self.finished = True
        self.exit_code = 1

    def _finish_success(self, state: KinematicState) -> None:
        self._publish_zero()
        error = self._task_error(state)
        timing = self._control_timing()
        self._record_trace(
            timing=timing,
            error=error,
            command=zero_velocity_command(self.controller_joints),
            phase="completed",
            cartesian_saturated=False,
            joint_saturated=False,
        )
        final_error_norm = float(np.linalg.norm(error))
        if not self._save_result(
            result="approved",
            reason="Tolerancia mantida durante a parada.",
            final_error_norm=final_error_norm,
        ):
            self.get_logger().error(
                "ENSAIO CARTESIANO REPROVADO: resultado obrigatorio nao foi salvo."
            )
            self.finished = True
            self.exit_code = 1
            return
        self.get_logger().info(
            "ENSAIO CARTESIANO APROVADO: "
            f"erro_final={final_error_norm:.6f} m; "
            f"max_qdot={self._maximum_command:.6f} rad/s; "
            f"t_sim={timing.simulated_seconds:.3f} s; "
            f"t_real={timing.wall_seconds:.3f} s."
        )
        self.finished = True
        self.exit_code = 0

    def _tick(self) -> None:
        try:
            self._tick_impl()
        except (
            ExperimentDataError,
            KinematicsError,
            NominalControlError,
            QpControlError,
            SelfCollisionCbfError,
            ValueError,
        ) as error:
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
            self.phase_start_sim_seconds = self._simulation_seconds()
            self._publish_zero()
            self.get_logger().info("Estado valido recebido; estabilizando.")
            return

        if self.phase is Phase.SETTLING:
            self._publish_zero()
            settle_timing = evaluate_control_timing(
                start_simulated=self.phase_start_sim_seconds,
                current_simulated=self._simulation_seconds(),
                start_wall=self.phase_start_monotonic,
                current_wall=now,
                max_simulated=self.settle_duration,
                max_wall=self.startup_timeout,
            )
            if settle_timing.wall_limit_reached:
                self._finish_failure(
                    "Tempo real maximo excedido durante a estabilizacao."
                )
                return
            if settle_timing.simulated_limit_reached:
                state = self._evaluate_kinematics()
                self.initial_position = np.asarray(state.position)
                self.initial_orientation = np.asarray(state.orientation_matrix)
                if self.orientation_target_mode == "initial":
                    self.target_orientation = self.initial_orientation.copy()
                elif self.orientation_target_mode == "vertical":
                    self.target_orientation = self._vertical_target_orientation(
                        self.initial_orientation
                    )
                else:
                    self.target_orientation = homogeneous_transform_from_xyz_rpy(
                        (0.0, 0.0, 0.0),
                        self.target_orientation_rpy,
                    )[:3, :3]
                self.waypoint_index = 0
                if self.task_type == "manipulation":
                    self.manipulation_targets = self._manipulation_waypoints()
                    self.waypoint_offsets = tuple(
                        tuple(target - self.initial_position)
                        for target in self.manipulation_targets
                    )
                    self.target_position = self.manipulation_targets[0].copy()
                else:
                    self.target_position = (
                        self.initial_position
                        + np.asarray(self.waypoint_offsets[self.waypoint_index])
                    )
                self.phase = Phase.CONTROLLING
                self.phase_start_monotonic = now
                self.control_start_monotonic = now
                self.control_start_sim_seconds = self._simulation_seconds()
                timing = self._control_timing(
                    now_wall=now,
                    now_simulated=self.control_start_sim_seconds,
                )
                initial_error = self._task_error(state)
                self._record_trace(
                    timing=timing,
                    error=initial_error,
                    command=zero_velocity_command(self.controller_joints),
                    phase="control_start",
                    cartesian_saturated=False,
                    joint_saturated=False,
                )
                self.get_logger().info(
                    "Posicao inicial="
                    f"{self.initial_position.tolist()}; "
                    f"waypoint={self.waypoint_index + 1}/{len(self.waypoint_offsets)}; "
                    f"alvo={self.target_position.tolist()}; "
                    f"offset={list(self.waypoint_offsets[self.waypoint_index])}; "
                    f"v_cart_max={self.max_cartesian_speed:.3f} m/s; "
                    f"qdot_max={self.max_abs_joint_velocity:.3f} rad/s."
                )
            return

        if self.phase is Phase.CONTROLLING:
            now_simulated = self._simulation_seconds()
            timing = self._control_timing(
                now_wall=now,
                now_simulated=now_simulated,
            )
            model_positions = self._model_positions()
            state = self._evaluate_kinematics(model_positions)
            self_collision_cbf = self._evaluate_self_collision_cbf(
                model_positions
            )
            workspace_cbf = self._evaluate_workspace_cbf(state)
            error = self._task_error(state)
            error_norm = float(np.linalg.norm(error))
            if error_norm <= self.position_tolerance:
                self.phase = Phase.HOLDING
                self.phase_start_monotonic = now
                self.phase_start_sim_seconds = now_simulated
                self._tolerance_simulated_seconds = timing.simulated_seconds
                self._tolerance_wall_seconds = timing.wall_seconds
                self._publish_zero()
                self._record_trace(
                    timing=timing,
                    error=error,
                    command=zero_velocity_command(self.controller_joints),
                    phase="tolerance_reached",
                    cartesian_saturated=False,
                    joint_saturated=False,
                    self_collision_cbf=self_collision_cbf,
                    workspace_cbf=workspace_cbf,
                )
                self.get_logger().info(
                    f"Tolerancia atingida: erro_tarefa={error_norm:.6f}; "
                    f"t_sim={timing.simulated_seconds:.3f} s; "
                    f"t_real={timing.wall_seconds:.3f} s; validando parada."
                )
                return
            if timing.simulated_limit_reached:
                self._record_trace(
                    timing=timing,
                    error=error,
                    command=zero_velocity_command(self.controller_joints),
                    phase="simulated_timeout",
                    cartesian_saturated=False,
                    joint_saturated=False,
                    self_collision_cbf=self_collision_cbf,
                    workspace_cbf=workspace_cbf,
                )
                self.request_abort(
                    "Tempo simulado maximo de controle excedido."
                )
                return
            if timing.wall_limit_reached:
                self._record_trace(
                    timing=timing,
                    error=error,
                    command=zero_velocity_command(self.controller_joints),
                    phase="wall_timeout",
                    cartesian_saturated=False,
                    joint_saturated=False,
                    self_collision_cbf=self_collision_cbf,
                    workspace_cbf=workspace_cbf,
                )
                self.request_abort(
                    "Tempo real maximo de seguranca do controle excedido."
                )
                return
            qp_diagnostics = None
            joint_constraint_active = False
            self_collision_constraint_active = False
            workspace_constraint_active = False
            joint_saturated = False
            common_arguments = {
                "error": error,
                "task_jacobian": self._task_jacobian(state),
                "model_joint_names": self.model_joint_names,
                "controller_joint_names": self.controller_joints,
                "gains": self._task_gains(),
                "damping": self.damping,
                "max_cartesian_speed": self.max_cartesian_speed,
                "max_abs_joint_velocity": self.max_abs_joint_velocity,
            }
            if self.controller_mode == "qp":
                cbf_matrices = []
                cbf_lower_bounds = []
                if (
                    self.self_collision_cbf_mode == "enforce"
                    and self_collision_cbf is not None
                ):
                    cbf_matrices.append(self_collision_cbf.matrix)
                    cbf_lower_bounds.append(self_collision_cbf.lower_bound)
                if (
                    self.workspace_cbf_mode == "enforce"
                    and workspace_cbf is not None
                ):
                    cbf_matrices.append(workspace_cbf.matrix)
                    cbf_lower_bounds.append(workspace_cbf.lower_bound)
                if cbf_matrices:
                    common_arguments["cbf_matrix"] = np.vstack(cbf_matrices)
                    common_arguments["cbf_lower_bound"] = np.concatenate(
                        cbf_lower_bounds
                    )
                result = compute_qp_position_control(
                    **common_arguments,
                    solver=self.qp_solver,
                )
                qp_diagnostics = result.diagnostics
                joint_constraint_active = result.joint_constraint_active
                self_cbf_count = (
                    self_collision_cbf.count
                    if self.self_collision_cbf_mode == "enforce"
                    and self_collision_cbf is not None
                    else 0
                )
                self_collision_constraint_active = any(
                    index < self_cbf_count
                    for index in result.diagnostics.active_cbf
                )
                workspace_constraint_active = any(
                    index >= self_cbf_count
                    for index in result.diagnostics.active_cbf
                ) and self.workspace_cbf_mode == "enforce"
            else:
                result = compute_position_control(**common_arguments)
                joint_saturated = result.joint_saturated
            self._publish(result.controller_velocity)
            self._maximum_command = max(
                self._maximum_command,
                max(abs(value) for value in result.controller_velocity),
            )
            self._record_trace(
                timing=timing,
                error=error,
                command=result.controller_velocity,
                phase="controlling",
                cartesian_saturated=result.cartesian_saturated,
                joint_saturated=joint_saturated,
                joint_constraint_active=joint_constraint_active,
                self_collision_constraint_active=(
                    self_collision_constraint_active
                ),
                workspace_constraint_active=workspace_constraint_active,
                qp_diagnostics=qp_diagnostics,
                self_collision_cbf=self_collision_cbf,
                workspace_cbf=workspace_cbf,
            )
            if now - self._last_progress_log >= 1.0:
                real_time_factor = (
                    timing.simulated_seconds / timing.wall_seconds
                    if timing.wall_seconds > 0.0
                    else 0.0
                )
                qp_progress = (
                    ""
                    if qp_diagnostics is None
                    else (
                        f"qp_iter={qp_diagnostics.iterations}; "
                        f"qp_us={1e6 * qp_diagnostics.solve_time:.1f}; "
                    )
                )
                cbf_progress = (
                    ""
                    if self_collision_cbf is None
                    else (
                        f"d_self_min={self_collision_cbf.minimum_distance:.4f} m; "
                        f"h_self_min={self_collision_cbf.minimum_barrier:.4f} m; "
                        f"cbf_ms={1e3 * self_collision_cbf.evaluation_time:.1f}; "
                        f"par={self_collision_cbf.closest_pair}; "
                    )
                )
                workspace_progress = (
                    ""
                    if workspace_cbf is None
                    else (
                        f"h_workspace_min={workspace_cbf.minimum_barrier:.4f} m; "
                        f"boundary={workspace_cbf.closest_boundary}; "
                    )
                )
                self.get_logger().info(
                    f"waypoint={self.waypoint_index + 1}/"
                    f"{len(self.waypoint_offsets)}; "
                    f"erro_tarefa={result.error_norm:.6f}; "
                    f"cart_sat={result.cartesian_saturated}; "
                    f"joint_sat={joint_saturated}; "
                    f"joint_active={joint_constraint_active}; "
                    f"self_cbf_active={self_collision_constraint_active}; "
                    f"workspace_cbf_active={workspace_constraint_active}; "
                    f"{qp_progress}"
                    f"{cbf_progress}"
                    f"{workspace_progress}"
                    f"t_sim={timing.simulated_seconds:.3f} s; "
                    f"t_real={timing.wall_seconds:.3f} s; "
                    f"RTF={real_time_factor:.3f}."
                )
                self._last_progress_log = now
            return

        if self.phase is Phase.HOLDING:
            self._publish_zero()
            now_simulated = self._simulation_seconds()
            timing = self._control_timing(
                now_wall=now,
                now_simulated=now_simulated,
            )
            state = self._evaluate_kinematics()
            error = self._task_error(state)
            error_norm = float(np.linalg.norm(error))
            if timing.wall_limit_reached:
                self.request_abort(
                    "Tempo real maximo excedido durante a validacao da parada."
                )
                return
            if error_norm > 2.0 * self.position_tolerance:
                self._record_trace(
                    timing=timing,
                    error=error,
                    command=zero_velocity_command(self.controller_joints),
                    phase="hold_error",
                    cartesian_saturated=False,
                    joint_saturated=False,
                )
                self.request_abort("Erro cresceu durante a validacao da parada.")
                return
            hold_timing = evaluate_control_timing(
                start_simulated=self.phase_start_sim_seconds,
                current_simulated=now_simulated,
                start_wall=self.phase_start_monotonic,
                current_wall=now,
                max_simulated=self.success_hold_duration,
                max_wall=self.max_wall_control_duration,
            )
            if hold_timing.simulated_limit_reached:
                if self.task_type == "manipulation":
                    if self.waypoint_index == 1:
                        self._set_gripper_width(self.gripper_grasp_width)
                    elif self.waypoint_index == 4:
                        self._set_gripper_width(self.gripper_open_width)
                self._waypoint_arrivals.append(
                    {
                        "waypoint_index": self.waypoint_index,
                        "offset_m": list(
                            self.waypoint_offsets[self.waypoint_index]
                        ),
                        "simulated_seconds": timing.simulated_seconds,
                        "wall_seconds": timing.wall_seconds,
                        "error_norm_task": error_norm,
                    }
                )
                if self.waypoint_index + 1 < len(self.waypoint_offsets):
                    self.waypoint_index += 1
                    if self.task_type == "manipulation":
                        self.target_position = self.manipulation_targets[
                            self.waypoint_index
                        ].copy()
                    else:
                        self.target_position = (
                            self.initial_position
                            + np.asarray(self.waypoint_offsets[self.waypoint_index])
                        )
                    self.phase = Phase.CONTROLLING
                    self.phase_start_monotonic = now
                    self.phase_start_sim_seconds = now_simulated
                    self.get_logger().info(
                        f"Iniciando waypoint {self.waypoint_index + 1}/"
                        f"{len(self.waypoint_offsets)}; "
                        f"alvo={self.target_position.tolist()}; "
                        f"offset={list(self.waypoint_offsets[self.waypoint_index])}."
                    )
                else:
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
