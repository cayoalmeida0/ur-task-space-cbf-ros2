"""Adapta a interface comum da gripper ao driver OnRobot real."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from ur_cbf_bringup.grippers import get_gripper_spec


class OnRobotRealAdapter(Node):
    def __init__(self):
        super().__init__("onrobot_real_adapter")
        self.declare_parameter("onrobot_type", "rg2")
        self.declare_parameter("command_input_topic", "/finger_width_controller/commands")
        self.declare_parameter(
            "command_output_topic", "/onrobot/finger_width_controller/commands"
        )
        self.declare_parameter("state_input_topic", "/onrobot/joint_states")
        self.declare_parameter("state_output_topic", "/onrobot/visual_joint_states")
        self.declare_parameter("state_joint_name", "finger_width")

        onrobot_type = self.get_parameter("onrobot_type").value
        self._spec = get_gripper_spec(onrobot_type)
        self._state_joint_name = self.get_parameter("state_joint_name").value

        command_input = self.get_parameter("command_input_topic").value
        command_output = self.get_parameter("command_output_topic").value
        state_input = self.get_parameter("state_input_topic").value
        state_output = self.get_parameter("state_output_topic").value

        self._command_publisher = self.create_publisher(
            Float64MultiArray, command_output, 10
        )
        self._state_publisher = self.create_publisher(JointState, state_output, 10)
        self._command_subscription = self.create_subscription(
            Float64MultiArray, command_input, self._command_callback, 10
        )
        self._state_subscription = self.create_subscription(
            JointState, state_input, self._state_callback, 10
        )

        self.get_logger().info(
            f"Adaptador real {onrobot_type.upper()} ativo; comando={command_input} -> "
            f"{command_output}; estado={state_input} -> {state_output}."
        )

    def _validated_width(self, width_m: float) -> float:
        if not math.isfinite(width_m):
            raise ValueError("Largura da gripper deve ser finita.")
        self._spec.joint_positions(width_m)
        return width_m

    def _command_callback(self, message: Float64MultiArray):
        if len(message.data) != 1:
            self.get_logger().error(
                "Comando da gripper deve conter exatamente uma largura."
            )
            return
        try:
            width_m = self._validated_width(float(message.data[0]))
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        output = Float64MultiArray()
        output.data = [width_m]
        self._command_publisher.publish(output)
        self.get_logger().info(f"Comando RG encaminhado: {width_m:.4f} m.")

    def _state_callback(self, message: JointState):
        try:
            index = message.name.index(self._state_joint_name)
            width_m = self._validated_width(float(message.position[index]))
        except (ValueError, IndexError) as error:
            self.get_logger().error(f"Estado RG invalido: {error}")
            return

        output = JointState()
        output.header = message.header
        output.name = list(self._spec.joint_names)
        output.position = list(self._spec.joint_positions(width_m))
        self._state_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OnRobotRealAdapter()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
