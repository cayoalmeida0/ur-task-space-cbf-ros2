"""Adapta uma largura OnRobot para as juntas fisicas simuladas no Gazebo."""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from ur_cbf_bringup.grippers import get_gripper_spec


class OnRobotWidthAdapter(Node):
    def __init__(self):
        super().__init__("onrobot_width_adapter")
        self.declare_parameter("onrobot_type", "rg2")
        self.declare_parameter("input_topic", "/finger_width_controller/commands")
        self.declare_parameter(
            "output_topic", "/onrobot_joint_position_controller/commands"
        )
        self.declare_parameter("publish_rate_hz", 50.0)

        onrobot_type = self.get_parameter("onrobot_type").value
        self._spec = get_gripper_spec(onrobot_type)
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if not math.isfinite(publish_rate_hz) or publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz deve ser positivo e finito.")

        self._desired_width_m = self._spec.maximum_width_m
        self._publisher = self.create_publisher(Float64MultiArray, output_topic, 10)
        self._subscription = self.create_subscription(
            Float64MultiArray, input_topic, self._command_callback, 10
        )
        self._timer = self.create_timer(
            1.0 / publish_rate_hz, self._publish_joint_command
        )

        self.get_logger().info(
            f"Adaptador {onrobot_type.upper()} ativo; largura=[0, "
            f"{self._spec.maximum_width_m:.3f}] m; entrada={input_topic}; "
            f"saida={output_topic}."
        )

    def _command_callback(self, message: Float64MultiArray):
        if len(message.data) != 1:
            self.get_logger().error(
                "Comando da gripper deve conter exatamente uma largura."
            )
            return
        width_m = float(message.data[0])
        if not math.isfinite(width_m):
            self.get_logger().error("Comando da gripper deve ser finito.")
            return
        try:
            self._spec.joint_positions(width_m)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        self._desired_width_m = width_m
        self.get_logger().info(f"Nova largura desejada: {width_m:.4f} m.")

    def _publish_joint_command(self):
        message = Float64MultiArray()
        message.data = list(self._spec.joint_positions(self._desired_width_m))
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OnRobotWidthAdapter()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
