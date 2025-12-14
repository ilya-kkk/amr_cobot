from __future__ import annotations

from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class JointStateExtrasNode(Node):
    """
    Publish a small JointState message with passive/uncontrolled joints (wheels, gripper sliders, etc.)
    so MoveIt / PlanningSceneMonitor can consider the robot state "complete".

    This node intentionally publishes ONLY the extra joints; CurrentStateMonitor merges multiple JointState
    messages over time.
    """

    def __init__(self) -> None:
        super().__init__("joint_state_extras_node")

        self.declare_parameter("out_topic", "/joint_states")
        self.declare_parameter(
            "extra_joints",
            [
                "left_wheel_joint",
                "right_wheel_joint",
                "1_hand_joint",
                "2_hand_joint",
            ],
        )
        self.declare_parameter("publish_rate_hz", 50.0)

        self._extra_joints: List[str] = list(self.get_parameter("extra_joints").value)
        out_topic = str(self.get_parameter("out_topic").value)

        # Match joint_state_broadcaster / sensor-data semantics (best effort, volatile)
        self._pub = self.create_publisher(JointState, out_topic, qos_profile_sensor_data)

        rate = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / max(1.0, rate)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(f"JointStateExtras publishing {self._extra_joints} -> '{out_topic}' @ {rate} Hz")

    def _tick(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._extra_joints)
        msg.position = [0.0] * len(self._extra_joints)
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = JointStateExtrasNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


