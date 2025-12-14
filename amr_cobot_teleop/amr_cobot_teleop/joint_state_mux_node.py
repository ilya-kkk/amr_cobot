from __future__ import annotations

from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class JointStateMuxNode(Node):
    """
    Merge incoming /joint_states with a fixed set of "extra" joints (published with default values).

    This is used to satisfy MoveIt/Servo PlanningSceneMonitor, which expects a complete robot state
    (all joints from the URDF) on the joint topic, even if some joints are passive or not simulated in ros2_control.
    """

    def __init__(self) -> None:
        super().__init__("joint_state_mux_node")

        self.declare_parameter("in_topic", "/joint_states")
        self.declare_parameter("out_topic", "/joint_states_mux")
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
        self._last_in: Optional[JointState] = None
        self._last_map: Dict[str, float] = {}

        in_topic = str(self.get_parameter("in_topic").value)
        out_topic = str(self.get_parameter("out_topic").value)

        # Use sensor-data QoS to connect to best-effort joint_state sources (ros2_control).
        self._sub = self.create_subscription(JointState, in_topic, self._on_joint_state, qos_profile_sensor_data)
        self._pub = self.create_publisher(JointState, out_topic, qos_profile_sensor_data)

        rate = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / max(1.0, rate)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(f"JointStateMux: '{in_topic}' -> '{out_topic}', extra={self._extra_joints}")

    def _on_joint_state(self, msg: JointState) -> None:
        self._last_in = msg
        # Build map of latest positions (velocity/effort are optional for MoveIt state)
        self._last_map = {name: float(pos) for name, pos in zip(msg.name, msg.position)}

    def _tick(self) -> None:
        if self._last_in is None:
            return

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._last_in.header.frame_id

        # Start with original joints (preserve ordering)
        out.name = list(self._last_in.name)
        out.position = list(self._last_in.position)
        out.velocity = list(self._last_in.velocity)
        out.effort = list(self._last_in.effort)

        # Append missing extras
        existing = set(out.name)
        for j in self._extra_joints:
            if j in existing:
                continue
            out.name.append(j)
            out.position.append(0.0)
            # Keep arrays consistent sizes
            if len(out.velocity) > 0:
                out.velocity.append(0.0)
            if len(out.effort) > 0:
                out.effort.append(0.0)

        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = JointStateMuxNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


