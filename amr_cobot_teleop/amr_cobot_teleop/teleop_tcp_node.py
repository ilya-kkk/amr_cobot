from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist


def _apply_deadzone(x: float, deadzone: float) -> float:
    ax = abs(x)
    if ax <= deadzone:
        return 0.0
    # Rescale so that output is still in [-1, 1]
    return (ax - deadzone) / (1.0 - deadzone) * (1.0 if x > 0.0 else -1.0)


def _axis(axes: list[float], idx: int) -> float:
    if idx < 0 or idx >= len(axes):
        return 0.0
    return float(axes[idx])


def _button(buttons: list[int], idx: int) -> bool:
    if idx < 0 or idx >= len(buttons):
        return False
    return bool(buttons[idx])


@dataclass
class Mapping:
    # Logical sticks
    axis_lx: int = 0
    axis_ly: int = 1
    axis_rx: int = 2
    axis_ry: int = 3

    # 2-position mode switch axis (optional)
    axis_mode_switch: int = -1
    mode1_when_switch_gt: float = 0.5
    mode2_when_switch_lt: float = -0.5

    axis_speed_scale: int = -1  # optional knob/slider

    invert_lx: bool = False
    invert_ly: bool = False
    invert_rx: bool = False
    invert_ry: bool = False

    # deadman_button = -1 means "always enabled"
    deadman_button: int = -1


class TeleopTcpNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_tcp_node")

        # Frames / topics
        self.declare_parameter("tool_frame", "tool0")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        # Rates / filtering
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("deadzone", 0.08)

        # Optional runtime scale (multiplies normalized [-1..1] values)
        self.declare_parameter("linear_scale", 1.0)
        self.declare_parameter("angular_scale", 1.0)

        # Mapping
        self.declare_parameter("axis_lx", 0)
        self.declare_parameter("axis_ly", 1)
        self.declare_parameter("axis_rx", 2)
        self.declare_parameter("axis_ry", 3)
        self.declare_parameter("axis_mode_switch", -1)
        self.declare_parameter("mode1_when_switch_gt", 0.5)
        self.declare_parameter("mode2_when_switch_lt", -0.5)
        self.declare_parameter("axis_speed_scale", -1)

        self.declare_parameter("invert_lx", False)
        self.declare_parameter("invert_ly", False)
        self.declare_parameter("invert_rx", False)
        self.declare_parameter("invert_ry", False)

        self.declare_parameter("deadman_button", -1)

        self._mapping = self._load_mapping()
        self._last_joy: Optional[Joy] = None
        self._mode: int = 1

        joy_topic = self.get_parameter("joy_topic").get_parameter_value().string_value
        twist_topic = self.get_parameter("twist_topic").get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value

        self._sub = self.create_subscription(Joy, joy_topic, self._on_joy, 10)
        self._pub = self.create_publisher(TwistStamped, twist_topic, 10)
        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        rate = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / max(1.0, rate)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"TeleopTCP ready: joy='{joy_topic}' -> twist='{twist_topic}' (tool_frame='{self.get_parameter('tool_frame').value}'), cmd_vel='{cmd_vel_topic}'"
        )

    def _load_mapping(self) -> Mapping:
        return Mapping(
            axis_lx=int(self.get_parameter("axis_lx").value),
            axis_ly=int(self.get_parameter("axis_ly").value),
            axis_rx=int(self.get_parameter("axis_rx").value),
            axis_ry=int(self.get_parameter("axis_ry").value),
            axis_mode_switch=int(self.get_parameter("axis_mode_switch").value),
            mode1_when_switch_gt=float(self.get_parameter("mode1_when_switch_gt").value),
            mode2_when_switch_lt=float(self.get_parameter("mode2_when_switch_lt").value),
            axis_speed_scale=int(self.get_parameter("axis_speed_scale").value),
            invert_lx=bool(self.get_parameter("invert_lx").value),
            invert_ly=bool(self.get_parameter("invert_ly").value),
            invert_rx=bool(self.get_parameter("invert_rx").value),
            invert_ry=bool(self.get_parameter("invert_ry").value),
            deadman_button=int(self.get_parameter("deadman_button").value),
        )

    def _on_joy(self, msg: Joy) -> None:
        self._last_joy = msg

    def _tick(self) -> None:
        twist_out = TwistStamped()
        twist_out.header.stamp = self.get_clock().now().to_msg()
        twist_out.header.frame_id = self.get_parameter("tool_frame").value

        cmd_out = Twist()

        # Default: publish zeros (safe stop)
        if self._last_joy is None:
            self._pub.publish(twist_out)
            self._cmd_pub.publish(cmd_out)
            return

        joy = self._last_joy
        enabled = True if self._mapping.deadman_button < 0 else _button(joy.buttons, self._mapping.deadman_button)
        if not enabled:
            self._pub.publish(twist_out)
            self._cmd_pub.publish(cmd_out)
            return

        deadzone = float(self.get_parameter("deadzone").value)
        lin_scale = float(self.get_parameter("linear_scale").value)
        ang_scale = float(self.get_parameter("angular_scale").value)

        # Optional speed knob [-1..1] -> [0..1]
        speed_factor = 1.0
        if self._mapping.axis_speed_scale >= 0:
            knob = _apply_deadzone(_axis(joy.axes, self._mapping.axis_speed_scale), deadzone)
            speed_factor = max(0.0, min(1.0, 0.5 * (knob + 1.0)))

        def v(idx: int, invert: bool) -> float:
            val = _apply_deadzone(_axis(joy.axes, idx), deadzone)
            if invert:
                val *= -1.0
            return val

        lx = v(self._mapping.axis_lx, self._mapping.invert_lx)
        ly = v(self._mapping.axis_ly, self._mapping.invert_ly)
        rx = v(self._mapping.axis_rx, self._mapping.invert_rx)
        ry = v(self._mapping.axis_ry, self._mapping.invert_ry)

        # Mode switch (if configured)
        if self._mapping.axis_mode_switch >= 0:
            sw = _axis(joy.axes, self._mapping.axis_mode_switch)
            if sw >= self._mapping.mode1_when_switch_gt:
                self._mode = 1
            elif sw <= self._mapping.mode2_when_switch_lt:
                self._mode = 2

        if self._mode == 1:
            # Mode 1 (manipulator TCP): LX->X, LY->Y, RX->RZ, RY->Z
            twist_out.twist.linear.x = lx * lin_scale * speed_factor
            twist_out.twist.linear.y = ly * lin_scale * speed_factor
            twist_out.twist.linear.z = ry * lin_scale * speed_factor
            twist_out.twist.angular.z = rx * ang_scale * speed_factor
        else:
            # Mode 2:
            #   AMR: LY->forward/back, LX->yaw
            #   TCP orientation: RX->RX, RY->RY (in tool frame)
            cmd_out.linear.x = -ly * lin_scale * speed_factor
            cmd_out.angular.z = -lx * ang_scale * speed_factor

            twist_out.twist.angular.x = rx * ang_scale * speed_factor
            twist_out.twist.angular.y = ry * ang_scale * speed_factor

        self._pub.publish(twist_out)
        self._cmd_pub.publish(cmd_out)


def main() -> None:
    rclpy.init()
    node = TeleopTcpNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()



