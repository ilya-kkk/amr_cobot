import os
import re

import rclpy
from rclpy.node import Node


def _read_text(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def _strip_xml_decl(s: str) -> str:
    # remove optional XML declaration
    return re.sub(r"^\s*<\?xml[^>]*\?>\s*", "", s, count=1, flags=re.IGNORECASE).lstrip()


def _remove_xml_comments(s: str) -> str:
    return re.sub(r"<!--.*?-->", "", s, flags=re.DOTALL)


def _remove_visual_and_collision_blocks(urdf: str) -> str:
    # Remove <visual>...</visual> and <collision>...</collision> blocks from links
    urdf = re.sub(r"<visual\b[\s\S]*?</visual>", "", urdf, flags=re.IGNORECASE)
    urdf = re.sub(r"<collision\b[\s\S]*?</collision>", "", urdf, flags=re.IGNORECASE)
    return urdf


def _normalize_whitespace(s: str) -> str:
    # Keep it readable but avoid weird CRLFs / extra indentation
    s = s.replace("\r\n", "\n").replace("\r", "\n")
    # Collapse multiple blank lines
    s = re.sub(r"\n{3,}", "\n\n", s)
    return s.strip() + "\n"


class Ros2ControlDescriptionNode(Node):
    """
    Publishes a ros2_control-friendly robot_description parameter (URDF without visual/collision),
    to avoid YAML parsing issues inside older gazebo_ros2_control when it tries to pass robot_description
    as a parameter override.
    """

    def __init__(self) -> None:
        super().__init__("ros2_control_description")

        # Default path in this repo/container; can be overridden.
        self.declare_parameter("urdf_path", "/ros2_ws/src/amr_cobot/urdf/AMR_COBOT.urdf")

        urdf_path = str(self.get_parameter("urdf_path").value)
        if not os.path.exists(urdf_path):
            self.get_logger().error(f"URDF not found: {urdf_path}")
            urdf_path = ""

        urdf = _read_text(urdf_path) if urdf_path else "<robot name=\"AMR_COBOT\"/>"
        urdf = _strip_xml_decl(urdf)
        urdf = _remove_xml_comments(urdf)
        urdf = _remove_visual_and_collision_blocks(urdf)
        urdf = _normalize_whitespace(urdf)

        # Set the param that gazebo_ros2_control will request
        self.declare_parameter("robot_description", urdf)
        self.get_logger().info(f"ros2_control robot_description ready (len={len(urdf)})")


def main() -> None:
    rclpy.init()
    node = Ros2ControlDescriptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


