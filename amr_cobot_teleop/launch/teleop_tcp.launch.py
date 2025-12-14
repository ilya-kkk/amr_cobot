import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("amr_cobot_teleop").find("amr_cobot_teleop")
    params = os.path.join(share, "config", "teleop_tcp.yaml")

    teleop = Node(
        package="amr_cobot_teleop",
        executable="teleop_tcp_node",
        name="teleop_tcp_node",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription([teleop])


