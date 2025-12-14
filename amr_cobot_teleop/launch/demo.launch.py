import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    amr_share = FindPackageShare("amr_cobot").find("amr_cobot")
    moveit_share = FindPackageShare("amr_cobot_moveit_config").find("amr_cobot_moveit_config")
    teleop_share = FindPackageShare("amr_cobot_teleop").find("amr_cobot_teleop")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(amr_share, "launch", "gazebo.launch.py")),
        launch_arguments={"gui": "true"}.items(),
    )

    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_share, "launch", "servo.launch.py"))
    )

    joy = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    teleop_params = os.path.join(teleop_share, "config", "teleop_tcp.yaml")
    teleop = Node(
        package="amr_cobot_teleop",
        executable="teleop_tcp_node",
        name="teleop_tcp_node",
        output="screen",
        parameters=[teleop_params],
    )

    # Bring up servo after Gazebo + controllers are likely ready
    delayed_servo_stack = TimerAction(period=10.0, actions=[servo_launch, teleop])

    return LaunchDescription([gazebo_launch, joy, delayed_servo_stack])


