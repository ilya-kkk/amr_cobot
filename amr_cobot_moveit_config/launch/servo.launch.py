import os

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _read_text(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def generate_launch_description():
    amr_share = FindPackageShare("amr_cobot").find("amr_cobot")
    moveit_share = FindPackageShare("amr_cobot_moveit_config").find("amr_cobot_moveit_config")

    urdf_path = os.path.join(amr_share, "urdf", "AMR_COBOT.urdf")
    srdf_path = os.path.join(moveit_share, "config", "amr_cobot.srdf")

    robot_description = {"robot_description": _read_text(urdf_path)}
    robot_description_semantic = {"robot_description_semantic": _read_text(srdf_path)}

    kinematics_yaml = os.path.join(moveit_share, "config", "kinematics.yaml")
    joint_limits_yaml = os.path.join(moveit_share, "config", "joint_limits.yaml")
    servo_yaml = os.path.join(moveit_share, "config", "servo.yaml")

    # Merge partial /joint_states into a complete-enough joint stream for MoveIt/Servo.
    # This avoids "Missing ... joint" and ensures Servo sees one consistent JointState message.
    joint_state_mux = Node(
        package="amr_cobot_teleop",
        executable="joint_state_mux_node",
        name="joint_state_mux_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "in_topic": "/joint_states",
            "out_topic": "/joint_states_mux",
        }],
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            servo_yaml,
            {"use_sim_time": True},
        ],
    )

    # Start servo after node is up (MoveIt Servo requires explicit start service).
    # Use an rclpy client node instead of ros2cli to avoid daemon/CLI issues in containers.
    start_servo_client = Node(
        package="amr_cobot_teleop",
        executable="servo_start_client_node",
        name="servo_start_client_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "service_name": "/servo_node/start_servo",
            "timeout_sec": 30.0,
        }],
    )

    delayed_start_servo = TimerAction(period=2.0, actions=[start_servo_client])

    return LaunchDescription([joint_state_mux, servo_node, delayed_start_servo])


