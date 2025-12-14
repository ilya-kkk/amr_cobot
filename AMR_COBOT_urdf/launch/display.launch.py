from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare(package='amr_cobot').find('amr_cobot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'AMR_COBOT.urdf')
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2
    rviz_config = os.path.join(pkg_share, 'urdf', 'AMR_COBOT.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui,
        rviz2,
    ])
