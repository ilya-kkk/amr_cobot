import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _load_file(package: str, relative_path: str) -> str:
    pkg_share = get_package_share_directory(package)
    abs_path = os.path.join(pkg_share, relative_path)
    with open(abs_path, 'r', encoding='utf-8') as f:
        return f.read()


def _load_yaml(package: str, relative_path: str) -> dict:
    pkg_share = get_package_share_directory(package)
    abs_path = os.path.join(pkg_share, relative_path)
    with open(abs_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    robot_description = {
        'robot_description': _load_file('amr_cobot', os.path.join('urdf', 'AMR_COBOT.urdf'))
    }

    robot_description_semantic = {
        'robot_description_semantic': _load_file('amr_cobot_moveit_config', os.path.join('config', 'amr_cobot.srdf'))
    }

    robot_description_kinematics = _load_yaml('amr_cobot_moveit_config', os.path.join('config', 'kinematics.yaml'))

    joint_limits_yaml = _load_yaml('amr_cobot_moveit_config', os.path.join('config', 'joint_limits.yaml'))
    robot_description_planning = {
        'robot_description_planning': joint_limits_yaml
    }

    moveit_controllers = _load_yaml('amr_cobot_moveit_config', os.path.join('config', 'moveit_controllers.yaml'))

    planning_pipelines = _load_yaml('amr_cobot_moveit_config', os.path.join('config', 'planning_pipelines.yaml'))
    ompl_planning = _load_yaml('amr_cobot_moveit_config', os.path.join('config', 'ompl_planning.yaml'))

    # MoveIt expects OMPL params under key "ompl" (same name as pipeline)
    ompl_pipeline_config = {}
    ompl_pipeline_config.update(planning_pipelines.get('ompl', {}))
    ompl_pipeline_config.update(ompl_planning)

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            moveit_controllers,
            {'planning_pipelines': planning_pipelines.get('planning_pipelines', ['ompl'])},
            {'planning_pipeline': 'ompl'},
            {'ompl': ompl_pipeline_config},
        ],
    )

    return LaunchDescription([move_group])
