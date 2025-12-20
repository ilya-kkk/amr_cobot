from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import re
import json
import time
import urllib.request


def generate_launch_description():
    # region agent log
    def _agent_log(hypothesisId: str, location: str, message: str, data: dict, runId: str = "pre-fix"):
        try:
            payload = {
                "sessionId": "debug-session",
                "runId": runId,
                "hypothesisId": hypothesisId,
                "location": location,
                "message": message,
                "data": data,
                "timestamp": int(time.time() * 1000),
            }
            req = urllib.request.Request(
                "http://127.0.0.1:7242/ingest/688c79a6-a285-4d94-ab8b-caee0a2ae45a",
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            urllib.request.urlopen(req, timeout=0.4).read()
        except Exception:
            pass
    # endregion agent log

    # Get the package directory
    pkg_share = FindPackageShare(package='amr_cobot')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'AMR_COBOT.urdf'])

    gui_cfg = LaunchConfiguration('gui')

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo client GUI (gzclient). If false, runs headless (gzserver only).'
    )
    
    # Read URDF file content for robot_state_publisher
    # Try multiple possible paths
    urdf_paths = [
        '/ros2_ws/src/amr_cobot/urdf/AMR_COBOT.urdf',  # Source directory (mounted)
        '/ros2_ws/install/amr_cobot/share/amr_cobot/urdf/AMR_COBOT.urdf',  # Install directory
    ]

    # region agent log
    _agent_log(
        "H_entry",
        "AMR_COBOT_urdf/launch/gazebo.launch.py:generate_launch_description",
        "launch entry",
        {
            "urdf_paths": urdf_paths,
            "cwd": os.getcwd(),
            "display": os.environ.get("DISPLAY"),
            "xauthority": os.environ.get("XAUTHORITY"),
        },
    )
    # endregion agent log

    # region agent log
    try:
        _xauth_path = os.environ.get("XAUTHORITY") or "/tmp/.docker.xauth"
        _xauth_exists = os.path.exists(_xauth_path)
        _xauth_size = os.path.getsize(_xauth_path) if _xauth_exists else 0
        _agent_log(
            "H_x11",
            "AMR_COBOT_urdf/launch/gazebo.launch.py:x11",
            "x11 env snapshot",
            {
                "DISPLAY": os.environ.get("DISPLAY"),
                "XAUTHORITY": _xauth_path,
                "xauth_exists": _xauth_exists,
                "xauth_size": _xauth_size,
            },
            runId="post-fix",
        )
    except Exception as _e:
        _agent_log(
            "H_x11_err",
            "AMR_COBOT_urdf/launch/gazebo.launch.py:x11",
            "failed reading xauth info",
            {"err": str(_e)},
            runId="post-fix",
        )
    # endregion agent log

    def _sanitize_urdf_text(s: str) -> str:
        # Remove UTF-8 BOM if present (rare, but safe) and strip XML declaration.
        s2 = s.lstrip("\ufeff")
        # lxml.fromstring() errors on unicode strings containing encoding declarations.
        s2 = re.sub(r'^\s*<\?xml[^>]*\?>\s*', '', s2, count=1, flags=re.IGNORECASE)
        return s2.lstrip()
    
    robot_desc = ''
    sanitized_urdf_path = ''
    for path in urdf_paths:
        if os.path.exists(path):
            try:
                # region agent log
                try:
                    with open(path, "rb") as _fpb:
                        _head = _fpb.read(120)
                    _agent_log(
                        "H_urdf_head",
                        "AMR_COBOT_urdf/launch/gazebo.launch.py:urdf_read",
                        "urdf head bytes",
                        {
                            "path": path,
                            "exists": True,
                            "head_hex": _head[:80].hex(),
                            "has_xml_decl": b"<?xml" in _head.lower(),
                            "has_encoding": b"encoding" in _head.lower(),
                            "has_crlf": b"\r\n" in _head,
                            "has_bom_utf8": _head.startswith(b"\xef\xbb\xbf"),
                        },
                    )
                except Exception as _e:
                    _agent_log(
                        "H_urdf_head_err",
                        "AMR_COBOT_urdf/launch/gazebo.launch.py:urdf_read",
                        "failed reading urdf bytes head",
                        {"path": path, "err": str(_e)},
                    )
                # endregion agent log

                with open(path, 'r') as infp:
                    robot_desc = infp.read()

                # Sanitize URDF for both robot_state_publisher and spawn_entity.py
                robot_desc_sanitized = _sanitize_urdf_text(robot_desc)
                if robot_desc_sanitized != robot_desc:
                    try:
                        sanitized_urdf_path = "/tmp/AMR_COBOT.sanitized.urdf"
                        with open(sanitized_urdf_path, "w", encoding="utf-8", newline="\n") as outfp:
                            outfp.write(robot_desc_sanitized)
                        robot_desc = robot_desc_sanitized

                        # region agent log
                        _agent_log(
                            "H_sanitize_written",
                            "AMR_COBOT_urdf/launch/gazebo.launch.py:sanitize",
                            "sanitized URDF written to tmp and robot_desc replaced",
                            {"src_path": path, "sanitized_path": sanitized_urdf_path, "len": len(robot_desc)},
                            runId="post-fix",
                        )
                        # endregion agent log
                    except Exception as _e:
                        sanitized_urdf_path = ""
                        # region agent log
                        _agent_log(
                            "H_sanitize_write_err",
                            "AMR_COBOT_urdf/launch/gazebo.launch.py:sanitize",
                            "failed writing sanitized URDF",
                            {"src_path": path, "err": str(_e)},
                            runId="post-fix",
                        )
                        # endregion agent log
                break
            except Exception as e:
                continue
    
    # Fallback: try FindPackageShare
    if not robot_desc:
        try:
            urdf_path = os.path.join(FindPackageShare(package='amr_cobot').find('amr_cobot'), 'urdf', 'AMR_COBOT.urdf')
            with open(urdf_path, 'r') as infp:
                robot_desc = infp.read()
            robot_desc = _sanitize_urdf_text(robot_desc)
        except:
            pass

    # region agent log
    _agent_log(
        "H_robot_desc",
        "AMR_COBOT_urdf/launch/gazebo.launch.py:robot_desc",
        "robot_desc loaded",
        {
            "loaded": bool(robot_desc),
            "len": len(robot_desc) if robot_desc else 0,
            "starts_with_xml_decl": robot_desc.lstrip().startswith("<?xml") if robot_desc else False,
            "contains_encoding_decl": ("encoding=" in robot_desc[:200].lower()) if robot_desc else False,
        },
    )
    # endregion agent log

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # A dedicated node holding a ros2_control-friendly robot_description (no visual/collision),
    # used by gazebo_ros2_control to avoid parameter parsing issues.
    ros2_control_description_node = Node(
        package='amr_cobot',
        executable='ros2_control_description_node',
        name='ros2_control_description',
        output='screen',
        parameters=[{
            'urdf_path': urdf_abs_path if 'urdf_abs_path' in locals() else '/ros2_ws/src/amr_cobot/urdf/AMR_COBOT.urdf'
        }]
    )

    # Static transform publisher between base_footprint and base_link.
    # IMPORTANT: diff_drive publishes odom -> base_footprint, so base_footprint must be parent of base_link
    # to avoid two parents for base_footprint.
    tf_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['--frame-id', 'base_footprint', '--child-frame-id', 'base_link']
    )

    # Gazebo Classic MUST be started with GazeboRosFactory, otherwise /spawn_entity service won't exist.
    # We launch gzserver/gzclient explicitly with required plugins to avoid silent failures.
    #
    # Prefer a project world that already includes fixed CNC; fallback to gazebo_ros empty.world.
    world_candidates = [
        '/ros2_ws/src/amr_cobot/worlds/amr_cobot_with_cnc.world',
        '/ros2_ws/install/amr_cobot/share/amr_cobot/worlds/amr_cobot_with_cnc.world',
    ]
    world_abs = ""
    for p in world_candidates:
        if os.path.exists(p):
            world_abs = p
            break
    world_path = world_abs if world_abs else PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'worlds', 'empty.world'])

    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            world_path,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen',
    )

    gzclient = ExecuteProcess(
        condition=IfCondition(gui_cfg),
        cmd=['gzclient', '--verbose'],
        output='screen',
    )

    # Spawn model in Gazebo - use file directly instead of topic
    # Use the same paths as above
    urdf_abs_path = urdf_paths[0]  # Default to source
    if sanitized_urdf_path and os.path.exists(sanitized_urdf_path):
        urdf_abs_path = sanitized_urdf_path
    for path in urdf_paths:
        if os.path.exists(path):
            if sanitized_urdf_path and os.path.exists(sanitized_urdf_path):
                urdf_abs_path = sanitized_urdf_path
            else:
                urdf_abs_path = path
            break

    # region agent log
    _agent_log(
        "H_spawn_args",
        "AMR_COBOT_urdf/launch/gazebo.launch.py:spawn_model",
        "spawn_entity arguments selected",
        {"urdf_abs_path": urdf_abs_path, "spawn_mode": "file", "using_sanitized": bool(sanitized_urdf_path)},
    )
    # endregion agent log
    
    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_model',
        arguments=['-entity', 'AMR_COBOT', '-file', urdf_abs_path],
        output='screen'
    )

    # Delay spawn_model to ensure Gazebo is ready
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_model]
    )


    # Joint state publisher (non-GUI version for headless mode)
    # NOTE: In Gazebo + ros2_control we rely on joint_state_broadcaster for /joint_states.
    # Running joint_state_publisher here would conflict and publish fake states.

    # RViz visualization tool
    rviz_config_candidates = [
        '/ros2_ws/src/amr_cobot/urdf/AMR_COBOT.rviz',
        '/ros2_ws/install/amr_cobot/share/amr_cobot/urdf/AMR_COBOT.rviz',
    ]
    rviz_config = ''
    for p in rviz_config_candidates:
        if os.path.exists(p):
            rviz_config = p
            break

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if rviz_config else [],
        # RViz RobotModel display reads robot_description from its own node parameters in ROS2.
        # Provide it explicitly to avoid "Error document empty" when RobotModel tries to parse an unset parameter.
        parameters=[{'robot_description': robot_desc}],
    )

    # rqt_graph (for demo visibility)
    rqt_graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph',
        output='screen'
    )

    # ros2_control controllers (spawn after the robot entity exists in Gazebo)
    arm_controller_params = PathJoinSubstitution([pkg_share, 'config', 'arm_velocity_controller.yaml'])

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    arm_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='arm_velocity_controller_spawner',
        arguments=[
            'arm_velocity_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    delayed_controllers = TimerAction(
        period=8.0,
        actions=[
            joint_state_broadcaster_spawner,
            arm_velocity_controller_spawner,
        ],
    )

    launch_actions = [
        gui_arg,
        robot_state_publisher_node,
        tf_footprint_base,
        gzserver,
        gzclient,
        delayed_spawn,
        delayed_controllers,
        rviz_node,
        rqt_graph_node,
    ]

    if ros2_control_description_node is not None:
        launch_actions.insert(2, ros2_control_description_node)

    return LaunchDescription(launch_actions)
