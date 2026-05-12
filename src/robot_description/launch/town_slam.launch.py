"""
town_slam.launch.py — Phase 1: SLAM mapping + QR detection in the town world

Usage:
  ros2 launch robot_description town_slam.launch.py            # auto exploration
  ros2 launch robot_description town_slam.launch.py mode:=teleop  # keyboard control

Launches:
  - Gazebo with mixed_town.world (models path auto-set)
  - robot_state_publisher
  - ros_gz topic bridges (scan, camera, odom, cmd_vel, tf, clock)
  - slam_toolbox (mapping mode)          t=5 s
  - qr_detector                          t=10 s
  - auto mode: explorer                  t=15 s
  - teleop mode: teleop_twist_keyboard   t=15 s
  - map_auto_saver (saves town_map.*)    t=70 s
  - rviz2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             TimerAction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro


def _maps_dir() -> str:
    from ament_index_python.packages import get_package_share_directory as gpsd
    share = gpsd('my_local')
    ws_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    return os.path.join(ws_root, 'src', 'my_local', 'maps')

def _town_dir() -> str:
    from ament_index_python.packages import get_package_share_directory as gpsd
    share = gpsd('my_local')
    ws_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    return os.path.join(ws_root, 'town')

TOWN_DIR   = _town_dir()
WORLD_FILE = os.path.join(TOWN_DIR, 'mixed_town.world')
MODELS_DIR = os.path.join(TOWN_DIR, 'models')


def generate_launch_description():

    pkg_path = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'service_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    slam_config = os.path.join(pkg_path, 'config', 'slam_config.yaml')

    mode     = LaunchConfiguration('mode')
    cont_map = LaunchConfiguration('continue_mapping')

    # (maps_dir and map_live resolved below, next to slam node definition)

    # ── environment ───────────────────────────────────────────────────
    gz_resource_path = ':'.join(filter(None, [
        MODELS_DIR,
        os.path.expanduser('~/.local/share/gz/models'),
        '/usr/share/gz/models',
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))

    # Set in current process so all child processes inherit it
    os.environ['GZ_SIM_RESOURCE_PATH'] = gz_resource_path
    set_gz_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path)

    declare_continue = DeclareLaunchArgument(
        'continue_mapping',
        default_value='false',
        description=(
            'Set to "true" to resume from the saved posegraph '
            '(town_map_BACKUP). Robot must spawn at the docking station (0,0).'
        ),
    )

    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value='hybrid',
        description=(
            '"hybrid" (default) — explorer runs automatically; '
            'press any key in the xterm window to take over, release to return to auto. '
            '"auto" — explorer only, no keyboard. '
            '"teleop" — keyboard only, no explorer.'
        ),
    )

    # ── core nodes ────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', WORLD_FILE],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'service_robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.15',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='main_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tf_bridge',
        arguments=['/model/service_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        remappings=[('/model/service_robot/tf', '/tf')],
        output='screen',
    )

    # ── SLAM toolbox (mapping mode) ───────────────────────────────────
    maps_dir   = _maps_dir()
    map_live   = os.path.join(maps_dir, 'town_map')

    _slam_base_params = {
        'use_sim_time': True,
        'odom_frame': 'odom',
        'map_frame': 'map',
        'base_frame': 'base_footprint',
        'scan_topic': '/scan',
        'mode': 'mapping',
    }
    # Fresh start — map_file_name explicitly empty so SLAM never auto-loads
    slam_fresh = TimerAction(period=5.0, actions=[
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config, {**_slam_base_params, 'map_file_name': ''}],
            condition=IfCondition(
                PythonExpression(["'", cont_map, "' != 'true'"])
            ),
        )
    ])
    # Continue from latest saved posegraph (town_map — updated every 10 s)
    slam_continue = TimerAction(period=5.0, actions=[
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config, {
                **_slam_base_params,
                'map_file_name': map_live,
                'map_start_at_dock': True,
            }],
            condition=IfCondition(
                PythonExpression(["'", cont_map, "' == 'true'"])
            ),
        )
    ])

    # Lifecycle manager activates slam_toolbox automatically
    lifecycle_slam = TimerAction(period=7.0, actions=[
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['slam_toolbox'],
                'bond_timeout': 0.0,
            }],
        )
    ])

    # ── QR detector ───────────────────────────────────────────────────
    qr_detector = TimerAction(period=10.0, actions=[
        Node(
            package='landmark_detector',
            executable='qr_detector',
            name='qr_detector',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

    is_auto   = PythonExpression(["'", mode, "' in ('auto', 'hybrid')"])
    is_hybrid = PythonExpression(["'", mode, "' == 'hybrid'"])

    # ── explorer (auto + hybrid modes) ───────────────────────────────
    # In hybrid mode it publishes to /cmd_vel_auto; mux decides what reaches /cmd_vel
    explorer = TimerAction(period=15.0, actions=[
        Node(
            package='service_robot',
            executable='explorer',
            name='explorer',
            output='screen',
            condition=IfCondition(is_auto),
        )
    ])

    # ── keyboard — opens in its own xterm window ──────────────────────
    # teleop mode  → publishes directly to /cmd_vel
    # hybrid mode  → publishes to /cmd_vel_teleop (mux blends it with explorer)
    teleop = TimerAction(period=15.0, actions=[
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e' if os.environ.get('DISPLAY') else '',
            remappings=[('/cmd_vel', '/cmd_vel_teleop')],
            condition=IfCondition(is_hybrid),
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e' if os.environ.get('DISPLAY') else '',
            condition=IfCondition(
                PythonExpression(["'", mode, "' == 'teleop'"])
            ),
        ),
    ])

    # ── mux — hybrid mode only ────────────────────────────────────────
    mux = TimerAction(period=15.0, actions=[
        Node(
            package='service_robot',
            executable='teleop_mux',
            name='teleop_mux',
            output='screen',
            condition=IfCondition(is_hybrid),
        )
    ])


    # ── RViz — delayed so SLAM has time to publish map→odom first ────
    rviz_config = os.path.join(pkg_path, 'config', 'slam.rviz')
    rviz = TimerAction(period=15.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        )
    ])

    return LaunchDescription([
        declare_continue,
        declare_mode,
        set_gz_path,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        tf_bridge,
        slam_fresh,
        slam_continue,
        mux,
        lifecycle_slam,
        qr_detector,
        explorer,
        teleop,
        rviz,
    ])