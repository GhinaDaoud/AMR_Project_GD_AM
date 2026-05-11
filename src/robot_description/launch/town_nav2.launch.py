"""
town_nav2.launch.py — Phase 2: Autonomous navigation in the town world

Launches:
  - Gazebo with mixed_town.world
  - robot_state_publisher
  - ros_gz topic bridges
  - slam_toolbox (localization mode, loads town_map.posegraph)
  - lifecycle_manager → slam_toolbox           t=5 s
  - Nav2 stack (planner, controller, behavior, bt_navigator)
  - lifecycle_manager → Nav2 nodes             t=10 s
  - mission_planner                            t=12 s
  - rviz2

Run:
  ros2 launch robot_description town_nav2.launch.py

Prerequisites:
  Run town_slam.launch.py first to build town_map.posegraph + town_map.pgm
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
import xacro

def _town_dir() -> str:
    from ament_index_python.packages import get_package_share_directory as gpsd
    share = gpsd('my_local')
    ws_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    return os.path.join(ws_root, 'town')

TOWN_DIR   = _town_dir()
WORLD_FILE = os.path.join(TOWN_DIR, 'mixed_town.world')
MODELS_DIR = os.path.join(TOWN_DIR, 'models')


def _maps_dir():
    from ament_index_python.packages import get_package_share_directory as gpsd
    share = gpsd('my_local')
    ws_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    return os.path.join(ws_root, 'src', 'my_local', 'maps')


def generate_launch_description():

    pkg_path = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'service_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    slam_config = os.path.join(pkg_path, 'config', 'slam_config.yaml')
    nav2_params = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    maps_dir    = _maps_dir()
    posegraph   = os.path.join(maps_dir, 'town_map')   # no extension

    # ── environment ───────────────────────────────────────────────────
    gz_resource_path = ':'.join(filter(None, [
        MODELS_DIR,
        os.path.expanduser('~/.local/share/gz/models'),
        '/usr/share/gz/models',
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))

    os.environ['GZ_SIM_RESOURCE_PATH'] = gz_resource_path
    set_gz_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path)

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

    # ── SLAM toolbox (localization mode) ─────────────────────────────
    slam_localization = TimerAction(period=5.0, actions=[
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config, {
                'use_sim_time': True,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_footprint',
                'scan_topic': '/scan',
                'mode': 'localization',
                'map_file_name': posegraph,
                'map_start_pose': [0.0, 0.0, 0.0],
            }],
        )
    ])

    lifecycle_localization = TimerAction(period=7.0, actions=[
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['slam_toolbox'],
                'bond_timeout': 0.0,
            }],
        )
    ])

    # ── Nav2 stack ────────────────────────────────────────────────────
    nav2_nodes = TimerAction(period=10.0, actions=[
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params],
        ),
    ])

    lifecycle_navigation = TimerAction(period=10.0, actions=[
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }],
        )
    ])

    # ── mission executor (Nav2 action client) ────────────────────────
    mission_planner = TimerAction(period=12.0, actions=[
        Node(
            package='mission_planner',
            executable='mission_planner',
            name='mission_planner',
            output='screen',
        )
    ])

    # ── mission web server (Flask) — open http://localhost:5000
    mission_web = TimerAction(period=15.0, actions=[
        Node(
            package='mission_planner',
            executable='mission_web',
            name='mission_web',
            output='screen',
        )
    ])

    # ── RViz — delayed so localization has time to publish map→odom ──
    rviz_config = os.path.join(pkg_path, 'config', 'nav2.rviz')
    rviz = TimerAction(period=20.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        )
    ])

    return LaunchDescription([
        set_gz_path,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        tf_bridge,
        slam_localization,
        lifecycle_localization,
        nav2_nodes,
        lifecycle_navigation,
        mission_planner,
        mission_web,
        rviz,
    ])