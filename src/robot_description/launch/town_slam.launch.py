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

    mode = LaunchConfiguration('mode')

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

    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value='auto',
        description='Exploration mode: "auto" (explorer node) or "teleop" (keyboard)',
    )

    # ── core nodes ────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # Combined server+GUI using ogre2 (Gazebo Harmonic default).
    # ogre2 handles GLB meshes correctly; ogre1 was what crashed before.
    # Combined mode gives gpu_lidar and camera sensors a proper GL context.
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
    slam = TimerAction(period=5.0, actions=[
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
                'mode': 'mapping',
            }],
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
        )
    ])

    is_auto   = PythonExpression(["'", mode, "' == 'auto'"])
    is_teleop = PythonExpression(["'", mode, "' == 'teleop'"])

    # ── auto mode: explorer ───────────────────────────────────────────
    explorer = TimerAction(period=15.0, actions=[
        Node(
            package='service_robot',
            executable='explorer',
            name='explorer',
            output='screen',
            condition=IfCondition(is_auto),
        )
    ])

    # ── teleop mode: keyboard (opens in its own xterm window) ────────
    teleop = TimerAction(period=15.0, actions=[
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e' if os.environ.get('DISPLAY') else '',
            condition=IfCondition(is_teleop),
        )
    ])

    # ── map auto saver (saves town_map.*) ────────────────────────────
    map_saver = TimerAction(period=70.0, actions=[
        Node(
            package='service_robot',
            executable='map_auto_saver',
            name='map_auto_saver',
            output='screen',
            parameters=[{'map_name': 'town_map'}],
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
        declare_mode,
        set_gz_path,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        tf_bridge,
        slam,
        lifecycle_slam,
        qr_detector,
        explorer,
        teleop,
        map_saver,
        rviz,
    ])