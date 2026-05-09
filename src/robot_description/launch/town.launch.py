"""
town.launch.py — Phase 1 SLAM exploration on the mixed_town world
Package : robot_description
Usage   : ros2 launch robot_description town.launch.py [mode:=auto|teleop]

Differences from slam.launch.py (maze)
---------------------------------------
  World      mixed_town.world at workspace root  (not maze.sdf)
  Models     GZ_SIM_RESOURCE_PATH includes <workspace>/models/
  Spawn      (2, 2, 0.15) — open road just outside the docking platform
  Bridge     world name is mixed_town_v2  (maze → mixed_town_v2)

Everything else is identical: same SLAM config, same explorer, same
map_auto_saver, same QR detector, same RViz config, same MAPS_DIR output.

Output files land in the same src/my_local/maps/ folder.
Run this to validate Phase 1 exploration on the town before adding QR codes.

Debug tips
----------
  Node list  : ros2 node list
  TF tree    : ros2 run tf2_tools view_frames
  Map topic  : ros2 topic echo /map --no-arr
  Drive      : ros2 run teleop_twist_keyboard teleop_twist_keyboard  (teleop mode)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    pkg_path = get_package_share_directory('robot_description')

    # Workspace root — same resolution trick as slam.launch.py
    _share = get_package_share_directory('my_local')
    _ws_root = os.path.abspath(os.path.join(_share, '..', '..', '..', '..'))
    MAPS_DIR = os.path.join(_ws_root, 'src', 'my_local', 'maps')
    os.makedirs(MAPS_DIR, exist_ok=True)

    world_file = os.path.join(_ws_root, 'mixed_town.world')
    models_dir = os.path.join(_ws_root, 'models')

    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value='auto',
        description="'auto' starts the explorer; 'teleop' leaves /cmd_vel free"
    )
    mode = LaunchConfiguration('mode')

    xacro_file = os.path.join(pkg_path, 'urdf', 'service_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    slam_config = os.path.join(pkg_path, 'config', 'slam_config.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}]
    )

    # GZ_SIM_RESOURCE_PATH must contain the models/ directory so that
    # model://sun, model://bench, model://supermarket, etc. resolve correctly.
    existing_gz = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = ':'.join(p for p in [models_dir, existing_gz] if p)

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '1', world_file],
        additional_env={'GZ_SIM_RESOURCE_PATH': gz_resource_path},
        output='screen'
    )

    # Spawn at (2, 2) — open road just outside the 3x3m docking platform
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'service_robot',
                    '-topic', 'robot_description',
                    '-x', '2.0', '-y', '2.0', '-z', '0.15'
                ],
                output='screen'
            )
        ]
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
        output='screen'
    )

    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tf_bridge',
        arguments=[
            '/model/service_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[('/model/service_robot/tf', '/tf')],
        output='screen'
    )

    # World name in mixed_town.world is "mixed_town_v2"
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        arguments=[
            '/world/mixed_town_v2/model/service_robot/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/mixed_town_v2/model/service_robot/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    slam = TimerAction(
        period=8.0,
        actions=[
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
                }]
            )
        ]
    )

    explorer = TimerAction(
        period=12.0,
        condition=IfCondition(PythonExpression(["'", mode, "' == 'auto'"])),
        actions=[
            Node(
                package='service_robot',
                executable='explorer',
                name='explorer',
                output='screen',
            )
        ]
    )

    qr_detector = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='landmark_detector',
                executable='qr_detector',
                name='qr_detector',
                output='screen',
            )
        ]
    )

    map_auto_saver = TimerAction(
        period=70.0,
        actions=[
            Node(
                package='service_robot',
                executable='map_auto_saver',
                name='map_auto_saver',
                output='screen',
            )
        ]
    )

    rviz_config = os.path.join(pkg_path, 'config', 'slam.rviz')
    rviz = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        declare_mode,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        tf_bridge,
        joint_state_bridge,
        slam,
        explorer,
        qr_detector,
        map_auto_saver,
        rviz,
    ])
