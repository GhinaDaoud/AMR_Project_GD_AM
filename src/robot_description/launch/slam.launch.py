"""
slam.launch.py — Main launch file for Phase 1 (SLAM + exploration)
Package : robot_description
Usage   : ros2 launch robot_description slam.launch.py [mode:=auto|teleop]

What it starts (in order)
--------------------------
  t= 0s  robot_state_publisher  — publishes URDF transforms (/tf_static)
  t= 0s  gz sim                 — Gazebo Harmonic with maze.sdf world
  t= 0s  parameter_bridge       — bridges Gazebo↔ROS2 topics (scan, camera,
                                   cmd_vel, odom, clock, tf, joint_states)
  t= 5s  ros_gz_sim create      — spawns robot at (-7, -7) in the maze
  t= 8s  slam_toolbox           — starts SLAM mapping (/map + /tf map→odom)
  t=10s  rviz2                  — visualiser (map, robot model, laser scan)
  t=12s  explorer (auto only)   — reactive obstacle-avoidance driver
  t=12s  qr_detector            — QR landmark detection + landmark_db.json
  t=70s  map_auto_saver         — saves map files every 60 s

Launch argument
---------------
  mode:=auto    (default) explorer drives autonomously
  mode:=teleop  explorer is NOT started; run teleop_twist_keyboard separately
                  ros2 run teleop_twist_keyboard teleop_twist_keyboard

Key paths resolved at launch time
----------------------------------
  MAPS_DIR      <workspace>/src/my_local/maps/   — all output files land here
  maze.sdf      patched in a tempfile so QR texture paths match this machine

Debug tips
----------
  Node list      : ros2 node list
  Topic list     : ros2 topic list
  TF tree        : ros2 run tf2_tools view_frames
  SLAM status    : ros2 topic echo /slam_toolbox/feedback
  Map topic      : ros2 topic echo /map --no-arr
  Restart a node : ros2 run <package> <executable>
"""

import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    pkg_path = get_package_share_directory('robot_description')

    # ── mode argument ────────────────────────────────────────────────────── #
    #   auto  : explorer drives autonomously (default)
    #   teleop: explorer is NOT started; drive with teleop_twist_keyboard
    #           ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # ──────────────────────────────────────────────────────────────────────── #
    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value='auto',
        description="'auto' starts the explorer node; "
                    "'teleop' leaves /cmd_vel free for keyboard control"
    )
    mode = LaunchConfiguration('mode')

    # Maps directory — resolve from install path so it always points to the
    # correct Windows-filesystem location regardless of WSL username.
    _share = get_package_share_directory('my_local')
    _ws_root = os.path.abspath(os.path.join(_share, '..', '..', '..', '..'))
    MAPS_DIR = os.path.join(_ws_root, 'src', 'my_local', 'maps')
    os.makedirs(MAPS_DIR, exist_ok=True)

    # Patch QR texture paths in maze.sdf at launch time so the file works
    # regardless of which Linux username is running it.
    qr_textures_dir = os.path.join(pkg_path, 'worlds', 'qr_textures')
    maze_template = os.path.join(pkg_path, 'worlds', 'maze.sdf')
    with open(maze_template) as f:
        sdf_content = f.read()
    sdf_content = sdf_content.replace(
        '/home/test/AMR_project/src/robot_description/worlds/qr_textures',
        qr_textures_dir
    )
    maze_world_patched = tempfile.mktemp(suffix='_maze.sdf')
    with open(maze_world_patched, 'w') as f:
        f.write(sdf_content)

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

    worlds_dir = os.path.join(pkg_path, 'worlds')
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = worlds_dir + ':' + existing_gz_path if existing_gz_path else worlds_dir

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '1', maze_world_patched],
        additional_env={'GZ_SIM_RESOURCE_PATH': gz_resource_path},
        output='screen'
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'service_robot',
                    '-topic', 'robot_description',
                    '-x', '-7.0', '-y', '-7.0', '-z', '0.15'
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

    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        arguments=[
            '/world/maze/model/service_robot/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/maze/model/service_robot/joint_state', '/joint_states'),
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

    # Explorer only in 'auto' mode
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

    # QR detector always runs (both modes need landmark detection)
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

    # Periodic map saver: saves occupancy grid (.pgm+.yaml) and posegraph
    # (.posegraph+.data) every 60 s. Starts at t=70s so SLAM has had time to
    # build an initial map before the first save fires.
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
