"""
nav2.launch.py — Part 2 launch file (navigation to landmarks)
Package : robot_description
Usage   : ros2 launch robot_description nav2.launch.py

What it starts (in order)
--------------------------
  t= 0s  robot_state_publisher  — URDF transforms
  t= 0s  gz sim                 — Gazebo with maze.sdf (QR paths patched)
  t= 0s  ros-gz bridges         — scan, camera, cmd_vel, odom, clock, tf, joint_states
  t= 5s  ros_gz_sim create      — spawns robot at (-7, -7)
  t= 8s  slam_toolbox           — localization mode: loads Part 1 posegraph,
                                   publishes full /map, provides map→odom TF
                                   via live scan matching (no initial pose needed)
  t=12s  planner_server         — NavFn global planner
  t=12s  controller_server      — Regulated Pure Pursuit local controller
  t=12s  bt_navigator           — behaviour-tree navigation executor
  t=12s  behavior_server        — spin / back-up / wait recovery behaviours
  t=14s  lifecycle_manager_navigation  — activates the four nav2 nodes above
  t=16s  mission_planner        — reads landmark_db.json, accepts /mission_goal
  t=10s  rviz2                  — visualiser (map, paths, laser)

Send navigation goals
---------------------
  ros2 topic pub --once /mission_goal std_msgs/msg/String \
    "data: 'delivery:Zone-C: pillar B (0, -4)'"

  ros2 topic pub --once /mission_goal std_msgs/msg/String \
    "data: 'delivery:Zone-B: pillar A (0, 3)'"
"""

import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    pkg_robot = get_package_share_directory('robot_description')

    # ── paths ────────────────────────────────────────────────────────── #
    _share   = get_package_share_directory('my_local')
    _ws_root = os.path.abspath(os.path.join(_share, '..', '..', '..', '..'))
    maps_dir = os.path.join(_ws_root, 'src', 'my_local', 'maps')

    nav2_params  = os.path.join(pkg_robot, 'config', 'nav2_params.yaml')
    rviz_config  = os.path.join(pkg_robot, 'config', 'nav2.rviz')
    slam_config  = os.path.join(pkg_robot, 'config', 'slam_config.yaml')
    xacro_file   = os.path.join(pkg_robot, 'urdf', 'service_robot.urdf.xacro')

    robot_description = xacro.process_file(xacro_file).toxml()

    # ── patch maze.sdf QR texture paths ─────────────────────────────── #
    qr_textures_dir = os.path.join(pkg_robot, 'worlds', 'qr_textures')
    maze_template   = os.path.join(pkg_robot, 'worlds', 'maze.sdf')
    with open(maze_template) as f:
        sdf_content = f.read()
    sdf_content = sdf_content.replace(
        '/home/test/AMR_project/src/robot_description/worlds/qr_textures',
        qr_textures_dir
    )
    with tempfile.NamedTemporaryFile(mode='w', suffix='_maze.sdf', delete=False) as tf:
        tf.write(sdf_content)
        maze_patched = tf.name

    # ── simulation ───────────────────────────────────────────────────── #
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    worlds_dir = os.path.join(pkg_robot, 'worlds')
    existing_gz = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource = worlds_dir + ':' + existing_gz if existing_gz else worlds_dir

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '1', maze_patched],
        additional_env={'GZ_SIM_RESOURCE_PATH': gz_resource},
        output='screen',
    )

    spawn_robot = TimerAction(period=5.0, actions=[
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'service_robot', '-topic', 'robot_description',
                       '-x', '-7.0', '-y', '-7.0', '-z', '0.15'],
            output='screen',
        )
    ])

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

    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        arguments=[
            '/world/maze/model/service_robot/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[('/world/maze/model/service_robot/joint_state', '/joint_states')],
        output='screen',
    )

    # ── localisation — slam_toolbox in localization mode ────────────── #
    # Loads the Part 1 posegraph (.posegraph + .data), publishes the full
    # /map, and provides the map→odom TF via live scan matching.
    # No initial pose click needed — scan matching handles it.
    slam_localization = TimerAction(period=8.0, actions=[
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config, {
                'use_sim_time': True,
                'mode': 'localization',
                'map_file_name': os.path.join(maps_dir, 'maze_map'),
                'map_start_at_dock': True,
            }],
        )
    ])

    lifecycle_slam = TimerAction(period=10.0, actions=[
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['slam_toolbox'],
            }],
            output='screen',
        )
    ])

    # ── navigation (planner + controller + bt + behaviors) ──────────── #
    planner_server = TimerAction(period=12.0, actions=[
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_params, {'use_sim_time': True}],
            output='screen',
        )
    ])

    controller_server = TimerAction(period=12.0, actions=[
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[nav2_params, {'use_sim_time': True}],
            output='screen',
        )
    ])

    bt_navigator = TimerAction(period=12.0, actions=[
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[nav2_params, {'use_sim_time': True}],
            output='screen',
        )
    ])

    behavior_server = TimerAction(period=12.0, actions=[
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[nav2_params, {'use_sim_time': True}],
            output='screen',
        )
    ])

    lifecycle_navigation = TimerAction(period=14.0, actions=[
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
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
            output='screen',
        )
    ])

    # ── mission planner ──────────────────────────────────────────────── #
    mission_planner = TimerAction(period=16.0, actions=[
        Node(
            package='mission_planner',
            executable='mission_planner',
            name='mission_planner',
            parameters=[{'use_sim_time': True}],
            output='screen',
        )
    ])

    # ── visualisation ────────────────────────────────────────────────── #
    rviz = TimerAction(period=10.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        )
    ])

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        tf_bridge,
        joint_state_bridge,
        slam_localization,
        lifecycle_slam,
        planner_server,
        controller_server,
        bt_navigator,
        behavior_server,
        lifecycle_navigation,
        mission_planner,
        rviz,
    ])
