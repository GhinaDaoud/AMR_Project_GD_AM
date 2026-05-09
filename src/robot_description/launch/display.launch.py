"""
display.launch.py — Minimal robot visualisation (no SLAM, no maze)
Package : robot_description
Usage   : ros2 launch robot_description display.launch.py

What it starts
--------------
  robot_state_publisher — publishes URDF transforms
  gz sim empty.sdf      — Gazebo with an empty world (no maze)
  ros_gz_sim create     — spawns robot at (0, 0)
  parameter_bridge      — bridges scan, camera, cmd_vel, odom, clock

Use this when
-------------
  - Checking the URDF model geometry / joint setup
  - Testing sensor bridges without the maze world
  - Quick sanity-check after modifying service_robot.urdf.xacro

Debug tips
----------
  View model  : open RViz2 manually → add RobotModel display, topic /robot_description
  Check TF    : ros2 run tf2_tools view_frames
  Drive robot : ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_path = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'service_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}]
    )

    # Launch Gazebo empty world
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    # Spawn robot into Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'service_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Bridge Gazebo topics to ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
    ])
