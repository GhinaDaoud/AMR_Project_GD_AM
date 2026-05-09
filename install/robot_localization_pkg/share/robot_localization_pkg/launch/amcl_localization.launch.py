from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # --- 1. SETTINGS & PATHS ---
    # Define the name of your ROS2 package
    pkg_name = "robot_localization_pkg"
    
    # Locate where the package is installed on your computer
    pkg_share = get_package_share_directory(pkg_name)

    # Construct the full paths to your map (YAML) and configuration (AMCL) files
    map_file = os.path.join(pkg_share, "maps", "room_map.yaml")
    params_file = os.path.join(pkg_share, "config", "amcl_params.yaml")

    # --- 2. MAP SERVER NODE ---
    # This node loads the 2D image (PGM) and serves it to other nodes as a 'map' topic
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            params_file,               # Load general settings from your YAML
            {"yaml_filename": map_file} # Specifically tell it which map file to open
        ]
    )

    # --- 3. AMCL NODE ---
    # The Adaptive Monte Carlo Localization node (the "brain" that locates the robot)
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[params_file]       # Load sensor/odom noise parameters from your YAML
    )

    # --- 4. LIFECYCLE MANAGER NODE ---
    # CRITICAL: Nav2 nodes use a 'lifecycle' system. They start in an unconfigured state.
    # This node sends the signal to Map Server and AMCL to "Wake Up" and start working.
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[params_file]       # Uses 'autostart' and 'node_names' from your YAML
    )

    # --- 5. EXECUTION ---
    # Return the list of nodes so the 'ros2 launch' command can run them all at once
    return LaunchDescription([
        map_server,
        amcl,
        lifecycle_manager
    ])
