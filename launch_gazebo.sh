#!/bin/bash
# Launch Gazebo with the mixed_town world.
# Run from the Project directory:  bash launch_gazebo.sh

#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODELS_DIR="$SCRIPT_DIR/models"
WORLD_FILE="$SCRIPT_DIR/mixed_town.world"

# Ignition Gazebo path - include local models + standard Gazebo model directories
# Search common ROS 2 Gazebo model locations
PREV_GZ_PATH="$GZ_SIM_RESOURCE_PATH"
export GZ_SIM_RESOURCE_PATH="$MODELS_DIR:$HOME/.local/share/gz/models:/usr/share/gz/models"

# Add ROS 2 Gazebo models (for Jazzy and other distros)
if [ -d "/opt/ros" ]; then
  for ros_path in /opt/ros/*/share/gz-sim*/models; do
    [ -d "$ros_path" ] && export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$ros_path"
  done
  for ros_path in /opt/ros/*/share/gazebo*/models; do
    [ -d "$ros_path" ] && export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$ros_path"
  done
fi

# Append pre-existing paths
[ -n "$PREV_GZ_PATH" ] && export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$PREV_GZ_PATH"

echo "GZ_SIM_RESOURCE_PATH = $GZ_SIM_RESOURCE_PATH"
echo "Launching: $WORLD_FILE"

gz sim --render-engine ogre2 "$WORLD_FILE"

# For ROS 2 + Gazebo (uncomment and adapt):
# ros2 launch gazebo_ros gazebo.launch.py world:="$WORLD_FILE"