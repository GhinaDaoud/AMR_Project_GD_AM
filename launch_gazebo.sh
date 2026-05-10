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

# Kill any leftover gz/ign processes from a previous crash (segfault leaves zombies
# that hold the shared-memory transport port and prevent a clean restart).
pkill -9 -x gz    2>/dev/null || true
pkill -9 -x ign   2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
sleep 0.5

# Clean up Gazebo shared-memory / lock files that survive a hard crash.
rm -rf /tmp/gz_transport* /tmp/ign_transport* 2>/dev/null || true

# ogre1 + hardware Mesa (d3d12): functional, fast.
# Known issues: scan-line artifacts in WSL2, SIGSEGV on exit (cosmetic — stderr silenced).
echo "Renderer: ogre1 + Mesa d3d12 (hardware)."
gz sim --render-engine ogre -r "$WORLD_FILE" 2>/dev/null || true