#!/usr/bin/env bash
# One-time setup: installs dependencies and builds the workspace.
# Run inside Ubuntu 24.04 (WSL2) with ROS 2 Jazzy installed:
#     cd /mnt/c/Users/aswin/Documents/Robotics && bash setup.sh
set -e

source /opt/ros/jazzy/setup.bash

echo ">> Installing Nav2, TurtleBot3 sim (Gazebo Harmonic) and ros_gz ..."
sudo apt update
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-minimal-tb3-sim \
  ros-jazzy-nav2-simple-commander \
  ros-jazzy-ros-gz \
  ros-jazzy-xacro \
  python3-colcon-common-extensions

# Strip Windows CRLF line endings in case the files were edited on Windows
find tb3_nav_ws/src -type f \( -name '*.py' -o -name '*.yaml' -o -name '*.xacro' -o -name '*.txt' -o -name '*.xml' \) \
  -exec sed -i 's/\r$//' {} +

echo ">> Building workspace ..."
cd tb3_nav_ws
colcon build --symlink-install
echo
echo "Done. In every new terminal run:"
echo "  source /opt/ros/jazzy/setup.bash && source $(pwd)/install/setup.bash"
