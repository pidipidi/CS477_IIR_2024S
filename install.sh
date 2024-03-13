#!/bin/sh

# Install dependencies
sudo apt install -y python3-pip
sudo apt install -y ros-foxy-moveit ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-gripper-controllers gazebo11 ros-foxy-gazebo-ros2-control ros-foxy-gazebo-ros-pkgs ros-foxy-xacro ros-foxy-moveit-common ros-foxy-py-trees ros-foxy-py-trees-ros-interfaces ros-foxy-joint-state-publisher 
sudo apt install -y python3-vcstool python3-pykdl
sudo apt install -y libnlopt-cxx-dev
sudo apt install ros-foxy-rmw-fastrtps-cpp

# Install realsense2 camera SDK
sudo apt-get -y install ros-foxy-diagnostic-updater
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get -y update
sudo apt-get -y install librealsense2-dkms
sudo apt-get -y install librealsense2-utils
sudo apt-get -y install librealsense2-dev

# Install default requirements
pip3 install -r ./src/CS477_IIR_2024S/requirements.txt

# Install moveit
sudo cp ./src/CS477_IIR_2024S/utils/include/move_group_interface_improved.h /opt/ros/foxy/include/moveit/move_group_interface/

vcs import --input src/CS477_IIR_2024S/docs/common_sim.repos --workers 1

#cd ..
source /opt/ros/foxy/setup.bash
rosdep install -i --from-path src --rosdistro foxy --skip-keys gazebo_grasp_plugin gazebo_version_helpers roboticsgroup_upatras_gazebo_plugins -y -r

# Install binary packages
sudo dpkg -i ./src/CS477_IIR_2024S/ros-foxy-gazebo-grasp-plugin_1.0.2-0focal_amd64.deb
sudo dpkg -i ./src/CS477_IIR_2024S/ros-foxy-gazebo-version-helpers_0.0.0-0focal_amd64.deb
sudo dpkg -i ./src/CS477_IIR_2024S/ros-foxy-roboticsgroup-upatras-gazebo-plugins_0.2.0-0focal_amd64.deb

colcon build --symlink-install --packages-ignore gazebo_plugins realsense_gazebo_plugin
colcon build --symlink-install --packages-select gazebo_plugins realsense_gazebo_plugin --parallel-workers=1 --cmake-args -DCMAKE_CXX_FLAGS="--param ggc-min-expand=20"
source ./install/local_setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
