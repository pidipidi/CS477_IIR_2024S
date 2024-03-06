# CS477: Introduction to Intelligent Robotics (IIR)

# Installation
## Pre-requites for this tutorial
This repository requires Ubuntu 20.04 and ROS2 Foxy. You can install the ROS Foxy following instructions on https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html.

<!-- ~~~~bash
sudo apt-get install ros-melodic-trac-ik ros-melodic-trac-ik-python ros-melodic-moveit-ros ros-melodic-moveit-planners* ros-melodic-moveit-ros-planning* ros-melodic-moveit-ros-move-group ros-melodic-moveit-ros-control-interface ros-melodic-moveit-kinematics ros-melodic-industrial-msgs ros-melodic-moveit-kinematics ros-melodic-ddynamic-reconfigure ros-melodic-gazebo-plugins ros-melodic-rqt-py-trees ros-melodic-py-trees* ros-melodic-gripper-action-controller ros-melodic-rqt-joint-trajectory-controller ros-melodic-joint-trajectory-controller python-catkin-tools python-pyassimp ros-melodic-soem ros-melodic-effort-controllers ros-melodic-realsense2-description ros-melodic-librealsense2 -y
~~~~ -->

## Installation of your project repository
~~~~bash
source /opt/ros/foxy/setup.sh
~~~~

Move to anyfolder you want to place the class code. Then, you can create the workspace,
~~~~bash
mkdir -p ~/cs477_ws/src
cd ~/cs477_ws/src/
~~~~

Let's copy the the class repo, install dependencies, and build it!
~~~~bash
git clone https://github.com/pidipidi/CS477_IIR_2024S
cd ..
# source ./src/CS477_IIR_2024S/install.sh
colcon build --symlink-install
source ./install/local_setup.bash
~~~~

Open a new terminal, source your main ROS2 environment and source this repo as an overlay.
~~~~bash
source /opt/ros/foxy/setup.bash
cd ~/cs477_ws
source install/local_setup.bash
~~~~

For convenience, you can add following to not source every time you open a new terminal:
~~~~bash
echo "source /opt/ros/foxy/setup.bash; ROS_VERSION=2; export ROS_PYTHON_VERSION=3" >> ~/.bashrc
echo "source ~/cs477_ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
~~~~


# Links 
- [Tutorial I](tutorial_1/README.md)
- [Tutorial II](tutorial_2/README.md)
- [Tutorial III](tutorial_3/README.md)
- [Tutorial IV](tutorial_4/README.md)


- [Assignment I](assignment_1/README.md)
- [Assignment II](assignment_2/README.md)
- [Assignment III]()

- [Manipulation Challenge](manip_challenge/README.md)


# ETC
There are many useful command-line tools like rostopic, rqt_graph, rosbag, etc. Please, see http://wiki.ros.org/ROS/CommandLineTools

There may be password authentification issue. Please, check following [answers](https://stackoverflow.com/questions/68775869/support-for-password-authentication-was-removed-please-use-a-personal-access-to, "stackoverflow link")



