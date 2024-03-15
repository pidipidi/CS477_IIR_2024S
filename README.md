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

# for AMD architecture user,
source ./src/CS477_IIR_2024S/install.sh
# for ARM architecture user,
source ./src/CS477_IIR_2024S/install_arm.sh
~~~~

After installation, build the packages.
~~~~bash
colcon build --symlink-install
~~~~

**Check that all the packages are successfully built.** If you encounter some errors, please try several time again and check the [troubleshooting](https://github.com/pidipidi/CS477_IIR_2024S/blob/main/README.md#troubleshooting). 
You may ignore some warnings like CATKIN~~ or allow-override.
After building the packages, please source it.

~~~~bash
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

# Troubleshooting

## 1. `Could not determine ref type of version : git@github.com: Permission denied (publickey)`

Please set up your ssh key. You can follow the procedure in the [link](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent).

## 2. cc1plus killed

This is memory issue of your device. First you can try

~~~~bash
colcon build --symlink-install --parallel-workers=1 --cmake-args -DCMAKE_CXX_FLAGS="--param ggc-min-expand=20"
~~~~

If the error persists and you are using VM for the ubuntu, you can try to increase the usage of your VM.

If you are using wsl2, you can follow [link](https://fizzylogic.nl/2023/01/05/how-to-configure-memory-limits-in-wsl2).

If you are using UTM, you can find memory setting slot in: select your VM image -> click upperwrite setting button(edit selected VM) -> system -> system.

We recommend you to allocate as much as you can, but the minimal guideline is 12GB.

## 3. gazebo dies after clean installation

First, make sure that all the packages are successfully built before running the script.

If you checked all installation/build is properly done, the problem might be solved by allocating more memory since gazebo depends heavily on RAM specification.

Check above section about allocating more memory to your VM.

# ETC
There are many useful command-line tools like rostopic, rqt_graph, rosbag, etc. Please, see http://wiki.ros.org/ROS/CommandLineTools

There may be password authentification issue. Please, check following [answers](https://stackoverflow.com/questions/68775869/support-for-password-authentication-was-removed-please-use-a-personal-access-to, "stackoverflow link")


