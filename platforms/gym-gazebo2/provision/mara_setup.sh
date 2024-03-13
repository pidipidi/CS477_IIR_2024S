source /opt/ros/foxy/setup.bash
source ~/git/external/ros2_mara_ws/install/setup.bash
source /usr/share/gazebo-11/setup.sh
export RMW_IMPLEMENTATION=rmw_opensplice_cpp
export PYTHONPATH=$PYTHONPATH:~/git/external/ros2_mara_ws/install/lib/python3/dist-packages
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/git/external/ros2_mara_ws/install/share
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/git/external/ros2_mara_ws/install/lib
