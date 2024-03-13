# Command-line Tools

## ros2 launch and ros2 node
Launch a Gazebo with the UR5 robot:
~~~~bash
# select option 1
ros2 launch manip_challenge ur5_setup.launch.py
~~~~

Let's check what nodes are actually running using the following command in a new terminal:
~~~~bash
ros2 node list
~~~~

You can see more information of the node by runnign following command:
~~~~bash
ros2 node info $node_name$
~~~~


## ros2 run
This command is used for running an executable file in a specified package.
~~~~bash
ros2 run tutorial_2 1_printout
~~~~

## TF
You can see different frames are being published via /tf topic:
~~~~bash
ros2 topic echo /tf
~~~~

You can print out the transformation between one coordinate frame to another:
~~~~bash
ros2 run tf2_ros tf2_echo [one frame] [another frame]
~~~~

You can also visualize the current TF tree by listening the /tf topic for 5.0 seconds:
~~~~bash
ros2 run tf2_tools view_frames.py
evince frames.pdf
~~~~

