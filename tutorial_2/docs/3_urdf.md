# Practice 3: Gazebo and URDF

As you have seen, we can simulate a UR5 robot in Gazebo using a predefined launch file:
~~~~bash
# select option 1
ros2 launch manip_challenge ur5_setup.launch.py
~~~~

How do we add an object in Gazebo? Let's create a URDF file.
~~~~bash
cd ${your_cs477_ws}/src/CS477_IIR_2024S/tutorial_2/tutorial_2
or,
# This will create and leave /log directory at the working directory
cd $(colcon list --paths-only | grep tutorial_2) 

gedit cube.urdf
~~~~

Then, spawn the object in Gazebo by calling a service:
~~~~bash
ros2 run tutorial_2 4_gazebo_object
~~~~

