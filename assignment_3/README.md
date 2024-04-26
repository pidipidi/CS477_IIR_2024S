## Note that there can be changes on this repository. Please, pull and re-build this repository.

# Assignment 3
## Problem 1

Implement all the necessary files and then build the assignment_3 package using
~~~~bash
colcon build --symlink-install --packages-select assignment_3
~~~~

If you want to build all the workspace, run following
~~~~bash
colcon build --symlink-install 
~~~~

### 1.A
You can run the min_jerk.py by running following 
~~~~bash
ros2 run assignment_3 min_jerk
~~~~

### 1.B
You can run the move_joint.py by running followings

Terminal 1: launch the robot with the position controller
~~~~bash
ros2 launch ur5_ros2_gazebo ur5_simulation.launch.py
~~~~

Terminal 2: move to a joint configuration goal
~~~~bash
ros2 run assignment_3 move_joint --1b
~~~~

### 1.C
You can run the move_joint.py by running followings 

Terminal 1: launch the robot with the position controller
~~~~bash
ros2 launch ur5_ros2_gazebo ur5_simulation.launch.py
~~~~

Terminal 2: move to a position goal
~~~~bash
ros2 run assignment_3 move_joint --1c
~~~~

### 1.D
You can run the move_joint.py by running following

Terminal 1: launch the robot with the position controller
~~~~bash
ros2 launch ur5_ros2_gazebo ur5_simulation.launch.py
~~~~

Terminal 2L move to a pose goal
~~~~bash
ros2 run assignment_3 move_joint --1d
~~~~

## Problem 2

### 2.A
Run A* on 2D space
~~~~bash
cd src/cs477_ws/assignment_3/assignment_3
python astar_2d_test.py
~~~~

### 2.B
Run A* on 3D space
~~~~bash
cd src/cs477_ws/assignment_3/assignment_3
python astar_3d_test.py
~~~~

### 2.C
Run A* with a UR5e

Terminal 1: launch the robot with the position controller
~~~~bash
ros2 launch ur5_ros2_gazebo ur5_simulation.launch.py
~~~~

Terminal 2L add objects in GAZEBO
~~~~bash
ros2 run assignment_3 add_object
~~~~

Terminal 3: enable the UR5e to avoid obstacles
~~~~bash
ros2 run assignment_3 move_astar
~~~~

