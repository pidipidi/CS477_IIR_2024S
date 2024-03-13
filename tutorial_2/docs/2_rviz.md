# Practice 2: RViZ
## Visualize the UR5 robot in RViZ
Launch a Gazebo with the UR5 robot:
~~~~bash
# select option 1
ros2 launch manip_challenge ur5_setup.launch.py
~~~~
You can run a 3D visualizer in a new terminal:
~~~~bash
ros2 run rviz2 rviz2
~~~~
To visualize the UR5 robot, add "RobotModel" and set its description topic to "/robot_description" (Ignore error messages).
Make sure to change Global Options - Fixed Frame to "world".


You can visualize any pose information by publishing and subscribing a Pose message:
~~~~bash
ros2 run tutorial_2 2_posestamp
~~~~
This means you can visualize a sequence of pose trajectory, too.
~~~~bash
ros2 run tutorial_2 3_posearray
~~~~
