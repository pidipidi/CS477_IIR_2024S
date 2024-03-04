# Practice 5: Turtlebot3

Let's first install Turtlebot3, a mobile robot, which can run on a physics based simulator, GAZEBO!
~~~~bash
sudo apt install ros-foxy-turtlebot3*
~~~~

Select a turtlebot model
~~~~bash
export TURTLEBOT3_MODEL=burger
~~~~

Set up Gazebo model path
~~~~bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg \
prefix turtlebot3_gazebo \
`/share/turtlebot3_gazebo/models/
~~~~

Launch Gazebo with simulation world
~~~~bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
~~~~

To Run a keyboard-based controller, open a new terminal.
~~~~bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
~~~~

You can now control the robot using keyboard and also look at the communication between nodes. 
Please, check following tutorial for more interesting scenarios! 
https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/ , 
https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html