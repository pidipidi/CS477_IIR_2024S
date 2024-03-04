# Practice 3: ROS Parameter
Let's test a ROS parameter server! A ROS parameter server is a type of shared dictionary services over ROS nodes. This is often used for data sharing between nodes that do not require strict real-time constraints. Please, be aware that storing and retrieving are not such fast enough to communicate information at runtime. 


## Python
Terminal 1: 
~~~~bash
ros2 run tutorial_1 ros_parameter
~~~~
If you run this, the parameter client will first declare the parameter `tutorial1` as `test1`, get and log this parameter every second, and set this parameter to the new value `test2`. You can change this value by command line tools.

## Commandline Tools
Make sure the previous node is running.

Terminal 2: You can check the custom parameter `my_parameter`by 
~~~~bash
ros2 param list
~~~~
To change it, simply run the following line in the console:

~~~~bash
ros2 param set /parameter_client /tutorial1 test3
~~~~

Now the parameter client will return the message with changed parameter.

You can retrieve the information of the parameter:
~~~~bash
ros2 param get /parameter_client /tutorial1
~~~~