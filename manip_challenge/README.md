# manipulation_challenge

We will launch the speed contest setup (i.e., manipulation challenge). In each terminal, please source the 'setup.bash'.

## How to test the pick_and_place system?

Terminal 1
~~~~bash
ros2 launch manip_challenge ur5_setup.launch.py
~~~~
Select the 'position' controller pressing '1'. If this is your first trial, it may take time. 

Terminal 2
~~~~bash
ros2 run manip_challenge example1
~~~~
You will be able to see how to send joint command and also close/open the gripper.



## How to publish and subscribe a task specification during the challenge?
The instructor/TA will send a string that contains a dictionary format of commands via a topic "/task_commands":
~~~~bash
    d = {"storage_left": ['book', 'eraser', 'soap2'],
        "storage_right": ['snacks', 'biscuits', 'glue', 'soap'] }
~~~~
This dictionary represents the final locations of objects on the front table. Your robot should subscribe the topic and pick-and-place the items on the right locations. You can test an example command using following script:

Terminal 1 (check the published topic from terminal)
~~~~bash
ros2 run manip_challenge item_list_sub
~~~~

Terminal 2 
~~~~bash
ros2 run manip_challenge item_list_pub
~~~~



## How to query the pose of an object in the world model?
We are using the physics-based simulator, GAZEBO, for this challenge. Without using any perception method, we can identify and obtain the state of objects by subscribing the internal topics from GAZEO. Following commands and code show how to query the pose of a specific object:
~~~~bash
ros2 run manip_challenge example2
~~~~

