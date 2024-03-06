

# Practice 1: Message publication and subscription
Let's test topic publisher and listener! First, check the two files from following directory.
~~~~bash
cd ~/cs477_ws/src/CS477_IIR_2024S/tutorial_1/tutorial_1/
~~~~
- `talker.py`
- `listener.py`
<!-- <p>&nbsp;</p> -->


## Simple publisher and subscriber
Please, open the publisher file:
~~~~bash
gedit talker.py
~~~~
You will be able to see there is a publisher that send a string format of message via 'chatter' topic. 

Then, let's open a subscriber file 
~~~~bash
gedit listener.py
~~~~
This file subscribes the string format of messages via 'chatter' topic and print out the contents using the info function.
<p>&nbsp;</p>

Please, open two terminals and then run the talker&listener nodes. <p></p>

Terminal 1: the talker node will publish a message via chatter topic.
~~~~bash
ros2 run tutorial_1 talker
~~~~
Terminal 3: the listener node will subscribe and printout the message. 
~~~~bash
ros2 run tutorial_1 listener
~~~~
<p>&nbsp;</p>
You can also see the 'chatter' topic from the topic list.

~~~~bash
ros2 topic list
~~~~

Also you can check the contents of the topic.
~~~~bash
ros2 topic echo /chatter
~~~~

## Publishing topic by Commandline Tools

Terminal 1: You can publish the chatter topic from your terminal.
~~~~bash
ros2 topic pub chatter std_msgs/String "data: 'test'"
~~~~
Terminal 2: You can subscribe the chatter topic via your terminal.
~~~~bash
ros2 topic echo chatter
~~~~

