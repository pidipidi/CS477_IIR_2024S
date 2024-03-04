# Practice 2: ROS Service
Let's test ROS service! ROS service is a kind of hand-shake system between a server and a client, where the service server responses given the service client's request. By using these, you can prevent to loose information during important communication. 

You will be able to see example service code from service_server.py and service_client.py. Please, try to run following commands after checking the code. 


## Python
Terminal 1: the server node will response to the request of the service request. This node waits for the client node to send a request.
~~~~bash
ros2 run tutorial_1 service_server
~~~~
Terminal 3: the service client node will request a response to the server. In this example, we will send a request to change a boolean data.
~~~~bash
ros2 run tutorial_1 service_client
~~~~

## Commandline Tools

You can request to the server via your terminal:

~~~~bash
ros2 service call /tutorial1 std_srvs/srv/SetBool "data: false""
~~~~

`std_srvs/srv/SetBool` is service type that used in this tutorial. You can check this at, https://docs.ros.org/en/noetic/api/std_srvs/html/srv/SetBool.html.