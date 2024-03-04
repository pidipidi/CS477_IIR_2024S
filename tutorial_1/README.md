This is the first tutorial. 

First build current package by

~~~~bash
cd ~/cs477_ws
colcon build --packages-select tutorial_1
~~~~

Note that you have to build the package in the workspace folder that includes `src` directory. Then source the setup files by

~~~~bash
source install/setup.bash
~~~~

**Don't forget to source** whenever you make changes or open the new terminal.

Now proceed the following tutorials.

- [1. Publisher and Subscriber](docs/1_publisher_and_subscriber.md)
- [2. Service](docs/2_service.md)
- [3. Parameter](docs/3_parameter.md)
- [4. Package](docs/4_create_pkg.md)
- [5. Turtlebot3](docs/5_Turtlebot3.md)