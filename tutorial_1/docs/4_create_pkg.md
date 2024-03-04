# Practice 4: Create your own package

You can create a package to run your own nodes using ament_python. Make sure you are in the src folder before running the package creation command.
~~~~bash
cd ~/cs477_ws/src/
ros2 pkg create --build-type ament_python --node-name test_node cs477_practice
~~~~
For this tutorial, you will use the optional argument `--node-name` which creates a simple Hello World type executable in the package. You will now have a new folder within your workspace’s `src` directory called `cs477_practice`.

You can build packages in the workspace root using `colcon build`.
~~~~bash
cd ~/cs477_ws
colcon build
~~~~

Since it might take long to build all the packages, you can run:
~~~~bash
colcon build --packages-select cs477_practice
~~~~
to build only the `cs477_practice` package instead.

*Other useful arguments for colcon build:*

`--packages-up-to` *builds the package you want, plus all its dependencies, but not the whole workspace (saves time)*

`--symlink-install` *saves you from having to rebuild every time you tweak python scripts*

`--event-handlers console_direct+` *shows console output while building (can otherwise be found in the log directory)**

After then, you should source
~~~~bash
source install/local_setup.bash
~~~~

*Sourcing the* `local_setup` *of the overlay will only add the packages available in the overlay to your environment.* `setup` *sources the overlay as well as the underlay it was created in, allowing you to utilize both workspaces.*

*So, sourcing your main ROS 2 installation’s* `setup` *and then the* `cs477_ws` *overlay’s* `local_setup`, *like you just did, is the same as just sourcing `cs477_ws`’s setup, because that includes the environment of its underlay.*


Now that your workspace has been added to your path, you will be able to use your new package’s executables.*

Verify your package is installed, run the command
~~~~bash
ros2 run cs477_practice test_node
~~~~

Checkout https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html for more details. You can use ament_cmake for C++ based development of your package.