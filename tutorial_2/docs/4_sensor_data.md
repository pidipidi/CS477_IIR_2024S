# Practice 4: Sensor Data Acquisition
Launch a Gazebo with the UR5 robot:
~~~~bash
# select option 1
ros2 launch manip_challenge ur5_setup.launch.py
~~~~
Realsense d435 camera is an RGB-D camera that produces RGB and depth images. You can find the topics:
~~~~bash
ros2 topic list | grep camera
~~~~

Then, you can subscribe the images and also visualize it through python or RViZ. Here are the examples.
~~~~bash
# Top-down view camera
ros2 topic echo /camera/color/image_raw
# Wrist camera
ros2 topic echo /wrist_camera/color/image_raw
~~~~
In RViZ, you can also click the add Image to visualize the image stream from the camera.
~~~~bash
ros2 run rviz2 rviz2
~~~~


You can subscribe the topic and store it using opencv.
~~~~bash
ros2 run tutorial_2 5_camera_image_saver
~~~~
If you want object detection, yolu can finetune the MASK R-CNN. If you need python3, you can use virtual environment for it. Let me know, if you need example for the virtual environment with ROS. 


