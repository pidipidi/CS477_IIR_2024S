import os
import xacro
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    robot_description_config = xacro.process_file(os.path.join(get_package_share_directory('robotiq_85_description'), 'urdf', 'robotiq_85_gripper.urdf.xacro'))
    robot_description = {'robot_description': robot_description_config.toxml()}

    # joint_state_pub_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher'
    # )

    joint_state_pub_gui_node = Node(
                                package='joint_state_publisher_gui',
                                executable='joint_state_publisher_gui',
                                name='joint_state_publisher_gui',
                                parameters=[robot_description])

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='log',
      arguments=['-d', os.path.join(get_package_share_directory("robotiq_85_description"), "config", "view_gripper.rviz")],
      parameters=[robot_description]
      )

    return LaunchDescription([rviz_node, robot_state_pub_node, joint_state_pub_gui_node])
