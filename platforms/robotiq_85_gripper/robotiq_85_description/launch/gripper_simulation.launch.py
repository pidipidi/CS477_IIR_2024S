# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.actions import DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
import xacro
import yaml


# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    
    # ***** GAZEBO ***** #   
    # DECLARE Gazebo WORLD file:
    ## world_path = os.path.join(
    ##     get_package_share_directory('robotiq_85_description'),
    ##     'worlds',
    ##     'gripper.world')
    
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    # ***** ROBOT DESCRIPTION ***** #
    # UR5 ROBOT Description file package:
    robotiq_description_path = os.path.join(
        get_package_share_directory('robotiq_85_description'))
    # UR5 ROBOT ROBOT urdf file path:
    xacro_file = os.path.join(robotiq_description_path,
                              'urdf',
                              'robotiq_85_gripper.urdf.xacro')
    # Generate ROBOT_DESCRIPTION for UR5 ROBOT:
    robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description = {'robot_description': robot_description_config}

    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description','-entity', 'robotiq_85_gripper', '-z -0.68'],
                        emulate_tty=True,
                        output='screen')

    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'gripper_controller'],
        output='screen'
    )

    joint_state_pub_gui_node = Node(
                                package='joint_state_publisher_gui',
                                executable='joint_state_publisher_gui',
                                name='joint_state_publisher_gui',
                                parameters=[robot_description])

    
    rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='log',
      arguments=['-d', os.path.join(get_package_share_directory("robotiq_85_description"), "config", "view_gripper.rviz")],
      parameters=[robot_description]
      )
    


    
    return LaunchDescription([
        gazebo, 
        node_robot_state_publisher,
        load_gripper_controller,
        spawn_entity,
        # joint_state_pub_gui_node,
        # rviz_node
    ])
