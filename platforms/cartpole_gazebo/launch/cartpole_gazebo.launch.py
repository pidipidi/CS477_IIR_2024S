#!/usr/bin/python3

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
import xacro


# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    
    # ***** GAZEBO ***** #   
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    # ***** ROBOT DESCRIPTION ***** #
    # ROBOT Description file package:
    cartpole_gazebo_description_path = os.path.join(
        get_package_share_directory('cartpole_gazebo'))
    # UR5 ROBOT ROBOT urdf file path:
    xacro_file = os.path.join(cartpole_gazebo_description_path,
                              'urdf',
                              'cartpole_gazebo.urdf.xacro')
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
                        arguments=['-topic', 'robot_description','-entity', 'cartpole_gazebo', '-z -0.1'],
                        emulate_tty=True,
                        output='screen')

    load_cartpole_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'cartpole_controller'],
        output='screen'
    )
    
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        load_cartpole_controller,
        load_joint_state_broadcaster,
        spawn_entity,
    ])


