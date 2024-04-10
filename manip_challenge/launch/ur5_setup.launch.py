#!/usr/bin/python3


# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                                LogInfo, RegisterEventHandler, TimerAction,
                                ExecuteProcess)
from launch.substitutions import TextSubstitution, LaunchConfiguration

import xacro
import yaml

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():

    # Get the launch directory
    world_model_dir = get_package_share_directory('manip_challenge')
    
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(world_model_dir,
                                       'config',
                                       'default_sim.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    # ***** GAZEBO ***** #   
    # DECLARE Gazebo WORLD file:
    world = os.path.join(
      get_package_share_directory('manip_challenge'),
      'worlds',
      'ur5_picking_challenge.world')
    
    # DECLARE Gazebo LAUNCH file:
    manip_challenge_models = os.path.join(
      get_package_share_directory('manip_challenge'),
      'models')
    
    if 'GAZEBO_MODEL_PATH' in os.environ:
        print("GAZEBO_MODEL_PATH already exists")
    else:
        print("GAZEBO_MODEL_PATH does not exist")
        os.environ['GAZEBO_MODEL_PATH'] = manip_challenge_models
        print("GAZEBO_MODEL_PATH set to", os.environ['GAZEBO_MODEL_PATH'])

    # Lauch UR5 within GAZEBO
    ur5 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ur5_ros2_gazebo'), 'launch'), '/ur5_simulation.launch.py']),
        launch_arguments={'world': world,
                          'cell_layout_1': "true",
                          'cell_layout_2': "false",
                          'hardware_interface': "PositionJointInterface",
                          'camera_enabled': "true",
                          'z_offset': '0.6',
                              }.items(),)

    # World Model
    world_model_node = Node(
            name="world_model_publisher",
            package='manip_challenge',
            executable='world_model_gazebo',
            output='screen',
            parameters=[params_file]
            )

    
    # INITIALIZE ROBOT JOINT:
    init_joints = Node(package='manip_challenge', executable='init_joints', output='screen')

    # Add manipulatable objects
    add_object = Node(package='manip_challenge', executable='add_object',
                      output='screen',
                      )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        params_file_arg,
        ur5,
        init_joints,
        add_object,
        world_model_node
    ])
