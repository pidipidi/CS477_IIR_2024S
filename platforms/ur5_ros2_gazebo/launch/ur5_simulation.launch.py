#!/usr/bin/python3

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Seemal Asif      - s.asif@cranfield.ac.uk                                   #
#           Phil Webb        - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: July, 2022.                                                                    #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #


# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.launch_context import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import xacro
import yaml


def evaluate_fn(context, *args, **kwargs):

    # ***** ARGUMENTS ***** #   
    world              = LaunchConfiguration('world')
    cell_layout_1      = LaunchConfiguration('cell_layout_1')
    cell_layout_2      = LaunchConfiguration('cell_layout_2')
    hardware_interface = LaunchConfiguration('hardware_interface')
    camera_enabled     = LaunchConfiguration('camera_enabled')
    z_offset           = LaunchConfiguration('z_offset')
    
    # ***** ROBOT DESCRIPTION ***** #
    # UR5 ROBOT Description file package:
    ur5_description_path = os.path.join(
        get_package_share_directory('ur5_ros2_gazebo'))
    # UR5 ROBOT ROBOT urdf file path:
    xacro_file = os.path.join(ur5_description_path,
                              'urdf',
                              'ur5.urdf.xacro')

    
    # Generate ROBOT_DESCRIPTION for UR5 ROBOT:
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={
        "cell_layout_1": cell_layout_1.perform(context),
        "cell_layout_2": cell_layout_2.perform(context),
        "hardware_interface": hardware_interface.perform(context),
        "camera_enabled": camera_enabled.perform(context),        
        })
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}


    # ***** GAZEBO ***** #       
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': world.perform(context)}.items(),
             )
    gzserver = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gzserver.launch.py']),
                launch_arguments={'world': world.perform(context)}.items(),
             )

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'ur5',
                                   '-z', z_offset.perform(context)],
                        output='screen')
        
    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )


    # Selection of GUI
    print("- GUI:")
    error = True
    while (error == True):
        print("     + Option N1: GUI - Gazebo.")
        print("     + Option N2: Headless - GZSERVER.")
        gz_flag = input ("  Please type and enter the option number(1 or 2): ")
        if (gz_flag == "1"):
            error = False
            gz_flag = False
        elif (gz_flag == "2"):
            error = False
            gz_flag = True
        else:
            print ("  Please select a valid option!")
    print("")


    # ***** RETURN LAUNCH DESCRIPTION ***** #
    if gz_flag:
        return [
            gzserver, 
            node_robot_state_publisher,
            spawn_entity,
            ]
    
    return [
        gazebo, 
        node_robot_state_publisher,
        spawn_entity,
        ]
    

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    print("")
    print("ros2_RobotSimulation --> UR5 ROBOT")
    print("Launch file -> ur5_simulation.launch.py")

    print("")
    print("Robot configuration:")
    print("")

    # DECLARE Gazebo WORLD file:
    world_file = os.path.join(
        get_package_share_directory('ur5_ros2_gazebo'),
        'worlds',
        'ur5.world')
    world_arg = DeclareLaunchArgument('world',
                                      default_value=world_file)


    cell_layout_1_arg = DeclareLaunchArgument('cell_layout_1',
                                              default_value='false')
    cell_layout_2_arg = DeclareLaunchArgument('cell_layout_2',
                                              default_value='true')
    camera_enabled_arg = DeclareLaunchArgument('camera_enabled',
                                              default_value='false')
    z_offset_arg = DeclareLaunchArgument('z_offset',
                                         default_value='0.0')

    # ========== COMMAND LINE ARGUMENTS ========== #
    print("- Controller:")
    while True:
        print("     + Option N1: Position Controller.")
        print("     + Option N2: Effort Controller.")
        flag = input ("  Please type and enter the option number(1 or 2): ")
        if (flag == "1"):
            hardware_interface = "PositionJointInterface"
            break
        elif (flag == "2"):
            hardware_interface = "EffortJointInterface"
            break
        print ("  Please select a valid option!")
    
    hardware_interface_arg = DeclareLaunchArgument('hardware_interface',
                                              default_value=hardware_interface)

                
    # Load CONTROLLERS
    load_ur5_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'ur5_controller'],
        output='screen'
    )

    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'gripper_controller'],
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Static TF NODE:
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=["0.16","0","0","1.5708","0","1.5708","robotiq_85_base_link", "tool_center_point"],
    )
    
    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        world_arg,
        cell_layout_1_arg,
        cell_layout_2_arg,
        hardware_interface_arg,
        camera_enabled_arg,
        z_offset_arg,
        load_ur5_controller,
        load_gripper_controller,
        load_joint_state_broadcaster,
        static_tf_publisher,
        OpaqueFunction(function=evaluate_fn)  
    ])
