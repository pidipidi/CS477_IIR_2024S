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

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA (2022) ROS2.0 ROBOT SIMULATION. URL: https://github.com/IFRA-Cranfield/ros2_RobotSimulation.

#  Reference: This UR5 Gazebo Simulation package has been created thanks to the information provided in the following GitHub repositories:
#   - Universal Robots ROS2 Driver: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver 
#   - Universal Robots ROS2 Gazebo Simulation: https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation

# ur5_simulation.launch.py:
# Launch file for the Universal Robots UR5 Robot GAZEBO SIMULATION in ROS2 Foxy: 

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml

# LOAD FILE:
## def load_file(package_name, file_path):
##     package_path = get_package_share_directory(package_name)
##     absolute_file_path = os.path.join(package_path, file_path)
##     try:
##         with open(absolute_file_path, 'r') as file:
##             return file.read()
##     except EnvironmentError:
##         # parent of IOError, OSError *and* WindowsError where available.
##         return None
## # LOAD YAML:
## def load_yaml(package_name, file_path):
##     package_path = get_package_share_directory(package_name)
##     absolute_file_path = os.path.join(package_path, file_path)
##     try:
##         with open(absolute_file_path, 'r') as file:
##             return yaml.safe_load(file)
##     except EnvironmentError:
##         # parent of IOError, OSError *and* WindowsError where available.
##         return None

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    
    # ***** GAZEBO ***** #   
    # DECLARE Gazebo WORLD file:
    ur5_ros2_gazebo = os.path.join(
        get_package_share_directory('ur5_ros2_gazebo'),
        'worlds',
        'ur5.world')
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': ur5_ros2_gazebo}.items(),
             )
    gzserver = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gzserver.launch.py']),
                launch_arguments={'world': ur5_ros2_gazebo}.items(),
             )

    # ========== COMMAND LINE ARGUMENTS ========== #
    print("")
    print("ros2_RobotSimulation --> UR5 ROBOT")
    print("Launch file -> ur5_simulation.launch.py")

    print("")
    print("Robot configuration:")
    print("")

    # Cell Layout:
    cell_layout_1 = "false"
    cell_layout_2 = "true"

    # Selection of Controller
    hardware_interface = "true"
    
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
        "cell_layout_1": cell_layout_1,
        "cell_layout_2": cell_layout_2,
        "hardware_interface": hardware_interface,})
    print("--------------------------")
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}
    print("--------------------------")

    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'ur5'],
                        output='screen')

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
        return LaunchDescription([
            gzserver, 
            node_robot_state_publisher,
            load_ur5_controller,
            load_gripper_controller,
            load_joint_state_broadcaster,
            spawn_entity,
            static_tf_publisher,
            ])
    
    return LaunchDescription([
        gazebo, 
        node_robot_state_publisher,
        load_ur5_controller,
        load_gripper_controller,
        load_joint_state_broadcaster,
        spawn_entity,
        static_tf_publisher,
    ])
