#!/usr/bin/env python3
"""
Copyright 2020 Daehyung Park

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
import rclpy
import rclpy.node

from pykdl_utils.kdl_kinematics import create_kdl_kin
from assignment_2 import misc
from hrl_geom.pose_converter import PoseConv
import os
import xacro
from ament_index_python.packages import get_package_share_directory

from manip_challenge import get_joint


JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
def get_pose(node):
    """ 
    Get the current joint state according to the order of JOINT_NAMES.

    Parameters
    ----------
    node:  
        a ROS2 node handle
    """

    # Create Forward Kinematics
    
    #------------------------------------------------------------
    # Construct Forward Kinematics
    #------------------------------------------------------------

    ur5_description_path = os.path.join(
        get_package_share_directory('ur5_ros2_gazebo'))
    xacro_file = os.path.join(ur5_description_path,
                              'urdf',
                              'ur5.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={
        "cell_layout_1": 'false',
        "cell_layout_2": 'true',
        "hardware_interface": 'PositionJointInterface',})
    robot_description = doc.toxml()
    
    arm_kdl = create_kdl_kin("base_link", "robotiq_85_base_link",
                             urdf_xml=robot_description)
    # offset from robotiq_85_base_link to tool_center_point
    tool_offset_frame = misc.list2KDLframe([0.16, 0, 0, 1.5708, 0, 1.5708])

    #------------------------------------------------------------
    # Get the current pose via FK
    #------------------------------------------------------------

    joint_angles = get_joint.get_joint_angles(node)
    homo_mat = arm_kdl.forward(joint_angles)
    pos, quat = PoseConv.to_pos_quat(homo_mat)
    pose = misc.list2Pose(list(pos)+list(quat))
    tool_frame = misc.pose2KDLframe(pose) * tool_offset_frame


    return misc.KDLframe2Pose(tool_frame)
    

    
    


def main(args=None):
    
    rclpy.init() 
    
    node = rclpy.create_node("get_pose")    
    rclpy.spin_once(node, timeout_sec=1)
    node.get_logger().info("Subscribe joint states")

    s = get_pose(node)
    node.get_logger().info("{}".format(s))
    
    node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
        



