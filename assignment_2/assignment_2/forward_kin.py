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

from geometry_msgs.msg import Point, Quaternion, PoseArray, Pose, Wrench
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from assignment_2 import move_joint

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']



def your_forward_kinematics(theta):
    """ Compute the end-effector pose given a set of joint angles """
    
    #------------------------------------------------------------
    # ADD YOUR CODE
    #------------------------------------------------------------
    # Place your homogeneous transformation matrix here! 
    


    # you can print out a pose message by filling followings
    # please, do not import external library like PyKDL 
    # you can import math or numpy like default libraries. 
    ps = Pose()
    ps.position.x = 0.
    ps.position.y = 0.
    ps.position.z = 0.
    ps.orientation.x = 0.
    ps.orientation.y = 0.
    ps.orientation.z = 0.
    ps.orientation.w = 0.  
    #------------------------------------------------------------
    
    return ps

def send_command(node, theta, duration=3):
    
    # construct a goal message
    g = FollowJointTrajectory.Goal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points.append(
        JointTrajectoryPoint(positions=theta, velocities=[0]*6,
                                 time_from_start=Duration(sec=int(duration),
            nanosec=int((duration-int(duration))*1e9)))
        )    
    move_joint.move_joint(node, g)


def main(args=None):
    
    rclpy.init() 
    
    node = rclpy.create_node("arm_client")    
    rclpy.spin_once(node, timeout_sec=1)
    node.get_logger().info("init_ur5: direct control mode")
    
    #------------------------------------------------------------
    # ADD YOUR CODE
    #------------------------------------------------------------
    # Place your desired joint angles! 
    
    # Problem 1.C (i)
    theta = [0,0,0,0,0.7,0]
    node.get_logger().info("{}".format(your_forward_kinematics(theta)))
    send_command(node, theta)


    # Problem 1.C (ii)
    theta = [0,0,0,0,0,0]
    node.get_logger().info("{}".format(your_forward_kinematics(theta)))
    send_command(node, theta)
    #------------------------------------------------------------
    
    node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
        



