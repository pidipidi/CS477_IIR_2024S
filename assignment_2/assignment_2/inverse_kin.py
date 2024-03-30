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

from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import numpy as np
from assignment_2 import move_joint

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def move_position(node, goal_pose, init_joint=None):
    """ 
    A function to send a list of joint angles to the robot. 
    This function waits for completing the commanded motion.

    Parameters
    ----------
    node:  
        a ROS2 node handle
    goal_pose : Pose
        a Pose message from geometry_msgs
    init_joint : list
        a initial/current angle
    """

    #------------------------------------------------------------
    # ADD YOUR CODE
    #------------------------------------------------------------
    # 1) construct forward kinematics (problem 1)

    
    # 2) Get the start position from the init_joint via forward kinematics

    
    # 3) Get the goal position from the goal_pose
    
    
    # 4) Get a pose/position trajectory from start to goal positions
    #pose_traj =

    
    # construct a goal message
    g = FollowJointTrajectory.Goal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
        
    # Get a sequence of joint angles that track the position trajecotory
    q = init_joint
    dt = 0.05
    time_from_start = 0
    for i in range(1, len(pose_traj)):
        
        # Find a Jacobian
        # J = 
        # J_p = J[:3]

        # Take Pseudo inverse
        # J_inv = 

        # Compute a delta position
        # dx =

        # Compute a delta theta        
        # dtheta =

        # Compute a desired theta
        #q = 
        #time_from_start = time_from_start + dt
        #g.trajectory.points.append(
        #    JointTrajectoryPoint(positions=q, velocities=[0]*6,
        #                         time_from_start=Duration(sec=int(time_from_start),
        #    nanosec=int((time_from_start-int(time_from_start))*1e9)))
        #    )        


    #------------------------------------------------------------
    
    move_joint.move_joint(node, g)




def main(args=None):
    
    rclpy.init() 
    
    node = rclpy.create_node("arm_client")    
    rclpy.spin_once(node, timeout_sec=1)
    node.get_logger().info("init_ur5: direct control mode")

    # Problem 2
    # Move to the initial joint configuration
    theta    = [-0.8, -1.5708, 1.5708, -1.5708, -1.5708, -0.8]
    duration = 4
    
    g = FollowJointTrajectory.Goal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points.append(
        JointTrajectoryPoint(positions=theta, velocities=[0]*6,
                                 time_from_start=Duration(sec=int(duration),
            nanosec=int((duration-int(duration))*1e9)))
        )        
    move_joint.move_joint(node, g)

    # Move following the linear position trajectory
    goal = Pose()
    goal.position.x = 0.418
    goal.position.y = 0.273
    goal.position.z = 0.264
    move_position(node, goal, theta)
    
    node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
        



