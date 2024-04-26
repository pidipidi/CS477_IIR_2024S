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
import numpy as np

from geometry_msgs.msg import Point, Quaternion, PoseArray, Pose, Wrench
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from manip_challenge import move_joint, move_gripper, get_joint, get_pose

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']



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


    # ---------------------------------------------------------------------
    # Check the robot status
    # --------------------------------------------------------------------
    # Get the current joint angle
    angles = get_joint.get_joint_angles(node)
    node.get_logger().info("Angles: {}".format(angles))
    
    # Get the current endeffector pose
    pose = get_pose.get_pose(node)
    node.get_logger().info("Pose: {}".format(pose))


    # ---------------------------------------------------------------------
    # Check gripper functions
    # ---------------------------------------------------------------------
    move_gripper.gripper_close(node)
    move_gripper.gripper_open(node)
    #arm.getGripperState()


    # ---------------------------------------------------------------------
    # Check joint movements
    # ---------------------------------------------------------------------    
    # Move Joints
    ## move_joint.move_joint(node, [0, 0, 0, -np.pi/2.0, 0, -np.pi/2.0], duration=6.0)
    ## move_joint.move_joint(node, [np.pi/3, 0, 0, -np.pi/2.0, 0, -np.pi/2.0], duration=6.0)

    # ---------------------------------------------------------------------
    # Check pick-and-place
    # ---------------------------------------------------------------------       
    move_gripper.gripper_open(node)
    move_joint.move_joint(node, [0, -np.pi/2., np.pi/2., -np.pi/2., -np.pi/2., np.pi/4.], duration=5.0)
    move_joint.move_joint(node, [0.16170570835536457, -1.4298043774726588, 1.32243941822082, -1.4706636556410826, -1.5801858129904114, 0.16369705271839724], duration=5.0)
    move_joint.move_joint(node, [0.16456877764159317, -1.4026709127086092, 1.6609417499881791, -1.829975190229056, -1.570936444921175, 0.16300417865254457-0.16], duration=5.0)
    
    move_gripper.gripper_close(node)
    move_joint.move_joint(node, [0.1551163086351373, -1.43718690172762, 1.3254160855275712, -1.4612148276759138, -1.5647456289308708, 0.15966451933142772-0.16], duration=5.0)

    move_joint.move_joint(node, [1.3679801926428028, -1.4297246128145917, 1.2446811592243003, -1.369128425872749, -1.5598725764592507, 2.1546155544206376], duration=5.0)
    move_joint.move_joint(node, [1.3677681606035597, -1.4306672689958717, 1.6188301424835283, -1.7400717740118714, -1.5650770640998972, 2.156513636080088], duration=5.0)
    move_gripper.gripper_open(node)
    move_joint.move_joint(node, [1.3770551501789219, -1.434575401913625, 1.2522653950772369, -1.3755392458833133, -1.5621581114491467, 2.1658595873828146], duration=5.0)

    ## #------------------------------------------------------------
    
    #node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
        



