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

from rclpy.action import ActionClient
from rosidl_runtime_py import message_to_yaml
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

def move_joint(node, goal):
    """ 
    A function to send a list of joint angles to the robot. 
    This function waits for completing the commanded motion.

    Parameters
    ----------
    node:  
        a ROS2 node handle
    goal : FollowJointTrajectory.Goal
        an object of FollowJointTrajectory.Goal(), where the joint order must be same as JOINT_NAMES. 
    """

    try:
        client = ActionClient(node, FollowJointTrajectory,
                              '/ur5_controller/follow_joint_trajectory')
        client.wait_for_server(5)
        node.get_logger().info("Connected to ur5_arm server")

        ## goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
    
        goal_future = client.send_goal_async(goal)
        # Return result when the command is delivered
        rclpy.spin_until_future_complete(node, goal_future)

        goal_handle = goal_future.result()
        node.get_logger().info('goal_handle:\n {}'.format(goal_handle))
    
        # Wait until the execution ends and return the result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
    
        result = result_future.result()
        if result is None:
            raise RuntimeError(
                'Exception while getting result: {!r}'.format(result_future.exception()))
        node.get_logger().info('Result:\n    {}'.format(message_to_yaml(result.result)))

    except KeyboardInterrupt:
        rclpy.shutdown("KeyboardInterrupt")
        raise
    
    node.get_logger().info("Initialized UR5")
    client.destroy()

               
def main(args=None):
    """ a main function """
    
    rclpy.init() 
    
    node = rclpy.create_node("arm_client")    
    rclpy.spin_once(node, timeout_sec=1)
    node.get_logger().info("init_ur5: direct control mode")

    # goal positions over time----------------------
    theta    = [0,0,0,-1.57,0,0]
    duration = 3 #second

    # construct a goal message
    g = FollowJointTrajectory.Goal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points.append(
        JointTrajectoryPoint(positions=theta, velocities=[0]*6,
                                 time_from_start=Duration(sec=int(duration),
            nanosec=int((duration-int(duration))*1e9)))
        )    
    # ----------------------------------------------

    move_joint(node, g)
    node.destroy_node()

    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
        


