#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class InitJoints(Node):

    def __init__(self):
        super().__init__('init_joints')
        self.get_logger().info("INITIALIZING INIT_JOINT NODE")
        self.client = ActionClient(self, FollowJointTrajectory, 'ur5_controller/follow_joint_trajectory')

    def send_goal(self, angles):
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().info('Action server not available after waiting')
            return
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        point = JointTrajectoryPoint()
        point.positions = angles
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(sec=3)
        goal_msg.trajectory.points.append(point)

        self.get_logger().info('Sending goal')
        self._send_goal_future = self.client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        # self.destroy_node()
        rclpy.shutdown()
        

def main(args=None):
    rclpy.init(args=args)

    init_joint_client = InitJoints()
    init_joint_client.send_goal([0., -1.8, 1., 0., 0., 0.])
    
    rclpy.spin(init_joint_client)

if __name__ == '__main__':
    main()
