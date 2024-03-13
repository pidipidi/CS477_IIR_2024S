#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose

import numpy as np
import time

class PoseArrayPublisher(Node):
    def __init__(self):
        super().__init__('posestamp_publihser')
        time.sleep(1)
        
        self.get_logger().info('Running ...')
        self.timer = self.create_timer(0.1, self.pub_goal) # 10Hz
        self.pub = self.create_publisher(PoseArray, 'test_goal', 10)
        self.pa = PoseArray()
        self.pa.header.frame_id = 'base_link'
        
        z_list = np.linspace(0.4,1.2,20)
        y_list = np.linspace(-0.4,0.4,20)
        for i in range(len(y_list)):
            pose = Pose()
            pose.position.x = 0.9
            pose.position.y = y_list[i]
            pose.position.z = z_list[i]
            pose.orientation.x = 0.
            pose.orientation.y = 0.
            pose.orientation.z = 0.
            pose.orientation.w = 1.
            self.pa.poses.append(pose)
        
    def pub_goal(self):
        self.pa.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.pa)
        
def main(args=None):
    rclpy.init(args=args)
    posearray_publihser = PoseArrayPublisher()
    rclpy.spin(posearray_publihser)
    posearray_publihser.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
    
    
