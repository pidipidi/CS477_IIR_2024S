#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time


class PoseStampPublisher(Node):
    def __init__(self):
        super().__init__('posestamp_publihser')
        time.sleep(1)
        
        self.get_logger().info('Running ...')
        self.timer = self.create_timer(0.1, self.pub_goal) # 10Hz
        self.pub = self.create_publisher(PoseStamped, 'test_goal', 10)
        
        self.ps = PoseStamped()
        self.ps.header.frame_id = 'base_link'
        self.ps.pose.position.x = 0.9
        self.ps.pose.position.y = 0.23
        self.ps.pose.position.z = 0.9
        self.ps.pose.orientation.x = 0.
        self.ps.pose.orientation.y = 0.
        self.ps.pose.orientation.z = 0.
        self.ps.pose.orientation.w = 1.

        
    def pub_goal(self):
        self.ps.header.stamp = self.get_clock().now().to_msg()
        self.ps.pose.position.x += 0.005
        self.pub.publish(self.ps)
        
def main(args=None):
    rclpy.init(args=args)
    posestamp_publisher = PoseStampPublisher()
    rclpy.spin(posestamp_publisher)
    posestamp_publisher.destroy_node()
    rclpy.shutdown()
        

if __name__ == '__main__':
    main()
    

    
