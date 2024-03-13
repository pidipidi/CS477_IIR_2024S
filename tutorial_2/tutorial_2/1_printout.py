#!/usr/bin/env python
import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    minimal_printout = Node('minimal_printout')
    minimal_printout.get_logger().info("This is log-information!")
    minimal_printout.get_logger().warn("This is log-warning!")
    minimal_printout.get_logger().error("This is log-error!")
    
    print("These are ROS printouts!")
    
    minimal_printout.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
