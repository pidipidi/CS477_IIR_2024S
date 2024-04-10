#!/usr/bin/env python3
"""
Copyright 2020 Daehyung Park

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
import json
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


def get_item_list(node):
    """
    Get the current item list (i.e., command)
    """    
    global task_commands
    
    def item_list_callback(msg):
        global task_commands
        task_commands = msg.data
        node.get_logger().info("{}".format(task_commands))
    
    node.create_subscription(String, '/task_commands',
                             item_list_callback, 10)


        
def main():
    rclpy.init()

    node = rclpy.create_node("item_list_sub")    
    rclpy.spin_once(node, timeout_sec=1)
    node.get_logger().info("item_list_sub: subscribe task commands")

    # Get a list of commands
    get_item_list(node)
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()        
    
if __name__ == '__main__':
    main()
