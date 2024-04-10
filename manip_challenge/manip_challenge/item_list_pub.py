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


def send_item_list(node):
    
    publisher = node.create_publisher(String, '/task_commands', 10)

    d = {"storage_left": ['book', 'eraser', 'soap2'],
             "storage_right": ['snacks', 'biscuits', 'glue', 'soap'] }
        
    msg = String()
    msg.data = json.dumps(d)
    publisher.publish(msg)

    
        
def main():
    rclpy.init()

    node = rclpy.create_node("item_list_pub")    
    rclpy.spin_once(node, timeout_sec=1)
    node.get_logger().info("item_list_pub: publish task commands")

    # Send a list of commands
    send_item_list(node)
    
    node.destroy_node()
    rclpy.shutdown()        
    
if __name__ == '__main__':
    main()
