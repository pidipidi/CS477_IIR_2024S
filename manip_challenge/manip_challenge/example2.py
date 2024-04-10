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
import time

from riro_srvs.srv import StringInt, StringPose

def get_object_pose(node, name):
    """ Get the pose of the designated name of object """
    
    cli_cmd = node.create_client(StringPose, "/get_object_pose")
    while not cli_cmd.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('/get_object_pose service not available, waiting again...')

    req = StringPose.Request(data=name)    
    future = cli_cmd.call_async(req)

    while rclpy.ok():
        if future.done():
            break
        rclpy.spin_once(node, timeout_sec=0)
        time.sleep(0.1)

    # printout
    node.get_logger().info(str(future.result().pose))

    return future.result().pose
    

def main(args=None):
    
    rclpy.init() 
    
    node = rclpy.create_node("example2")    
    rclpy.spin_once(node, timeout_sec=1)
    node.get_logger().info("example2: get_object_pose")

    pose = get_object_pose(node, "book")
    
    node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
        

