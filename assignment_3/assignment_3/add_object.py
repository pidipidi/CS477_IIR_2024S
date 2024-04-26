#!/usr/bin/env python
"""
Copyright 2020 Daehyung Park

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
import rclpy
import numpy as np, math
import time
from gazebo_msgs.srv import SpawnEntity

from manip_challenge import add_object as ao


def main():

    rclpy.init()
    
    node = rclpy.create_node("change_world")
    time.sleep(5)


    from assignment_3.solution import move_joint_sol as mj
    arm = mj.ArmClient()
    arm.move_joint([0.1646, -1.3117, 1.9484, -2.2074, -1.5707, 0.1646])

    
    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    
    node.get_logger().info("spawn sdf objects!!")

    world2baselink = np.array([0,0,0.1])
    
    grid_size = 0.1
    X,Y,Z = np.meshgrid(np.arange(0.3,0.8,grid_size).astype(float),
                        np.arange(0.,0.1,grid_size).astype(float),
                        np.arange(0.,0.25,grid_size).astype(float))
    obstacles = np.vstack([np.ravel(X),np.ravel(Y),np.ravel(Z)]).T
    obstacles += world2baselink

    for i, pos in enumerate(obstacles):
        ao.spawn_sdf_object(node, client, 'sphere', [pos[0],pos[1],pos[2], 0, 0, 0], name="obstacle"+str(i))

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()        
