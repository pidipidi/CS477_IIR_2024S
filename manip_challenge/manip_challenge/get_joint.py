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

from sensor_msgs.msg import JointState


JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
def get_joint_angles(node):
    """ 
    Get the current joint state according to the order of JOINT_NAMES.

    Parameters
    ----------
    node:  
        a ROS2 node handle
    """
    global _js_joint_name, _js_joint_position, _js_joint_velocity
    _js_joint_name     = None
    _js_joint_position = None
    _js_joint_velocity = None
    
    #callback function: when a joint_states message arrives, save the values
    def _joint_states_callback(msg):
        global _js_joint_name, _js_joint_position, _js_joint_velocity
        _js_joint_name     = msg.name
        _js_joint_position = msg.position
        _js_joint_velocity = msg.velocity

    node.create_subscription(JointState, 'joint_states',
                             _joint_states_callback,
                                 10)
            
    rate = node.create_rate(10)
    while rclpy.ok():
        if _js_joint_name is None: 
            rclpy.spin_once(node, timeout_sec=0)
            continue

        angles = []
        for ns in JOINT_NAMES:
            for i, joint_name in enumerate(_js_joint_name):
                if joint_name.find(ns)>=0:
                    angles.append(_js_joint_position[i])
                    
        if len(angles)==6:
            return angles
        rate.sleep()
    


def main(args=None):
    
    rclpy.init() 
    
    node = rclpy.create_node("get_joint")    
    rclpy.spin_once(node, timeout_sec=1)
    node.get_logger().info("Subscribe joint states")

    s = get_joint_angles(node)
    node.get_logger().info("{}".format(s))
    
    node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
        



