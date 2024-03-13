#!/usr/bin/env python3
"""
Copyright 2020 Daehyung Park

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
import rclpy
import time
import sys
import argparse
import numpy as np

import rclpy.node
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

import time

import matplotlib.pyplot as plt


class ActionClient(Node):

    def __init__(self):
        """
        Initialize the action client
        """
        self.node = rclpy.create_node(self.__class__.__name__)

        self._joint_states = None
        
        self._pub = self.node.create_publisher(Float64MultiArray, "ur5_controller/commands", 1) #, qos_profile=qos_profile_sensor_data)
        self._sub = self.node.create_subscription(JointState, "joint_states", self.state_callback, qos_profile=qos_profile_sensor_data)


        
    def state_callback(self, message):
        """
        Callback method for the subscriber of JointState
        """
        self._joint_states = message


    def get_joint_states(self):
        """ 
        Get current joint states
        """
        
        rclpy.spin_once(self.node)

        # take joint states
        joint_states = self._joint_states
        while rclpy.ok() and joint_states is None:
            rclpy.spin_once(self.node)
            joint_states = self._joint_states

        ## print(joint_states)
        pos = joint_states.position[0]
        vel = joint_states.velocity[0]
        sec = self.node.get_clock().now()
        
        return pos, vel, sec

    def move(self, des_angles, duration=3.0):
        """ """
        self.node.get_logger().info("Start a primitive!")
        # Default p/d gains
        Kp = -1.0
        Kd = -1.0
        
        # Enter Kp, Kd to tune PD controller
        print("- SELECT P / D GAINS")
        error = True
        while (error == True):
            input_Kp = input ("  Please enter P gain (default: -1.0): ")
            if (input_Kp == ""):
                break            
            try:
                Kp = float(input_Kp)
                error = False
                
            except:
                print("  !! Please enter a valid float type gain !!")
                time.sleep(1)
        print("")
        error = True
        while (error == True):
            input_Kd = input ("  Please enter D gain (default: -1.0): ")
            if (input_Kd == ""):
                break            
            try:
                Kd = float(input_Kd)
                error = False
                
            except:
                print("  !! Please enter a valid float type gain !!")
                time.sleep(1)
        print("")
        

        action_msg = Float64MultiArray()
        start_time = self.node.get_clock().now()
        last_time = start_time
        pos_array = []
        
        while rclpy.ok():

            pos, vel, sec = self.get_joint_states()
            # print(sec, pos)
            # if len(pos_array) == 0 or sec != pos_array[-1][0]:
                # pos_array.append((sec, pos))
            pos_array.append(((sec-start_time).nanoseconds*1e-9, pos))
                
                

            # set a joint action command
            # -------------------------------------------------
            # ADD YOUR CODE
            #--------------------------------------------------
            # Design your (serial or parallel) PD controller
            # pos_err = 
            # vel_err =             
            # action = 

            ## Execute "action"
            # action_msg.....
            #--------------------------------------------------
            self._pub.publish(action_msg)

            # wait with 1000hz
            while rclpy.ok():
                time_now = self.node.get_clock().now()
                if time_now - last_time > rclpy.duration.Duration(nanoseconds=1e+7):
                    last_time = time_now
                    break
                rclpy.spin_once(self.node, timeout_sec=0.0)

            if self.node.get_clock().now() - start_time > rclpy.duration.Duration(seconds=duration):
                self.node.get_logger().info("Time Over!")
                break
                
            # rate.sleep sleeps forever. So manually implemented..
            ## rate.sleep()

        # Plot graph
        x, y = zip(*pos_array)

        plt.figure(figsize=(10, 6))
        plt.plot(x, y)
        plt.xlabel('t')
        plt.ylabel('/joint_states/position[0]')
        plt.grid(True)
        plt.show()

            
               
def main(args=None): # argv=sys.argv):
    
    rclpy.init()
    args = get_args(sysargv=sys.argv)[0]

    # create a class of action client
    ac = ActionClient()

    pos1 = np.float64([0.0])
    pos2 = np.float64([0.4])
    pos3 = np.float64([-0.4])
    
    ac.move(pos1)

    # repeat movements
    while rclpy.ok():
        ac.move(pos2, 5.0)
        ac.move(pos3, 5.0)
        
    rclpy.shutdown()


def get_args(sysargv=None):
    parser = argparse.ArgumentParser(
        description=" ")
    argv = sysargv[1:] if sysargv is not None else sys.argv[1:]
    args = parser.parse_known_args(argv)
    return args
    
if __name__ == '__main__':
    main()
        
