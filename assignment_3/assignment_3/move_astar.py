#!/usr/bin/env python3
"""
Copyright 2020 Daehyung Park

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
import time

import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose

import numpy as np
from sklearn.neighbors import BallTree

from assignment_3 import move_joint as mj
from assignment_3 import astar 
from assignment_3 import quaternion


def generate_obstacles(resolution=0.05):
    """ 
    A function to generate obstacles

    Parameters
    ----------
    resolution : float
        a value of grid resolution

    Returns
    -------
    obstacles : list
        a list of obstacle coordinates.
    obstacle_tree : BallTree 
        a BallTree object from Scikit-Learn. The tree includes
        the coordinates of obstacles.
    """
    X,Y,Z = np.meshgrid(np.arange(0.3,0.8,resolution).astype(float),
                        np.arange(0.,0.1,resolution).astype(float),
                        np.arange(0,0.25,resolution).astype(float))
    obstacles = np.vstack([np.ravel(X),np.ravel(Y),np.ravel(Z)]).T
    obstacle_tree = BallTree(obstacles)
    
    return obstacles, obstacle_tree


def test(arm):
    """ 
    A function to generate obstacle avoidance motion

    Parameters
    ----------
    arm : class object
        a class object to control the robot

    """

    # initialize variables
    resolution  = 0.025
    robot_size = 0.1    
    actions = [[-1,-1,-1], [-1,0,-1], [-1,1,-1], [0,-1,-1],
               [0,0,-1], [0,1,-1], [1,-1,-1], [1,0,-1], [1,1,-1],
               [-1,-1,0], [-1,0,0], [-1,1,0], [0,-1,0],
               [0,1,0], [1,-1,0], [1,0,0], [1,1,0],
               [-1,-1,1], [-1,0,1], [-1,1,1], [0,-1,1], [0,0,1],
               [0,1,1], [1,-1,1], [1,0,1], [1,1,1] ]
    actions = np.array(actions)*resolution
    grid_limits = [[0.4,-0.4,0.0],[0.7,0.3,0.5]]
    obstacles, obstacle_tree = generate_obstacles()
    
    # initialize the robot
    arm.move_joint([0.1646, -1.3117, 1.9484, -2.2074, -1.5707, 0.1646])
    time.sleep(3)

    # get the start pose
    start_pose = arm.fk_request(arm.js_joint_position)

    # set the goal pose
    goal_pose = Pose()
    goal_pose.position.x = 0.52
    goal_pose.position.y = -0.2
    goal_pose.position.z = 0.1677 - 0.15
    goal_pose.orientation.x = 0.707
    goal_pose.orientation.y = 0.707
    goal_pose.orientation.z = 0.0
    goal_pose.orientation.w = 0.0

    start = [start_pose.position.x,start_pose.position.y,start_pose.position.z] 
    goal  = [goal_pose.position.x,goal_pose.position.y,goal_pose.position.z]

    # get the nearest pose on the grid before planning
    start_on_grid = astar.get_grid_pos(start, resolution,
                                          grid_limits )
    goal_on_grid = astar.get_grid_pos(goal, resolution,
                                          grid_limits )

    # run the planning algorithm
    path = astar.astar_planning(start_on_grid, goal_on_grid, actions,
                                resolution, grid_limits,
                                obstacle_tree, robot_size)
    pos_traj = [np.array(start)] + path + [np.array(goal)]

    
    # smoothing if you can
    # ...
    
    pose_traj = []
    for i, pos in enumerate(pos_traj):
        p = Pose()
        # ------------------------------------------------------
        # Place your code here
        # ------------------------------------------------------
        # position
        #p.position.x = ...
        #p.position.y = ...
        #p.position.z = ...
        
        # orientation (You can use the SLERP function in quaternion.py)
        #p.orientation = ...
        
        # ------------------------------------------------------
        pose_traj.append(p)
    
    arm.move_pose_trajectory(pose_traj, duration=5.)



def main(args=None):
    """ a main function """
    
    rclpy.init() 
    node = rclpy.create_node("move_astar")    
    rclpy.spin_once(node, timeout_sec=1)
    node.get_logger().info("init_ur5: direct control mode")

    # create action client
    arm = mj.ArmClient()
    test(arm)    

    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
