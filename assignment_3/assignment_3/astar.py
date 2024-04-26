#!/usr/bin/env python3
"""
Copyright 2020 Daehyung Park

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import math
import numpy as np

class Node:
    """
    Node class for dijkstra search
    """
    def __init__(self, pos, idx, cost, h, prev_idx):
        self.pos  = np.array(pos)
        self.idx  = idx
        self.cost = cost
        self.h    = h
        self.prev_idx = prev_idx # previous node's index

    def __str__(self):
        return str(self.x) + "," + str(self.cost) + "," + str(self.prev_idx)


def get_grid_index(state, resolution, limits, grid_dim):
    """ 
    A function to convert a state to the index of grid.

    Parameters
    ----------
    state : list
        a list of coordinate values
    resolution : float
        a value of grid resolution
    grid_limits : list
        a list of minimum and maximum configuration limits, where
        each limit is a list of float values.
        (e.g., [[min_x,_min_y,min_z],[max_x,max_y,max_z]])
    grid_dim : list
        a list of grid dimensions.

    Returns
    -------
     : Integer
        The index of the input state in grid.
    """
    if type(state) in [list, tuple]: state = np.array(state)
        
    ids = np.round((state-limits[0])/resolution).astype(int)
    if len(state)==2:
        idx = ids[0]*grid_dim[1]+ids[1]
        if idx<0 or idx>=grid_dim[0]*grid_dim[1]:
            print("out of grid: {}".format(idx) )
        
    elif len(state)==3:
        idx = ids[0]*grid_dim[1]*grid_dim[2]+ids[1]*grid_dim[2]+ids[2]
        if idx<0 or idx>=grid_dim[0]*grid_dim[1]*grid_dim[2]:
            print("out of grid: {}".format(idx) )
        
    else:
        return NotImplemented
    
    return idx


def get_grid_pos(state, resolution, limits):
    """ 
    A function that return the nearest coordinate on the grid.

    Parameters
    ----------
    state : list
        a list of coordinate values
    resolution : float
        a value of grid resolution
    grid_limits : list
        a list of minimum and maximum configuration limits, where
        each limit is a list of float values.
        (e.g., [[min_x,_min_y,min_z],[max_x,max_y,max_z]])

    Returns
    -------
    new_state : list
        a list of the nearest coordinate values on the grid
    """
    if type(state) in [list, tuple]: state = np.array(state)
        
    ids = np.round((state-limits[0])/resolution).astype(int)

    new_state = limits[0] + resolution*ids
    return new_state.tolist()


def is_valid(state, grid_limits, obstacle_tree, robot_size):
    """ 
    A function to check the validity of the input state.

    Parameters
    ----------
    state : list
        a list of coordinate values
    grid_limits : list
        a list of minimum and maximum configuration limits, where
        each limit is a list of float values.
        (e.g., [[min_x,_min_y,min_z],[max_x,max_y,max_z]])
    obstacle_tree : BallTree 
        a BallTree object from Scikit-Learn. The tree includes
        the coordinates of obstacles.
    robot_size : float
        a radius of a robot. This value is used for collision checking.       

    Returns
    -------
     : boolean
        True if the input state is valid
    """
    ## from IPython import embed; embed(); sys.exit()
    
    # check workspace limits
    if any( state[i] < grid_limits[0][i] for i in range(len(grid_limits[0])) ):
        return False
    if any( state[i] > grid_limits[1][i] for i in range(len(grid_limits[1])) ):
        return False

    # check collision
    if len(np.shape(state))==1: state=state[np.newaxis,:]
    count = obstacle_tree.query_radius(state, robot_size, count_only=True)
    if count>0: return False
    return True
    
    
def astar_planning(start, goal, actions, resolution, grid_limits,
                       obstacle_tree, robot_size, **kwargs):
    """ 
    A function to generate a path from a start to a goal 
    using A* search-based planning algorithm.  

    Parameters
    ----------
    start : list
        a list of start coordinate values (e.g., [x,y,z]).  
    goal : list
        a list of goal coordinate values (e.g., [x,y,z]).  
    actions : list
        a list of available actions, where each action is a list
        of float values (e.g., [[vx,vy,vz],[vx,vy,vz], ..., ]).  
    resolution : float
        a value of grid resolution
    grid_limits : list
        a list of minimum and maximum configuration limits, where
        each limit is a list of float values.
        (e.g., [[min_x,_min_y,min_z],[max_x,max_y,max_z]])
    obstacle_tree : BallTree 
        a BallTree object from Scikit-Learn. The tree includes
        the coordinates of obstacles.
    robot_size : float
        a radius of a robot. This value is used for collision checking.       

    Returns
    -------
    path : list
        a list of coordinate values that is the shortest path from 
        the start to the goal.
    """
    if type(grid_limits) is not np.ndarray: grid_limits = np.array(grid_limits)
    grid_dim    = np.round((grid_limits[1]-grid_limits[0])/resolution+1).astype(int)
    
    openset, closedset = dict(), dict()

    # Set a start node
    start_node = Node(start,
                      get_grid_index(start, resolution, grid_limits, grid_dim),
                      0, np.linalg.norm(np.array(start)-goal), -1)
    openset[start_node.idx] = start_node

    # Set a goal node
    goal_node = Node(goal,
                     get_grid_index(goal, resolution, grid_limits, grid_dim),
                     0, 0, -1)
    if start_node.idx==goal_node.idx: return []
    print("Start and goal indices: {} and {}".format(start_node.idx,
                                                         goal_node.idx))

    
    while True:
        # Empty openset
        if len(openset) == 0: return None

        #------------------------------------------------------------
        # ADD YOUR CODE
        #------------------------------------------------------------
        # Select the minimum value node as the current node (index)
        #cur_idx  = min(openset, key=lambda o: ....) 
        #cur_node = ...

        
        #------------------------------------------------------------

        # When found the goal index
        if cur_idx == goal_node.idx :
            print("goal found! {}, {}".format(cur_idx, cur_node.prev_idx))
            goal_node.prev_idx = cur_node.prev_idx
            goal_node.cost = cur_node.cost
            goal_node.h    = cur_node.h
            break

        # Remove the item from the open set
        del openset[cur_idx]

        #------------------------------------------------------------
        # ADD YOUR CODE
        #------------------------------------------------------------        
        # Add it to the closed set
        closedset[cur_idx] = cur_node

        # expand nodes based on available actions
        for i, action in enumerate(actions): 
            next_pos = cur_node.pos+action

            # ...







            
            
        #------------------------------------------------------------

    # Track the path from goal to start
    path = [goal_node.pos]    
    #------------------------------------------------------------
    # ADD YOUR CODE
    #------------------------------------------------------------
    # From the goal node, iterative find the previous node 
    # ...
    
    #while prev_idx != start_node.idx:
    #   ...    

    
    #------------------------------------------------------------
    return path[::-1]

                

