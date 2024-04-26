#!/usr/bin/env python
import numpy as np
import gym
import cs477_gym
from sklearn.neighbors import BallTree

import astar


def generate_obstacles(xlim, ylim):
    """ 
    A function to generate obstacles

    Parameters
    ----------
    xlim : list
        [lower limit, upper limit] of x coordinate
    xlim : list
        [lower limit, upper limit] of y coordinate

    Returns
    -------
    obstacles : list
        a list of obstacle coordinates.
    obstacle_tree : BallTree 
        a BallTree object from Scikit-Learn. The tree includes
        the coordinates of obstacles.
    """
    obstacles   = []
    #0
    for i in range(0, int(xlim[1]*2/3)):
        obstacles.append([i, ylim[1]])
    for i in range(int(xlim[1]*2/3), xlim[1]):
        obstacles.append([i, ylim[1]/3. ])
    for i in range(int(ylim[1]/3), ylim[1]):
        obstacles.append([xlim[1]*2/3, i ])
    #1
    for i in range(0, int(xlim[1]/3)):
        obstacles.append([i, ylim[1]*2/3])
    for i in range(int(xlim[1]/3), xlim[1]):
        obstacles.append([i, 0 ])
    for i in range(0, int(ylim[1]*2/3)):
        obstacles.append([xlim[1]/3, i ])


    for i in range(int(ylim[1]*1/4), int(ylim[1]*3/4)):
        obstacles.append([xlim[1]/2, i ])
        
        
    obstacle_tree = BallTree(np.array(obstacles))
        
    return obstacles, obstacle_tree



if __name__ == '__main__':

    # Initialize variables
    start = [5.,55.]
    goal  = [55.,5.]
    obstacles, obstacle_tree = generate_obstacles([0,60],[0,60])
    grid_size  = 1.0
    robot_size = 0.5

    # actions 
    actions = [[-1,0], [0,-1], [1,0], [0,1]]
    # if you want to use below, please specify the actions on the report.
    ## actions = [[-1,0], [0,-1], [1,0], [0,1],
    ##            [-1,-1],[-1,1],[1,-1],[1,1],]

    # initialize openai gym
    env = gym.make("reaching-v0")
    env.set_start_state(start)
    env.set_goal_size(1.0)
    env.set_goal_state(goal)
    env.set_objects(obstacles)
    env.set_robot_size(robot_size)    
    env.reset()

    # initialize limits
    grid_limits = [env.observation_space.low,
                   env.observation_space.high]

    # run your algorithm
    resolution = 1.
    path = astar.astar_planning(start, goal, actions,
                                    resolution, grid_limits,
                                    obstacle_tree, robot_size)

    
    for i, p in enumerate(path):
        if i==0: continue
        #------------------------------------------------------------
        # ADD YOUR CODE
        #------------------------------------------------------------
        # action = ...
        
        #------------------------------------------------------------
        env.render()
        env.step(action)

    ret = input("Press key for closing the window!")        
    env.close()
