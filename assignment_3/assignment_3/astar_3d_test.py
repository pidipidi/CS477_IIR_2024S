#!/usr/bin/env python
import numpy as np
import gym
import cs477_gym
import astar
from sklearn.neighbors import BallTree



def generate_obstacles(resolution=1):
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
    X,Y,Z = np.meshgrid(np.arange(7,14,resolution).astype(int),
                        np.arange(7,14,resolution).astype(int),
                        np.arange(7,14,resolution).astype(int))
    obstacles = np.vstack([np.ravel(X),np.ravel(Y),np.ravel(Z)]).T
    obstacle_tree = BallTree(obstacles)
        
    return obstacles, obstacle_tree



if __name__ == '__main__':

    # Initialize variables
    start      = [1.,1.,1.]
    goal       = [19.,19.,19.]
    resolution = 1.
    robot_size = 1.
    obstacles, obstacle_tree = generate_obstacles(resolution)
    
    # actions 
    actions = [[-1,-1,-1], [-1,0,-1], [-1,1,-1], [0,-1,-1], [0,0,-1], [0,1,-1], [1,-1,-1], [1,0,-1], [1,1,-1],
               [-1,-1,0], [-1,0,0], [-1,1,0], [0,-1,0],          [0,1,0], [1,-1,0], [1,0,0], [1,1,0],
               [-1,-1,1], [-1,0,1], [-1,1,1], [0,-1,1], [0,0,1], [0,1,1], [1,-1,1], [1,0,1], [1,1,1] ]
    # if you want to use below, please specify the actions on the report.
    ##actions = [[-1,0,0], [0,-1,0], [1,0,0], [0,1,0],[0,0,-1],[0,0,1]]
    actions = np.array(actions)*resolution
    
    # initialize openai gym
    env = gym.make("reaching-v1")
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
