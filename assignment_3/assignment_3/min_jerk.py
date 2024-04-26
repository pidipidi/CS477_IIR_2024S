#!/usr/bin/env python3
"""
Copyright 2020 Daehyung Park

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
import numpy as np
import sys, copy

def min_jerk(start, goal, dur, freq=100):
    """ 
    A function to generate a minimum-jerk trajectory.

    Parameters
    ----------
    start : array
        a vector of start position (e.g., np.array([0,0,0]))
    goal  : array
        a vector of goal position (e.g., np.array([1,2,1]))
    dur   : float
        time duration of the resulted trajectory
    freq  : float
        time frequency (Hz)

    Returns
    -------
    time : array
           a list of progress from 0 to 1
    X    : array
           a position trajectory from a start to a goal
    Xd   : array
           a velocity trajectory
    Xdd  : array
           an acceleration trajectory
    Xddd : array
           a jerk trajectory
    """
    if type(start) in [list, tuple]: start = np.array(start)
    if type(goal) in [list, tuple]: goal = np.array(goal)
        
    D = len(start) # dimension
    P = int(freq * dur) # the number of points

    time = []
    X  = []
    Xd = []
    Xdd  = []
    Xddd = []
    for i in range(P):
        # ------------------------------------------------------
        # Place your code here
        # ------------------------------------------------------
        #t = 

        time.append(t)
        #X.append( )
        #Xd.append( )
        #Xdd.append( )
        #Xddd.append( )
        # ------------------------------------------------------

    return time, np.array(X), np.array(Xd), np.array(Xdd), np.array(Xddd)


def plot_traj(t, pos, vel, acc=None, jerk=None):
    """ Plot trajectories over joints"""
    D = len(pos[0])
    
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(12,8))

    for i in range(D):
        ax = fig.add_subplot(D, 4, i*4+1)
        if i==0: ax.set_title("Pos.")
        plt.plot(t, pos[:,i], 'r-', markersize=12)
        ax.set_ylabel("Dimension {}".format(i+1))

        ax = fig.add_subplot(D, 4, i*4+2)
        if i==0: ax.set_title("Vel.")
        plt.plot(t, vel[:,i], 'r-', markersize=12)

        ax = fig.add_subplot(D, 4, i*4+3)
        if i==0: ax.set_title("Acc.")
        if acc is not None: plt.plot(t, acc[:,i], 'r-', markersize=12)

        ax = fig.add_subplot(D, 4, i*4+4)
        if i==0: ax.set_title("Jerk.")
        if jerk is not None: plt.plot(t, jerk[:,i], 'r-', markersize=12)
        
    plt.show()


def main(args=None):
    
    # start and goal
    start = np.array([-4.0, 10.0, 4.0])
    goal  = np.array([4.0, 4.0, 4.0])
    N,D   = 2, len(start)

    freq     = 100 #Hz 
    duration = 5.
    
    t, trj, trj_vel, trj_acc, trj_jerk = min_jerk(start, goal, duration, freq)

    # Visualize the trajectories
    plot_traj(t, trj, trj_vel, trj_acc, trj_jerk)

    
if __name__ == '__main__':
    main()

