#!/usr/bin/env python3
"""
Copyright 2020 Daehyung Park

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
import gym
import gym_gazebo2
import numpy as np
import random

import os
os.system("kill $(ps aux | grep 'gz' | awk '{print $2}')")
os.system("kill $(ps aux | grep 'ros' | awk '{print $2}')")

MAX_ITERATIONS = 10**3 # 50
NUM_EPISODES = 10000

def main():

    env = gym.make('CartPole-v0')
    env.theta_threshold_radians = 30./180.*np.pi

    for episode in range(NUM_EPISODES):
        observation = env.reset()
        observation = env.reset()
        
        pos_error = 0
        vel_error = 0
        integral  = 0
        
        iteration = 0
        while True:
            iteration += 1

            # -------------------------------------------------
            # ADD YOUR CODE
            #--------------------------------------------------
            # Design your (serial or parallel) PID controller
            #Kp = 
            #Kd = 
            #Ki = 



            action = np.random.uniform()
            
            observation, reward, done, info = env.step(action)
            # -------------------------------------------------

            if not done and iteration>MAX_ITERATIONS:                
                break            
            if done:
                break
        
    env.close()

            
if __name__ == "__main__":
    main()
