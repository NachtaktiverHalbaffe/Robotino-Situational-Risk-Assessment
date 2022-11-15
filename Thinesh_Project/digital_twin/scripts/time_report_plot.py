#!/usr/bin/env python3

# Agent.py
 
import numpy as np
 
class Agent:
    """
    The Base class that is implemented by
    other classes to avoid the duplicate 'choose_action'
    method
    """
    def choose_action(self, state):
        action = 0
        if np.random.uniform(0, 1) < self.epsilon:
            action = self.action_space.sample()
        else:
            action = np.argmax(self.Q[state, :])
        return action


actionSpace = np.arange (0, 0.21, 0.01)

print(actionSpace)

import h5py
import numpy as np
import h5py
import os
import rospkg
import numpy as np
import matplotlib.pyplot as plt

import random
rospack = rospkg.RosPack()


path12 = os.path.join(rospack.get_path("digital_twin"),'logs','report','17_07_extended_final_with1cm_offset','log_17_07_2021_2cm_learning_07_extend.txt')


with open(path12) as fp:
        Lines = fp.readlines()
        diff = []
        count= 0
        vel_time=0
        odom_time=0
        for line in Lines:
                if 'velTwist' in line:
                    print(line.split(',')[3].split('[')[1].split(']')[0])
                    vel_time = line.split(',')[3].split('[')[1].split(']')[0]
                if 'real_data' in line:
                    #print(round(float(line.split('[')[0]),5))
                    #value = round(float(line.split('\n')[0]),5)
                    #print(line.split('[')[2].split(']')[0])
                    odom_time = line.split('[')[2].split(']')[0]
                
                if odom_time > vel_time:
                    diff.append(count,odom_time-vel_time)
                    count += 1


plt.scatter(*zip(*diff),color='green',linewidths=1)
#plt.scatter(*zip(*reward),color='red',linewidths=1)
#plt.scatter(*zip(*s),linewidths=1)




plt.xlabel("steps")
plt.ylabel("velocity")

plt.show()