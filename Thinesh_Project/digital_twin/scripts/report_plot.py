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

import random
rospack = rospkg.RosPack()
names=['2cm_limit_direct.txt', 'better_rl_01.txt', 'better_rl_02.txt', '2cm_limit_rl.txt', 'rl_11.txt']

names = ['rl_11.txt']
#used for the presentation
#names = ['2cm_limit_rl.txt']

os.path.join(rospack.get_path("digital_twin"),'logs','report','qTable_17_07_2021_2cm_07_extend.hdf5')

for fileName in names:
    pathName = os.path.join('/home/cpfactory/digitalTwin_ws/src/IDT/digital_twin/logs/report/17_07_extended_final_with1cm_offset/demo',fileName)

    r=[]
    highlight = []
    with open(pathName) as fp:
        Lines = fp.readlines()
        

        count= 0
        for line in Lines:
                if 'CrewardList' in line: 
                    break
                if '0.0' in line:
                    print(round(float(line.split('\n')[0]),5))
                    value = round(float(line.split('\n')[0]),5)
                    
                    if value > 0.02:
                        print(value,float(line.split('\n')[0]))
                        highlight.append((count,value))
                    else:
                        r.append((count,value))
                    count += 1


    import numpy as np
    import matplotlib.pyplot as plt




    x = [1. , 2., 3.5]
    y = [2.3, 4., 6.]
    #plt.xlim(0,4)

    plt.axhline(y=0.02, color='blue', linestyle='dotted',label='2cm limit value')
    plt.stem(*zip(*highlight),'r',markerfmt='ro',label='Deviation value above 2cm = reset position')

    #fig, ax = plt.subplots()
    #plt.title(pathName[-50:])

    #plt.title(fileName)
    #plt.title('Position deviation - simulation and real Robotino with offset')
    #plt.scatter(*zip(*r),color='red',linewidths=1, label='position deviation value')
    #(markers, stemlines, baseline) = plt.stem(*zip(*r))
    #plt.setp(stemlines, 'linestyle', 'dotted')


    markerline, stemlines, baseline = plt.stem(*zip(*r),'green', markerfmt='go', label='Deviation value below 2cm')
    plt.setp(stemlines, 'color', plt.getp(markerline,'color'))
    plt.setp(stemlines, 'linestyle', 'dotted')
    #plt.scatter(*zip(*highlight),color='blue',linewidths=1, label='reset position')
    #plt.plot([], [], ' ', label="Extra label on the legend")


    plt.xlabel("Consecutive sample(at ~1cm)")
    plt.ylabel('Positon deviation (in meters)')
    plt.figtext(0.5, 0.93, "Position deviation between simulation and real Robotino with offset", ha="center", fontsize=12, bbox={"facecolor":"orange", "alpha":0.5, "pad":5})
    plt.legend()
    plt.show()

    #x = [1,2,3,4,5,6]
    #y = [3,4,5,6,7,8]

    #plt.plot(x[1:], y[1:], 'ro')
    #plt.plot(x[0], y[0], 'g*')
