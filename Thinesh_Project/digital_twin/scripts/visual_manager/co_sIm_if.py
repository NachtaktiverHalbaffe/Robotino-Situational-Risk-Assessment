#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import random

from numpy.core import numeric


action = np.arange (0.01, 0.31, 0.01)
print(len(action))

number = round(0.01111111,2)
if number in action :
    print(number, 'is present in the range.')
else :
    print(number, 'is not present in the range.')


sampl = np.random.uniform(low=0.01, high=0.3, size=1)
print(random.choice (action))

print(max(action))
np.arange (0.01, 0.31, 0.01)
action1 = round(random.choice (action),2)
print('action1',action1)
print(max(np.clip(action1,0.01,0.3)))