import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from tf.transformations import *
import gym
import itertools
import matplotlib
import matplotlib.style
import numpy as np
import pandas as pd
import sys




print(np.arange (0.01, 0.1, 0.01))