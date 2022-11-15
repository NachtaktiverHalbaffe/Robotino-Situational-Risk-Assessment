#!/usr/bin/env python3
import numpy as np
'''
# R matrix
R = np.matrix([ [-1,-1,-1,-1,0,-1],
                [-1,-1,-1,0,-1,100],
                [-1,-1,-1,0,-1,-1],
                [-1,0,0,-1,0,-1],
                [-1,0,0,-1,-1,100],
                [-1,0,-1,-1,0,100] ])

# Q matrix
Q = np.matrix(np.zeros([6,6]))

# Gamma (learning parameter).
gamma = 0.8

# Initial state. (Usually to be chosen at random)
initial_state = 5

print(R)
print(Q)

# This function returns all available actions in the state given as an argument
def available_actions(state):
    current_state_row = R[state,]
    print(current_state_row)
    print(np.where(current_state_row >= 0))
    av_act = np.where(current_state_row >= 0)[1]
    print('av_act', av_act)
    return av_act

# Get available actions in the current state
available_act = available_actions(initial_state) 
print('available_act', available_act)

# This function chooses at random which action to be performed within the range 
# of all the available actions.
def sample_next_action(available_actions_range):
    next_action = int(np.random.choice(available_act,1))
    return next_action

# Sample next action to be performed
action = sample_next_action(available_act)
print(action)


# This function updates the Q matrix according to the path selected and the Q 
# learning algorithm
def update(current_state, action, gamma):
    print('current_state',current_state,'action', action)
    print('action',Q[action,],'maxAction',np.max(Q[action,]))
    max_index = np.where(Q[action,] == np.max(Q[action,]))[1]
    print('max_index',np.where(Q[action,] == np.max(Q[action,])))
    if max_index.shape[0] > 1:
        max_index = int(np.random.choice(max_index, size = 1))
    else:
        max_index = int(max_index)
    max_value = Q[action, max_index]
    
    # Q learning formula
    Q[current_state, action] = R[current_state, action] + gamma * max_value

# Update Q matrix
update(initial_state,action,gamma)



#-------------------------------------------------------------------------------
# Training

# Train over 10 000 iterations. (Re-iterate the process above).
for i in range(10):
    current_state = np.random.randint(0, int(Q.shape[0]))
    available_act = available_actions(current_state)
    action = sample_next_action(available_act)
    update(current_state,action,gamma)
    
# Normalize the "trained" Q matrix
print("Trained Q matrix:")
print(Q/np.max(Q)*100)

#-------------------------------------------------------------------------------
# Testing

# Goal state = 5
# Best sequence path starting from 2 -> 2, 3, 1, 5

current_state = 2
steps = [current_state]



while current_state != 5:

    next_step_index = np.where(Q[current_state,] == np.max(Q[current_state,]))[1]
    
    if next_step_index.shape[0] > 1:
        next_step_index = int(np.random.choice(next_step_index, size = 1))
    else:
        next_step_index = int(next_step_index)
    
    steps.append(next_step_index)
    current_state = next_step_index

# Print selected sequence of steps
print("Selected path:")
print(steps)
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from tf.transformations import *



def realCallback(data):
    global real_data
    real_odom = data
    # get the position in x, y and yaw values
    q1 = [real_odom.pose.pose.orientation.x,real_odom.pose.pose.orientation.y,real_odom.pose.pose.orientation.z,real_odom.pose.pose.orientation.w]
    euler_real = euler_from_quaternion(q1)
    real_data = ['real_data',real_odom.pose.pose.position.x, real_odom.pose.pose.position.y,euler_real[2],real_odom.header.stamp]
    print(real_data)

def simCallback(data):
    global sim_data
    
    sim_odom = data
    # need position.x position.y and rotatioin.z
    q2 = [sim_odom.pose.pose.orientation.x ,sim_odom.pose.pose.orientation.y,sim_odom.pose.pose.orientation.z, sim_odom.pose.pose.orientation.w]
    euler_sim = euler_from_quaternion(q2)
    sim_data = ['sim_data',sim_odom.pose.pose.position.x-1.240, sim_odom.pose.pose.position.y -4.05,euler_sim[2],sim_odom.header.stamp]
    print(sim_data)

def velCallback(velTwist):
    vel_cmd = velTwist
    # need linear.x and angular.z
    cmd_data = ['velTwist',velTwist.linear.x,velTwist.angular.z, rospy.get_rostime()]
    print(cmd_data)




if __name__ == '__main__':
    print('deviation')
    rospy.wait_for_service('/gazebo/set_model_state')
    sim_data =[]
    real_data =[]
    rospy.Subscriber('/sim_odom', Odometry, simCallback, queue_size=10)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, realCallback, queue_size=10)
    rospy.Subscriber('/cmd_vel', Twist, velCallback, queue_size=10)    
    rospy.init_node('deviation', anonymous=True)
    #rospy.on_shutdown(myhook)
    rospy.spin()
'''

from mpl_toolkits import mplot3d


import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure()

ax = plt.axes(projection='3d')

# Data for a three-dimensional line
zline = np.linspace(0, 15, 1000)
xline = np.sin(zline)
yline = np.cos(zline)
ax.plot3D(xline, yline, zline, 'gray')

# Data for three-dimensional scattered points
zdata = 15 * np.random.random(100)
xdata = np.sin(zdata) + 0.1 * np.random.randn(100)
ydata = np.cos(zdata) + 0.1 * np.random.randn(100)
ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Greens')
plt.show()
print('done')

'''