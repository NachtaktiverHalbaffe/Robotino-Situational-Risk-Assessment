#!/usr/bin/env python3

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic


import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from tf.transformations import *
import matplotlib
import matplotlib.style
import numpy as np
import random

matplotlib.style.use('ggplot')

import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

from nav_msgs.msg import Odometry
from tf.transformations import *

from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
# objective is to get the cart to the flag.
# for now, let's just move randomly:


import h5py
import os
import rospkg

#env = gym.make("MountainCar-v0")

LEARNING_RATE = 0.1

DISCOUNT = 0.95
EPISODES = 1
SHOW_EVERY = 3


DISCRETE_OS_SIZE = [21]
#print(env.observation_space.high,env.observation_space.low)
discrete_os_win_size = 0.01#(env.observation_space.high - env.observation_space.low)/DISCRETE_OS_SIZE
#print(discrete_os_win_size,env.action_space.n)
# Exploration settings
#epsilon = 0.5  # not a constant, qoing to be decayed


# ---------------------------------------------------------
Learning = True
readFile = True
# ---------------------------------------------------------


print('deviation')
rospy.wait_for_service('/gazebo/set_model_state')
sim_data =[]
real_data =[]

# global values for q learning
reward = -1
current_state = 0.01
action = 0.01
new_state = 0.01

discount_factor = 1.0
alpha = 0.95
epsilon = 1

cmd_data = Twist()
rot_change = True
new_pose = True

get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
model = GetModelStateRequest()
model.model_name='robotino3'
resolution = 0.01
actionSpace = np.arange (0, 0.21, resolution)

episodeReward =0
totalReward = 0
rewardList= []


rospack = rospkg.RosPack()
savePath = os.path.join(rospack.get_path("digital_twin"),'logs','qTable_12_09.hdf5')

if not readFile:
    q_table = np.zeros((len(actionSpace),len(actionSpace)))
else:
    with h5py.File(savePath, 'r') as f:
        q_table = f['qTable'][:]
#print(q_table)


def setPose(data):
    print('----------------------setpose-----------------------')
    global episodeReward
    global totalReward
    totalReward += episodeReward
    rewardList.append(episodeReward)
    print('totalReward ', totalReward, 'episodeReward ', episodeReward, 'rewardList ', rewardList)

    state_msg = ModelState()
    state_msg.model_name = 'robotino3'

    # check map position is needed 
    state_msg.pose.position.x = data.pose.pose.position.x + 1.24
    state_msg.pose.position.y = data.pose.pose.position.y + 4.05
    state_msg.pose.position.z = 0
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = data.pose.pose.orientation.z
    state_msg.pose.orientation.w = data.pose.pose.orientation.w
    state_msg.reference_frame = 'world'

    # wait for /gazebo/set_model_state service to set the robotino position in simulation
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )
        global epsilon
        if epsilon > 0.01:
            epsilon -= 0.01#eps_decay
    except rospy.ServiceException as e:
        print("/gazebo/set_model_state Service call failed: %s"%e)
    

def realCallback(data):
    global reward
    global real_data
    global sim_data
    global new_pose

    real_data = ['real_data',data.pose.pose.position.x, data.pose.pose.position.y,data.header.stamp]
    print(real_data)

    # get the sim postiton and used it to find the deviation.
    sim_position = get_model_srv(model)

    sim_data = ['sim_data',sim_position.pose.position.x-1.240, sim_position.pose.position.y -4.05,sim_position.header.stamp]
    print(sim_data)

    deviation = math.sqrt((real_data[1] - sim_data[1])**2 +(real_data[2] - sim_data[2])**2)

    # get the difference in x, y and yaw values
    q1 = [data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
    q2 = [sim_position.pose.orientation.x ,sim_position.pose.orientation.y,sim_position.pose.orientation.z, sim_position.pose.orientation.w]
    
    euler_real = euler_from_quaternion(q1)
    euler_sim = euler_from_quaternion(q2)
    
    rotationDiff = [a-b for a,b  in zip(euler_real,euler_sim)]

    if Learning:
        # set reward value
        reward = +1 if (deviation) < 0.01  else -(deviation)
        if deviation > 0.05 or rot_change:
            setPose(data)
            if deviation >0.05:
                reward = -10
            else:
                reward = -0.01

    if abs(rotationDiff[2]) > 0.01:
        setPose(data)

    print('reward ',reward,' deviation ',deviation, 'rotationDiff', round(abs(rotationDiff[0]),4))
    new_pose = True



def velCallback(velTwist):
    global new_pose
    global cmd_data
    print('velCallback',new_pose,velTwist.linear.x,velTwist.linear.y,rospy.get_rostime().secs,rospy.get_rostime().nsecs)
    cmd_data = ['velTwist',velTwist.linear.x,velTwist.angular.z, rospy.get_rostime()]
    if Learning and new_pose:
        
        print(cmd_data)

        if velTwist.linear.x>0.01:
            # need linear.x and angular.z
            qLearn(velTwist)
            new_pose = False
        elif velTwist.angular.z>0.01:
            #new_pose = False
            velocity_publisher_robot.publish(velTwist)
            publishVel(0)
    '''else:
        #if velTwist.linear.x<0.03:
            qAction = np.argmax(q_table[int(velTwist.linear.x/resolution)])*resolution
            publishVel(qAction)
            #velocity_publisher.publish(velTwist)
            velocity_publisher_robot.publish(velTwist)'''

def updateReward(new_state, current_state, action, reward):
    global episodeReward
    current_idx = int(round(current_state,2)/resolution)
    new_idx = int(round(new_state,2)/resolution)
    
    #### changes for Q-Learning
    # Maximum possible Q value in next step (for new state)
    max_future_q = np.max(q_table[current_idx])

    # Current Q value (for current state and performed action)
    current_q = q_table[current_idx ,new_idx]



    # And here's our equation for a new Q value for current state and action
    new_q = (1 - LEARNING_RATE) * current_q + LEARNING_RATE * (reward + DISCOUNT * max_future_q)

    # Update Q table with new Q value
    q_table[current_idx, new_idx] = new_q
    

    ''' predict = q_table[current_idx ,int(round(action,2))]
        target = reward + DISCOUNT * q_table[new_idx, int(round(get_action(new_state),2)/resolution)]
        q_table[current_idx, new_idx] = q_table[current_idx, new_idx] + LEARNING_RATE * (target - predict)
    '''
    print('q updateReward',q_table[current_idx, new_idx],'new_state', new_state,'current_state',current_state, 
            'action',action, 'reward', reward, 'current_idx', current_idx, current_state, 'new_idx ',new_idx, round(new_state,2),rospy.get_rostime())

    episodeReward += reward


def get_action(state):
        action = 0
        if np.random.uniform(0,1) < epsilon:
            # Get random action
            action = random.choice(actionSpace)
            print('act Random',action)
            #print(action)
        else:
            # Get action from Q table
            row = q_table[int(state/resolution)]
            action = random.choice(np.argwhere(row == row.max()))[0]*resolution # resolution
            print('act',action)
        return action


def qLearn(data):
    global new_state
    global current_state
    global action
    global reward
    global rot_change
    new_state = round(data.linear.x,3)
    #print('qLearn ',new_state,data.linear.x)

    if data.linear.x >= 0.01:
        updateReward(new_state,current_state, action, reward)
        current_state = new_state
        action = get_action(current_state)
        rot_change = False
    else:
        action = 0
        rot_change = True
    publishVel(action)
    velocity_publisher_robot.publish(data)



def publishVel(speedX):
    global cmd_data
    vel_msg = Twist()
    vel_msg.linear.x = speedX
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = cmd_data[2]
    velocity_publisher.publish(vel_msg)
    print('publish velocity',speedX,cmd_data[2],cmd_data[1])




def createEpsilonGreedyPolicy(Q, epsilon, num_actions):
    """
    Creates an epsilon-greedy policy based
    on a given Q-function and epsilon.
       
    Returns a function that takes the state
    as an input and returns the probabilities
    for each action in the form of a numpy array 
    of length of the action space(set of possible actions).
    """
    def policyFunction(state):
        Action_probabilities = np.ones(num_actions,
                dtype = float) * epsilon / num_actions
                  
        best_action = np.argmax(Q[state])
        Action_probabilities[best_action] += (1.0 - epsilon)
        return Action_probabilities
   
    return policyFunction

def saveData():
    ''' save model on shutdown signal'''
    if Learning:
        filename = 'qTable_test_12_09.txt'
        print('saveData',filename)
        with open(filename, 'w') as dataFile:
            dataFile.write(np.array2string(q_table,precision = 2, separator=',' ))
            print("Write successful")
        with h5py.File(savePath, 'w') as f:
            f.create_dataset('qTable', data=q_table)


if __name__ == '__main__':
    '''    
rr = range(2, 20, 3)
    print('deviation')
    rospy.wait_for_service('/gazebo/set_model_state')
    sim_data =[]
    real_data =[]

    # global values for q learning
    reward = -1
    current_state = 0.01
    action = 0.01
    new_state = 0.01


    discount_factor = 1.0
    alpha = 0.6 
    epsilon = 0.1


    cmd_data = Twist()
    rot_change = False
    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    model = GetModelStateRequest()
    model.model_name='robotino3'
    # Create an epsilon greedy policy function
    # appropriately for environment action space
    #policy = createEpsilonGreedyPolicy(Q, epsilon, 20)
    new_pose = True

    actionSpace = np.arange (0.01, 0.21, 0.01)
'''

    rospy.init_node('learning', anonymous=True)
    #rospy.Subscriber('/odom', Odometry, simCallback, queue_size=10)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, realCallback, queue_size=10)
    rospy.Subscriber('/cmd_vel', Twist, velCallback, queue_size=10)

    velocity_publisher = rospy.Publisher('cmd_vel_RL', Twist, queue_size=10)
    velocity_publisher_robot = rospy.Publisher('my_cmd_vel', Twist, queue_size=10)

    rospy.on_shutdown(saveData)
    rospy.spin()


