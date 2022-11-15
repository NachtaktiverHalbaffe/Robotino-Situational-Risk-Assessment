#!/usr/bin/env python3


## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

from numpy.lib.polynomial import poly1d
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import *
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

def realCallback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.pose.pose)
    #target.write(str(data.pose.pose))
    #target.flush()
    global real_odom
    real_odom = data
    #print('real_pose',real_odom)
    #print (real_odom)
    global diff_flag
    diff_flag = False
    deviation()

def simCallback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'u heard %s', data.pose.pose)
    #sim_target.write(str(data.pose.pose))
    #sim_target.flush()
    global sim_odom

    # check
    sim_odom = data
    #print(sim_odom)

    deviation()

def deviation():
    global real_odom

    global diff_flag
    
    ''' this is to get quaternion difference  
    q1_inv = [real_odom.pose.pose.orientation.x,real_odom.pose.pose.orientation.y,real_odom.pose.pose.orientation.z,-real_odom.pose.pose.orientation.w]
    q1 = [real_odom.pose.pose.orientation.x,real_odom.pose.pose.orientation.y,real_odom.pose.pose.orientation.z,real_odom.pose.pose.orientation.w]
    q2 = [sim_odom.pose.pose.orientation.x ,sim_odom.pose.pose.orientation.y,sim_odom.pose.pose.orientation.z, sim_odom.pose.pose.orientation.w]

    qr = quaternion_multiply(q2, q1_inv)
    print(qr[2])
    '''

    # get the difference in x, y and yaw values
    q1 = [real_odom.pose.pose.orientation.x,real_odom.pose.pose.orientation.y,real_odom.pose.pose.orientation.z,real_odom.pose.pose.orientation.w]
    q2 = [sim_odom.pose.pose.orientation.x ,sim_odom.pose.pose.orientation.y,sim_odom.pose.pose.orientation.z, sim_odom.pose.pose.orientation.w]
    real_position = [real_odom.pose.pose.position.x, real_odom.pose.pose.position.y]
    sim_position = [sim_odom.pose.pose.position.x -1.2400000000000002, sim_odom.pose.pose.position.y -4.050000000000001]
    
    positionDiff = [a-b for a,b in zip(real_position,sim_position)]

    euler_real = euler_from_quaternion(q1)
    euler_sim = euler_from_quaternion(q2)
    
    rotationDiff = [a-b for a,b  in zip(euler_real,euler_sim)]
    difference_tuple = (positionDiff[0],positionDiff[1],rotationDiff[2])

    #if (round(positionDiff[0],2) != 0):
    print(positionDiff[0])
    if (round(positionDiff[0],2) > 0.3):
        print(round(positionDiff[0],2),round(positionDiff[1],2),round(rotationDiff[2],2))
        setPose(real_odom)
        diff_flag = True
    sim_position  = []
    real_position = []
    
    '''
    global processed_time_sec
    global processed_time_nsec
    now = rospy.get_rostime()
    #rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
    if (processed_time_sec - now.secs<0):# and processed_time_nsec <= now.nsecs):
        #print(processed_time_sec,now.secs,processed_time_nsec, now.nsecs,round(real_odom.header.stamp.nsecs-sim_odom.header.stamp.nsecs,6))
        #if(abs(round(real_odom.header.stamp.nsecs-sim_odom.header.stamp.nsecs,6)) >0  or processed_time_sec - now.secs < 0):
        print('x:',round(positionDiff[0],4),'y:',round(positionDiff[1],4),'yaw:',round(rotationDiff[2],4),real_odom.header.stamp.nsecs,sim_odom.header.stamp.nsecs)
        processed_time_sec = now.secs
        processed_time_nsec = now.nsecs
    '''
    #print('x:',round(positionDiff[0],4),'y:',round(positionDiff[1],4),'yaw:',round(rotationDiff[2],4),real_odom.header.stamp.nsecs,sim_odom.header.stamp.nsecs)
    #pub.publish(difference_tuple)
    '''need linear x and angular z to publish the twist command 
    # feedback input is diff in (x, y, yaw) 
    # current control command (linear.x, angular.z)  
    # output control command (linear.x, angular.z) -> cmd_vel for simulation 
    # 
geometry_msgs/Vector3 linear
  float64 x
  float64 y = 0
  float64 z = 0
geometry_msgs/Vector3 angular
  float64 x = 0
  float64 y = 0
  float64 z
    # 
    '''
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    #rospy.Subscriber('/odom', Odometry, realCallback, queue_size=10)
    rospy.Subscriber('/sim_odom', Odometry, simCallback, queue_size=10)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, realCallback, queue_size=10)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



def setPose(data):
    #print(data)
    
    state_msg = ModelState()
    state_msg.model_name = 'robotino3'
    # check mapp position is needed 
    state_msg.pose.position.x = data.pose.pose.position.x - map_positon_x
    state_msg.pose.position.y = data.pose.pose.position.y - map_positon_y
    state_msg.pose.position.z = 0
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = data.pose.pose.orientation.z
    state_msg.pose.orientation.w = data.pose.pose.orientation.w
    state_msg.reference_frame = 'world'
    #print(state_msg.pose.position.x)

    # wait for /gazebo/set_model_state service to set the robotino position in simulation
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )
        print (resp.status_message,map_positon_y)
    
    except rospy.ServiceException as e:
        print("/gazebo/set_model_state Service call failed: %s"%e)



if __name__ == '__main__':
    print('deviation')
    #target = open("real_file.txt","a")
    #sim_target = open("sim_file.txt","a")
    processed_time_sec = 0
    processed_time_nsec = 0 
    pub = rospy.Publisher('deviation_data', String, queue_size=10)
    rospy.init_node('deviation', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    
    real_odom = Odometry()
    sim_odom = Odometry()
    map_pose = rospy.get_param('origin',[-1.24,-4.05,0])

    rospy.wait_for_service('/gazebo/set_model_state')

    map_positon_x = map_pose[0]
    map_positon_y = map_pose[1]
    diff_flag = False

    listener()





    #talker()
