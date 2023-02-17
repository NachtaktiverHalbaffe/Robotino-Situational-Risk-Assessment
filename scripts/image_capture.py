#!/usr/bin/env python3
import shutil
from turtle import distance
from unicodedata import name
import numpy as np
import time
from PIL import Image, ImageDraw
import sys
import rospy
import heapq
import os
import matplotlib
import math as m

PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../", ""))

#-----------------------------Camera Data Imports-------------------------------
from cv_bridge import CvBridge, CvBridgeError
import cv2

#-----------------------------Imports for the Messageed
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud 
from sensor_msgs.msg import JointState
from robotino_msgs.msg import MotorReadings
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from datetime import datetime
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

#----------------------------Subjective Logic Imports----------------------------

#Set maximum depth of the Python interpreter stack
sys.setrecursionlimit(4000)


#Global variable to read pose data
Pose_real_data = [0,0,0,0]

#GMCL Pose call back
def Pose_Callback(data):
    global Pose_real_data
    #print(data)
    #if(data.child_frame_id=="body" or True):
    Pose_real_data = ["Pose_real_data",
                data.pose.pose.position.x, 
                data.pose.pose.position.y,
                data.pose.pose.orientation.z,  # Converted from [-1, 1] to [-pi, pi] using angle=arcsin(z)*2
                data.header.stamp]

def Image_Callback(img_data):
    global Image_data
    global model
    
    if True:
        print("Snapshot")
    
        bridge = CvBridge()

        cv_image = bridge.imgmsg_to_cv2(img_data,"rgb8")
    
    #Resize Image to 640 * 480 - anything smaller than that wont work with DenseDepth Model
        width = int(img_data.width * 0.80)
        height = int(img_data.height * 0.80)
        dim = (width, height)
        #img_resized = cv2.resize(cv_image, dim, interpolation = cv2.INTER_AREA)

        Image_data = ['Image_Data',
                    dim, # dimensions of resized image
                    cv_image,  # image data
                    img_data.header.stamp]


        name = 's_'+str(Pose_real_data[1])+'_'+str(Pose_real_data[2])+'_'+str(Pose_real_data[3])+'_e'

        Save_Camera_Input = f'{PATH}/image/yolo/Camera_Time_{Image_data[3]}{name}.png'

        rospy.sleep(1)


        print(cv2.imwrite(Save_Camera_Input, Image_data[2]), 'this should save')



#----------------------------------Main Function-----------------------------------------------

def main():

    
    rospy.init_node('image_capture', anonymous=True)
    #Subscribe to AMCL Pose data
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, Pose_Callback, queue_size=10)
    rospy.Subscriber('/image_raw', Image, Image_Callback, queue_size=10)
    rospy.sleep(1)
    
    #offset_angle_rad = caliberation()
    while not rospy.is_shutdown():
        print("Capturing Data")
        rospy.sleep(1.1)


if __name__ == '__main__':
    main()


