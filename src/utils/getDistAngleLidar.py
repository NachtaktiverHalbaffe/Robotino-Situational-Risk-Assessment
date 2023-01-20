#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import math

TARGET_OFFSET_VALUE = 0.6

def getDistAngle(theta):

    print("Fetching Distance .......")
    # rospy.init_node('listener', anonymous=True)
    # objectDist = []
    navDist = 0.
    alpha_deg = 0.
    laserMsg =rospy.wait_for_message('/scan', LaserScan, 2)
    
    # for angle in theta:

    #     objectDist.append(laserMsg.ranges[math.floor((angle * 2)+240)]) 
    print("Actual theta", theta)

    if theta > 0:
        if theta - math.floor(theta) <= 0.2:
            theta = math.floor(theta)
        elif (theta - math.floor(theta) > 0.2)  and (theta - math.floor(theta) <= 0.7):
            theta = math.floor(theta) + 0.5
        elif (theta - math.floor(theta) > 0.7):
            theta = math.ceil(theta)
    elif theta < 0:
        if math.ceil(theta) - theta <= 0.2:
            theta = math.ceil(theta)
        elif (math.ceil(theta) - theta > 0.2)  and (math.ceil(theta) - theta <= 0.7):
            theta = math.ceil(theta) - 0.5
        elif (math.ceil(theta) - theta > 0.7):
            theta = math.floor(theta)


    print("Converted theta", theta)

    # if ((theta * 2)+240 - math.floor((theta * 2)+240)) <= 0.5:
    #     objectDist = laserMsg.ranges[math.floor((theta * 2)+240)]
    # else:
    #     objectDist = laserMsg.ranges[math.ceil((theta * 2)+240)]
    index = int((theta * 2)+240)
    objectDist = laserMsg.ranges[index] 

    

    if objectDist>0:
        
        if theta < 0:
            theta_rad = math.radians(-1 * theta)

            P = (objectDist* math.sin((math.pi/2)-theta_rad))- TARGET_OFFSET_VALUE
            B = (objectDist * math.cos((math.pi/2)-theta_rad))

            navDist = math.sqrt(B**2 + P**2)
            alpha_deg = -1*(90 - math.degrees(math.asin(P/navDist)))
        else:
        
            theta_rad = math.radians(theta)

            P = (objectDist* math.sin((math.pi/2)-theta_rad))-TARGET_OFFSET_VALUE
            B = (objectDist * math.cos((math.pi/2)-theta_rad))

            
            navDist = math.sqrt(B**2 + P**2)
            alpha_deg = 90 - math.degrees(math.asin(P/navDist))

    return (objectDist , navDist, alpha_deg)
