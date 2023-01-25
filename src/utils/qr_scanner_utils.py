#!/usr/bin/env python3
import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

from .conversions import euler2Quaternions

TARGET_OFFSET_VALUE = 1.0


def laser2mapConv(x, y, z, roll, pitch, yaw):

    transform_listener = tf.TransformListener()
    transform_listener.waitForTransform(
        "/laser_link", "/map", rospy.Time(), rospy.Duration(4.0)
    )

    i = 0

    while i < 1:

        posedstamped = PoseStamped()
        posedstamped.header.frame_id = "laser_link"
        posedstamped.header.stamp = rospy.Time(0)
        posedstamped.pose.position.x = x
        posedstamped.pose.position.y = y
        posedstamped.pose.position.z = z

        quanternion = euler2Quaternions(roll, pitch, yaw)

        posedstamped.pose.orientation.x = quanternion[0]
        posedstamped.pose.orientation.y = quanternion[1]
        posedstamped.pose.orientation.z = quanternion[2]
        posedstamped.pose.orientation.w = quanternion[3]

        try:
            posedstamped_map = transform_listener.transformPose("map", posedstamped)
            i = i + 2

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue

    return posedstamped_map


def getDistAngle(theta):
    navDist = 0.0
    alpha_deg = 0.0
    laserMsg = rospy.wait_for_message("/scan", LaserScan, 2)

    if theta > 0:
        if theta - math.floor(theta) <= 0.2:
            theta = math.floor(theta)
        elif (theta - math.floor(theta) > 0.2) and (theta - math.floor(theta) <= 0.7):
            theta = math.floor(theta) + 0.5
        elif theta - math.floor(theta) > 0.7:
            theta = math.ceil(theta)
    elif theta < 0:
        if math.ceil(theta) - theta <= 0.2:
            theta = math.ceil(theta)
        elif (math.ceil(theta) - theta > 0.2) and (math.ceil(theta) - theta <= 0.7):
            theta = math.ceil(theta) - 0.5
        elif math.ceil(theta) - theta > 0.7:
            theta = math.floor(theta)

    # if ((theta * 2)+240 - math.floor((theta * 2)+240)) <= 0.5:
    #     objectDist = laserMsg.ranges[math.floor((theta * 2)+240)]
    # else:
    #     objectDist = laserMsg.ranges[math.ceil((theta * 2)+240)]
    index = int((theta * 2) + 240)
    objectDist = laserMsg.ranges[index]

    if objectDist > 0:

        if theta < 0:
            theta_rad = math.radians(-1 * theta)

            P = (objectDist * math.sin((math.pi / 2) - theta_rad)) - TARGET_OFFSET_VALUE
            B = objectDist * math.cos((math.pi / 2) - theta_rad)

            navDist = math.sqrt(B**2 + P**2)
            alpha_deg = -1 * (90 - math.degrees(math.asin(P / navDist)))
        else:

            theta_rad = math.radians(theta)

            P = (objectDist * math.sin((math.pi / 2) - theta_rad)) - TARGET_OFFSET_VALUE
            B = objectDist * math.cos((math.pi / 2) - theta_rad)

            navDist = math.sqrt(B**2 + P**2)
            alpha_deg = 90 - math.degrees(math.asin(P / navDist))

    return (objectDist, navDist, alpha_deg)
