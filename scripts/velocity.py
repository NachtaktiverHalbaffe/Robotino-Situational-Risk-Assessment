#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose

msg_turn = Twist()

msg_turn.linear.x = 0
msg_turn.linear.y = 0
msg_turn.linear.z = 0
msg_turn.angular.x = 0
msg_turn.angular.y = 0
msg_turn.angular.z = 0

Pose_data = [0, 0, 0]

# ---------------------------------------------------Pose Callback----------------------------------------------
def Pose_Callback(data):
    global Pose_data
    Pose_data = [
        data.pose.pose.position.x,
        data.pose.pose.position.y,
        data.pose.pose.orientation.z,
    ]


def stop_robot():
    msg_stop = Twist()

    msg_stop.linear.x = 0
    msg_stop.linear.y = 0
    msg_stop.linear.z = 0
    msg_stop.angular.x = 0
    msg_stop.angular.y = 0
    msg_stop.angular.z = 0

    return msg_stop


def drive_forward(vel=0.1):
    msg_drive_forward = Twist()

    msg_drive_forward.linear.x = 1 * vel
    msg_drive_forward.linear.y = 0
    msg_drive_forward.linear.z = 0
    msg_drive_forward.angular.x = 0
    msg_drive_forward.angular.y = 0
    msg_drive_forward.angular.z = 0

    return msg_drive_forward


def drive_backward(vel=0.1):
    msg_drive_backward = Twist()

    msg_drive_backward.linear.x = -1 * vel
    msg_drive_backward.linear.y = 0
    msg_drive_backward.linear.z = 0
    msg_drive_backward.angular.x = 0
    msg_drive_backward.angular.y = 0
    msg_drive_backward.angular.z = 0

    return msg_drive_backward


def rotate(vel=0, anticlockwise=True):
    msg_drive_rotate = Twist()

    if anticlockwise == True:
        msg_drive_rotate.linear.x = 0
        msg_drive_rotate.linear.y = 0
        msg_drive_rotate.linear.z = 0
        msg_drive_rotate.angular.x = 0
        msg_drive_rotate.angular.y = 0
        msg_drive_rotate.angular.z = 1 * vel

    else:
        msg_drive_rotate.linear.x = 0
        msg_drive_rotate.linear.y = 0
        msg_drive_rotate.linear.z = 0
        msg_drive_rotate.angular.x = 0
        msg_drive_rotate.angular.y = 0
        msg_drive_rotate.angular.z = -1 * vel

    return msg_drive_rotate


def move(path=[], direction=[], orientation=[], vel=0.15):
    move_publisher = rospy.Publisher("cmd_vel_real", Twist, queue_size=10)

    rate = rospy.Rate(0.5)
    rate.sleep()
    print("Initializations")
    t0 = rospy.Time.now().to_sec()

    for t, d, o in zip(path, direction, orientation):
        print(t, d, o)
        if d == "f":
            while (rospy.Time.now().to_sec() - t0) < t:
                move_publisher.publish(drive_forward(vel=vel))
            t0 = rospy.Time.now().to_sec()
        elif d == "r":
            while (rospy.Time.now().to_sec() - t0) < t:
                move_publisher.publish(rotate(vel=vel, anticlockwise=o))
        t0 = rospy.Time.now().to_sec()
