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

#---------------------------------------------------Pose Callback----------------------------------------------
#GMCL Pose call back
def Pose_Callback(data):
    global Pose_data
    Pose_data = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.orientation.z]
    # print("Data Dump")
    #print("Postion X: ", Pose_data[0])
    # print("Position Y: ", data.pose.pose.position.y)
    # print("Orientation Z: ", data.pose.pose.orientation.z)
    # print("Linear X: ", data.twist.twist.linear.x)
    # print("Linear Y: ", data.twist.twist.linear.y)
    # print("Rotation: ", data.twist.twist.angular.z)
    #global Pose_real_data
    # if(data.child_frame_id=="body"):
    #     Pose_real_data = ["Pose_real_data",
    #                 data.pose.pose.position.x, 
    #                 data.pose.pose.position.y,
    #                 data.pose.pose.orientation.z,  # Converted from [-1, 1] to [-pi, pi] using angle=arcsin(z)*2
    #                 data.header.stamp]
    #     print("Pos X: ", Pose_real_data[1])
    #rate = rospy.Rate(0.5)
    #rate.sleep()
    
""" Stop """
""" Forward """
def stop_robot():
    msg_stop = Twist()
    
    msg_stop.linear.x = 0
    msg_stop.linear.y = 0
    msg_stop.linear.z = 0
    msg_stop.angular.x = 0
    msg_stop.angular.y = 0
    msg_stop.angular.z = 0

    return msg_stop

""" Forward """
def drive_forward(vel = 0.1):
    msg_drive_forward = Twist()
    
    msg_drive_forward.linear.x = 1 * vel
    msg_drive_forward.linear.y = 0
    msg_drive_forward.linear.z = 0
    msg_drive_forward.angular.x = 0
    msg_drive_forward.angular.y = 0
    msg_drive_forward.angular.z = 0

    return msg_drive_forward

""" Backward """
def drive_backward(vel = 0.1):
    msg_drive_backward = Twist()

    msg_drive_backward.linear.x = -1 * vel
    msg_drive_backward.linear.y = 0
    msg_drive_backward.linear.z = 0
    msg_drive_backward.angular.x = 0
    msg_drive_backward.angular.y = 0
    msg_drive_backward.angular.z = 0

    return msg_drive_backward

""" Rotate """
def rotate(vel=0, clockwise = True):
    msg_drive_rotate = Twist()

    if clockwise:
        msg_drive_rotate.linear.x = 0
        msg_drive_rotate.linear.y = 0
        msg_drive_rotate.linear.z = 0
        msg_drive_rotate.angular.x = 0
        msg_drive_rotate.angular.y = 0
        msg_drive_rotate.angular.z = 1 * vel
    
    if clockwise:
        msg_drive_rotate.linear.x = 0
        msg_drive_rotate.linear.y = 0
        msg_drive_rotate.linear.z = 0
        msg_drive_rotate.angular.x = 0
        msg_drive_rotate.angular.y = 0
        msg_drive_rotate.angular.z = -1 * vel

    return msg_drive_rotate

""" Move the robot """
""" Forward, backward, Rotote"""
def move():
    #rospy.init_node('move_publisher_node', anonymous=True) #Creation of node
    move_publisher = rospy.Publisher('cmd_vel_real', Twist, queue_size=10)

    rate = rospy.Rate(0.5)
    t0 = rospy.Time.now().to_sec()
    rate.sleep()
    x0 = Pose_data[0]
    z0 = Pose_data[2]

    while not rospy.is_shutdown():

        # while abs(Pose_data[0] - x0) < 0.5:
        #     print("X0: ", x0, "Pose_data[0]: ", Pose_data[0], "diff: ", abs(Pose_data[0] - x0))
        #     #move_publisher.publish(drive_forward(vel=0.15))
        #     z0 = Pose_data[2]

        # 1.98 = 360 deg, 0.99 = 180 deg
        while abs(Pose_data[2] - z0) < 2:
            print("Z0: ", z0, "Pose_data[2]: ", Pose_data[2], "diff: ", (Pose_data[2] - z0))
            #move_publisher.publish(rotate(vel=0.15))
            x0 = Pose_data[0]
        
        

        # while (rospy.Time.now().to_sec() - t0) < 5.1:
        #     move_publisher.publish(drive_forward(vel=0.15))
        #     #print(Pose_data[0])
        #     t1 = rospy.Time.now().to_sec()
        
        # while (rospy.Time.now().to_sec() - t1) < 5.1:
        #     move_publisher.publish(rotate(vel=0.15))
        #     t0 = rospy.Time.now().to_sec()
        
        # move_publisher.publish(rotate(vel=0.15))
        # rate.sleep()
        # move_publisher.publish(drive_forward(vel=0.15))
        # #move_publisher.publish(drive_forward(vel=0.15))
        # rate.sleep()
        #move_publisher.publish(drive_backward())
        #rate.sleep()
        #move_publisher.publish(rotate(vel=0.15))
        #rate.sleep()

def listener():
    rospy.init_node("Odometry node", anonymous=True)
    #Subscribe to AMCL Pose data
    rospy.Subscriber('/odom', Odometry, Pose_Callback, queue_size=10)
    #print("test 1")
    move()
    rospy.spin()    ##Run the node until we shutdown

if __name__ == '__main__':
    try:
        #move()
        listener()
        #move()
        print("Completed !!!") 
    except rospy.ROSInterruptException:
        pass