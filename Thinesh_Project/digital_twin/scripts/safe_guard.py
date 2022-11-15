#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(value):
    value

    #/move_base/global_costmap/inflation_layer/set_parameters


if __name__ == '__main__':
    print('inflation')
    #target = open("real_file.txt","a")
    #sim_target = open("sim_file.txt","a")
    pub = rospy.Publisher('/inflation_value', int, queue_size=10)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback, queue_size=10)

    rospy.init_node('rs_config_modifier', anonymous=True)
    rospy.spin()
