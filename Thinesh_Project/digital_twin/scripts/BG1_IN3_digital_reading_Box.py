#!/usr/bin/env python3


import rospy
from robotino_msgs.msg import DigitalReadings

def callback(value):

    BG1_value = value[2]   # true means box is loaded left belt
    #/move_base/global_costmap/inflation_layer/set_parameters


if __name__ == '__main__':
    print('inflation')
    #target = open("real_file.txt","a")
    #sim_target = open("sim_file.txt","a")
    rospy.Subscriber('/digital_readings', DigitalReadings, callback, queue_size=10)

    rospy.init_node('rs_config_modifier', anonymous=True)
    rospy.spin()
