#!/usr/bin/env python3

import dynamic_reconfigure.client
import rospy

def callback(value):


    exposure_roi_client = dynamic_reconfigure.client.Client("move_base/global_costmap/inflation_layer",timeout=1)

    exposure_roi_params = { 'inflation_radius': value}
    exposure_roi_client.update_configuration(exposure_roi_params)
    #/move_base/global_costmap/inflation_layer/set_parameters


if __name__ == '__main__':
    print('inflation')
    #target = open("real_file.txt","a")
    #sim_target = open("sim_file.txt","a")
    rospy.Subscriber('move_base/global_costmap/inflation_layer/inflation_value', int, callback, queue_size=10)
    rospy.init_node('rs_config_modifier', anonymous=True)
    rospy.spin()
