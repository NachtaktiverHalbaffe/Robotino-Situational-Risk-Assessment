#!/usr/bin/env python3


from typing import DefaultDict
import rospy
from std_msgs.msg import String
from robotino_msgs.msg import DigitalReadings

def talker():
    pub = rospy.Publisher('set_digital_values', DigitalReadings, queue_size=10)
    rospy.init_node('talker', anonymous=True)


    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        dd = DigitalReadings()
        dd.stamp = rospy.get_rostime()
        dd.values = [True, True, True, True, True, True, True, True]
        # sets BG3 value to ON , OUT1 value 

        '''
        OUT0     
        OUT1  =   position[0]
        OUT2
        OUT3
        OUT4
        OUT5
        OUT6
        OUT7 
        
        '''
        #dd.values = [True, False, True, True, True, False, False, False]
        dd.values = [False, False, False, False, False, False, False, False]
        #dd.values = [True]
        print(dd)
        pub.publish(dd)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



