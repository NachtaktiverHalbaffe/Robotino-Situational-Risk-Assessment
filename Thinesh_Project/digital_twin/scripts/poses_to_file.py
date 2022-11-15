#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import time


class SavePoses(object):
    def __init__(self):
        
        self._pose = Pose()
        self._pose_sub = rospy.Subscriber('/odom', Odometry , self.sub_callback)
        
        self.write_to_file(self._pose)

    def sub_callback(self, msg):
        
        self._pose = msg.pose.pose
    
    def write_to_file(self, pose_value):
        target = open("file.txt","a")

        with open('test.yaml',"rb") as f:
            data_value = f.read(160)
            while data_value != b"":
                data_value = f.read(160)
                target.write(data_value)
                target.flush()
        
        with open('poses.txt', 'w') as file:
            
            for key, value in self.poses_dict.iteritems():
                if value:
                    file.write(str(key) + ':\n----------\n' + str(value) + '\n===========\n')
                    
        rospy.loginfo("Written all Poses to poses.txt file")
        


if __name__ == "__main__":
    rospy.init_node('pose_recorder', log_level=rospy.INFO) 
    save_spots_object = SavePoses()
    rospy.spin() # mantain the service open.
