import os, sys
from time import sleep
from gridmap import get_obstacles_in_pixel_map

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)) + "/yolov7")
from detect_online import get_conf_and_model, loaded_detect
import matplotlib

from move_utils import *
from move_utils_cords import *


def realCallback(data):

    global real_data
    real_data = [
        "real_data",
        data.pose.pose.position.x,
        data.pose.pose.position.y,
        data.pose.pose.orientation.z,  # this will be a value e[-1, 1] and can be converted [-pi, pi] with angle=arcsin(z)*2
        data.header.stamp,
    ]


if __name__ == "__main__":
    global real_data
    rospy.init_node("test_loc", anonymous=True)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, realCallback, queue_size=10)
    rospy.sleep(1)
    rospy.sleep(3)

    while not rospy.is_shutdown():
        local_acml_location = deepcopy(real_data)
        local_acml_location = offset_to_robo(local_acml_location)
        # print(local_acml_location)
