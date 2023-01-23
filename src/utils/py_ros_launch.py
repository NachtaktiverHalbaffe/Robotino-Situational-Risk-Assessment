import roslaunch
import rospy
import os, sys
from threading import *


def ros_launch_exec(path: str):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [path])
    launch.start()
    rospy.loginfo("Started ROS")

    launch.spin()
    launch.shutdown()
    return launch


def ros_launch_without_core(path: str):
    uuid = roslaunch.rlutil.get_or_generate_uuid(
        options_runid=None, options_wait_for_master=False
    )
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(
        uuid, roslaunch_files=[path], is_core=True
    )
    launch.start()
    rospy.loginfo("Started ROS")
    launch.spin()

    launch.shutdown()


if __name__ == "__main__":
    path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../..", ""))
    print(path)
    PATH_LAUNCH = f"{path}/launch/prototype.launch"
    try:
        ros_launch_exec(PATH_LAUNCH)
    except rospy.ROSInterruptException:
        pass
