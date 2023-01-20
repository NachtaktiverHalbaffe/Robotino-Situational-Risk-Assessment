import roslaunch
import rospy
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
    PATH_GMAPPING_LAUNCH = "launch/gmapping.launch"
    PATH_GENERATE_MAP_LAUNCH = "launch/generate_map.launch"
    try:
        t1 = Thread(target=ros_launch_exec, daemon=True, args=[PATH_GMAPPING_LAUNCH])
        t1.start()
    except rospy.ROSInterruptException:
        pass
