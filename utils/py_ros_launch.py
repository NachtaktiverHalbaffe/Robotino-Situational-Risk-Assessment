import roslaunch
import rospy
from threading import *

launch_files_v = "/home/abdul/AbdulRehman_ws/src/IDT/real_nav_control/launch"
PATH_GMAPPING_LAUNCH = launch_files_v + "/" + "gmapping.launch"
PATH_GENERATE_MAP_LAUNCH = launch_files_v + "/" + "generate_map.launch"


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
    try:
        # launch = ros_launch_without_core()
        t1 = Thread(target=ros_launch_without_core, daemon=True)
        t1.start()
    except rospy.ROSInterruptException:
        pass
