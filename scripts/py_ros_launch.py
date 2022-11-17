import roslaunch
import rospy
from threading import *

launch_files_v = "/home/abdul/AbdulRehman_ws/src/IDT/real_nav_control/launch"
gmapping_launch = launch_files_v + "/" + "gmapping.launch"
generate_map_launch = launch_files_v + "/" + "generate_map.launch"
def ros_launch_exec():
    rospy.init_node('G_Mapping', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [gmapping_launch])
    launch.start()
    rospy.loginfo("started")

    #rospy.sleep(3)
    launch.spin()
    # 3 seconds later
    launch.shutdown()
    return launch

def ros_launch_without_core():
    launch_files_v = "/home/abdul/AbdulRehman_ws/src/IDT/real_nav_control/launch"
    generate_map_launch = launch_files_v + "/" + "generate_map.launch"
    uuid = roslaunch.rlutil.get_or_generate_uuid(options_runid=None, options_wait_for_master=False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_files=[generate_map_launch], is_core=True)
    launch.start()
    rospy.loginfo("started")
    launch.spin()
    
    launch.shutdown()

if __name__ == '__main__':
    try:
        #launch = ros_launch_without_core()
        t1=Thread(target=ros_launch_without_core, daemon=True)
        t1.start()
    except rospy.ROSInterruptException:
        pass