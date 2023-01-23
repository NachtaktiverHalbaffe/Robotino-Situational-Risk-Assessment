#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path

from autonomous_operation.PRM import apply_PRM_init, Node
from prototype.msg import ObstacleList
from utils.constants import Topics, Nodes
from utils.ros_logger import set_rospy_log_lvl


# --------------------------- Global variables ---------------------------------
# Holds the current pose. Type: geometry_msgs.PoseWithCovarianceStamped
global currentPoint_acml
# Holds a list of the current identified obstacles. Type: msgs.ObstacleList
global obstacles

# ------------------------------ Constants --------------------------------------
MAP_REF = "./maps/FinalGridMapv2cleaned.png"


def runPRM(
    targetMessage: PoseStamped, pubTopic=Topics.GLOBAL_PATH.value, map_ref=MAP_REF
):
    """
    Runs the PRM to get a trajectory

    Args:
        targetMessage (geometry_msgs.Point): Target point to which the robotino should navigate. Is given by the subscriber who calls this function\
        currenPoint and obstacles are taken from an global variable
        pubTopic (str): Name of the topic to which the path should be published
        map_ref (str): Path to the reference map
    Returns:
        Path is published to topic "/path_raw"
    """
    # Get target from geometry message
    xTarget = targetMessage.pose.position.x
    yTarget = targetMessage.pose.position.y
    # Current position of robotino
    global currentPoint_acml
    xCurrent = currentPoint_acml.pose.pose.position.x
    yCurrent = currentPoint_acml.pose.pose.position.y
    # Obstacles
    global obstacles

    rospy.logdebug(f"[Path Planner] Starting PRM with target ({xTarget},{yTarget})")
    # TODO choose right PRM func
    traj, _, _, _ = apply_PRM_init(
        map_ref=map_ref,
        obstacles=obstacles,
        start_node=Node(xCurrent, yCurrent),
        goal_node=Node(xTarget, yTarget),
    )

    # Construct path message
    path = Path()
    for node in traj:
        # Create a pose
        pose = PoseStamped()
        pose.pose.position.x = node.x
        pose.pose.position.y = node.y
        pose.pose.position.z = 0
        # Append pose to path
        path.poses.append(pose)

    # Publish path
    publisher = rospy.Publisher(pubTopic, Path)
    try:
        rospy.logdebug(f"Publishing path to {pubTopic}: {path}")
        publisher.publish(path)
    except:
        print(f"Error, cant publish path to topic {pubTopic}")


def setCurrentPose(currentPose: PoseWithCovarianceStamped):
    global currentPoint_acml
    currentPoint_acml = currentPose


def setObstacles(obstaclesList: ObstacleList):
    global obstacles
    obstacles = obstaclesList


def planner():
    """
    Runs the node itself and subscribes to all necessary topics. This is basically a adapter for the PRM\
    node and holds some utils stuff for path planning
    """
    rospy.init_node(Nodes.PATH_PLANNER.value)
    set_rospy_log_lvl(rospy.DEBUG)
    rospy.loginfo(f"Starting node {Nodes.PATH_PLANNER.value}")
    map_ref = rospy.get_param("~map_ref")

    # Starts the global PRM
    rospy.Subscriber(
        Topics.TARGET.value,
        PoseStamped,
        runPRM,
        callback_args=[Topics.GLOBAL_PATH.value, map_ref],
        queue_size=1,
    )
    # Starts the local PRM
    rospy.Subscriber(
        Topics.LOCAL_TARGET.value,
        PoseStamped,
        runPRM,
        callback_args=[Topics.LOCAL_PATH.value, map_ref],
        queue_size=1,
    )
    # Sets the currentPoint_acml global variable
    rospy.Subscriber(Topics.ACML.value, PoseWithCovarianceStamped, setCurrentPose)
    # Sets the obstacles global variable
    rospy.Subscriber(Topics.OBSTACLES.value, ObstacleList, setObstacles)
    # Prevents python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    try:
        planner()
    except:
        rospy.loginfo(f"Shutdown node {Nodes.PATH_PLANNER.value}")
