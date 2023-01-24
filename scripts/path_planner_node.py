#!/usr/bin/env python3
import time
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
from PIL import Image

from autonomous_operation.PRM import apply_PRM_init, Node
from autonomous_operation.object_detection_modified import (
    Obstacle,
    apply_object_detection,
)
from prototype.msg import ObstacleList
from utils.constants import Topics, Nodes
from utils.ros_logger import set_rospy_log_lvl

publisherGlobal = rospy.Publisher(Topics.GLOBAL_PATH.value, Path, queue_size=10)
publisherGlobal = rospy.Publisher(Topics.LOCAL_PATH.value, Path, queue_size=10)


def setCurrentPose(currentPose: PoseWithCovarianceStamped):
    global currentPoint
    currentPoint = currentPose


def setObstacles(obstaclesList: ObstacleList):
    global obstacles
    tmpObstacle = []
    for obstacle in obstaclesList:
        # Create corners
        tmpCorners = []
        for corner in obstacle.corners:
            tmpCorners.append((corner.position.x, corner.position.y))
        # Create Obstacle
        tmpObstacle.append(Obstacle(tmpCorners))
    # TODO construct obstacles
    obstacles = tmpObstacle


def runPRM(targetMessage: PoseStamped, pubTopic: str = Topics.GLOBAL_PATH.value):
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
    global currentPoint
    global obstacles
    # Get target from geometry message
    xTarget = targetMessage.pose.position.x
    yTarget = targetMessage.pose.position.y
    # Current position of robotino
    xCurrent = currentPoint.pose.pose.position.x
    yCurrent = currentPoint.pose.pose.position.y
    print(rospy.get_param("~map_ref"))
    rospy.logdebug(f"[Path Planner] Starting PRM with target ({xTarget},{yTarget})")

    map_ref = apply_object_detection(rospy.get_param("~map_ref"))
    # TODO choose right PRM func
    traj, _, _, _ = apply_PRM_init(
        map_ref=map_ref,
        # obstacles=obstacles,
        obstacles=[],
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
    publisher = rospy.Publisher(pubTopic, Path, queue_size=10)
    try:
        rospy.logdebug(f"[Path Planner] Publishing path to {pubTopic}: {path}")
        publisher.publish(path)
    except:
        print(f"[Path Planner] Error, cant publish path to topic {pubTopic}")


def planner():
    """
    Runs the node itself and subscribes to all necessary topics. This is basically a adapter for the PRM\
    node and holds some utils stuff for path planning
    """
    rospy.init_node(Nodes.PATH_PLANNER.value)
    set_rospy_log_lvl(rospy.DEBUG)
    rospy.loginfo(f"Starting node {Nodes.PATH_PLANNER.value}")
    # Starts the global PRM
    globalSub = rospy.Subscriber(
        Topics.TARGET.value,
        PoseStamped,
        runPRM,
        queue_size=10,
    )
    # Starts the local PRM
    rospy.Subscriber(
        Topics.LOCAL_TARGET.value,
        PoseStamped,
        runPRM,
        callback_args=[Topics.LOCAL_PATH.value],
        queue_size=10,
    )
    # Sets the currentPoint_acml global variable
    locSub = rospy.Subscriber(
        Topics.LOCALIZATION.value,
        PoseWithCovarianceStamped,
        setCurrentPose,
        queue_size=10,
    )
    # Sets the obstacles global variable
    rospy.Subscriber(Topics.OBSTACLES.value, ObstacleList, setObstacles, queue_size=10)

    # Dirty fix to make sure that topics are subscribed and has active connection to corresponding node
    rate = rospy.Rate(1)
    while locSub.get_num_connections() == 0:
        locSub.unregister()
        locSub = rospy.Subscriber(
            Topics.LOCALIZATION.value,
            PoseWithCovarianceStamped,
            setCurrentPose,
            queue_size=10,
        )
        globalSub.unregister()
        globalSub = rospy.Subscriber(
            Topics.TARGET.value,
            PoseStamped,
            runPRM,
            queue_size=10,
        )
        if rospy.is_shutdown():
            return
        rate.sleep()

    # Prevents python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    try:
        planner()
    except:
        rospy.loginfo(f"Shutdown node {Nodes.PATH_PLANNER.value}")
