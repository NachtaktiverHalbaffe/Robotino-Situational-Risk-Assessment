#!/usr/bin/env python3

import rospy
import os
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Int16, Bool

from autonomous_operation.PRM import (
    Edge,
    Node,
    apply_PRM,
    apply_PRM_init,
    get_traj_edges,
)
from real_robot_navigation.move_utils import modify_map
from autonomous_operation.object_detection import Obstacle
from prototype.msg import ObstacleList
from utils.constants import Topics, Nodes
from utils.navigation_utils import createMapRef, trajToPath
from utils.ros_logger import set_rospy_log_lvl
from real_robot_navigation.move_utils_cords import (
    get_amcl_from_pixel_location,
    get_base_info,
    get_pixel_location_from_acml,
)

PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../", ""))
THRESHOLD_EDGE = 8

pathPublisher = rospy.Publisher(
    Topics.GLOBAL_PATH.value, Path, queue_size=10, latch=True
)

obstacles = None
PRMNodes = []
newObstacles = False
currentPoint = PoseWithCovarianceStamped()
currentPoint.pose.pose.position.x = -3.2
currentPoint.pose.pose.position.y = 1.2

obstacleMargin = 0
fallbackPlanning = False


def setFallbackPlanning(isInFallbackMode: Bool):
    if isInFallbackMode.data:
        fallbackPlanning = True


def setObstacleMargin(margin: Int16):
    global obstacleMargin
    obstacleMargin = margin.data


def setCurrentPose(currentPose: PoseWithCovarianceStamped):
    global currentPoint
    currentPoint = currentPose


def setObstacles(obstaclesList: ObstacleList):
    global obstacles
    global newObstacles

    tmpObstacle = []
    for obstacle in obstaclesList.obstacles:
        # Create corners
        tmpCorners = []
        for corner in obstacle.corners:
            tmpCorners.append((corner.x, corner.y))
        # Create Obstacle
        tmpObstacle.append(Obstacle(tmpCorners, label=obstacle.label))
    obstacles = tmpObstacle
    if len(obstaclesList.obstacles) != 0:
        newObstacles = True


def runPRM(targetMessage: PoseStamped):
    """
    Runs the PRM to get a trajectory

    Args:
        targetMessage (geometry_msgs.Point): Target point to which the robotino should navigate. Is given by the subscriber who calls this function\
        currenPoint and obstacles are taken from an global variable
        pubTopic (str): Name of the topic to which the path should be published
    Returns:
        Path is published to topic "/path_raw"
    """
    global currentPoint
    global obstacles
    global PRMNodes
    global newObstacles
    global pathPlannerConfig
    global obstacleMargin
    global fallbackPlanning

    # Get target from geometry message
    xTarget = targetMessage.pose.position.x
    yTarget = targetMessage.pose.position.y
    # Current position of robotino
    if not fallbackPlanning:
        xCurrent = currentPoint.pose.pose.position.x
        yCurrent = currentPoint.pose.pose.position.y
    else:
        fallbackPose = rospy.wait_for_message(
            Topics.FALLBACK_POSE.value, PoseWithCovarianceStamped
        )
        xCurrent = fallbackPose.pose.pose.position.x
        yCurrent = fallbackPose.pose.pose.position.y
    rospy.loginfo(f"[Path Planner] Starting PRM with target ({xTarget},{yTarget})")

    # Get configuration
    base_info, _ = get_base_info()
    all_obst = pathPlannerConfig[1]
    # Convert target and goal to node coordinates
    start = get_pixel_location_from_acml(*(xCurrent, yCurrent), *base_info)
    start = [int(start[0]), int(start[1])]
    goal = get_pixel_location_from_acml(*(xTarget, yTarget), *base_info)
    goal = [int(goal[0]), int(goal[1])]

    # Add detected obstacles to map and modify map_ref so obstacles are placed
    if obstacles != None:
        all_obst += obstacles
    map_ref = modify_map(pathPlannerConfig[0], [], all_obst, convert_back_to_grey=True)

    if len(PRMNodes) != 0 and not newObstacles:
        traj, _, PRMNodes, _ = apply_PRM(
            map_ref=map_ref,
            obstacles=all_obst,
            nodes=PRMNodes,
            start=start,
            goal=goal,
            obstacleMargin=obstacleMargin,
        )
    else:
        traj, _, PRMNodes, _ = apply_PRM_init(
            map_ref=map_ref,
            obstacles=all_obst,
            start=start,
            goal=goal,
            obstacleMargin=obstacleMargin,
        )
        newObstacles = False

    # Construct path message
    edges = get_traj_edges(traj)
    trajectory = []
    i = -1
    for node in traj:
        # Convert nodes back from pixelmap-domain into amcl domain
        if (i == -1) or (float(edges[i].length) > THRESHOLD_EDGE):
            # trajectory.append(get_amcl_from_pixel_location(node.x, node.y, *base_info))
            trajectory.append((node.x, node.y))
        else:
            rospy.logdebug(f"[Path Planner] Skipped edge with length {edges[i]}")
        i += 1
    path = trajToPath(trajectory)
    rospy.loginfo(f"[Path Planner] PRM finished running")
    # Publish path
    try:
        rospy.logdebug(f"[Path Planner] Publishing path to {Topics.GLOBAL_PATH.value}")
        pathPublisher.publish(path)
    except:
        rospy.logwarn(
            f"[Path Planner] Error, cant publish path to topic {Topics.GLOBAL_PATH.value}"
        )


def planner():
    """
    Runs the node itself and subscribes to all necessary topics. This is basically a adapter for the PRM\
    node and holds some utils stuff for path planning
    """
    global pathPlannerConfig
    rospy.init_node(Nodes.PATH_PLANNER.value)
    set_rospy_log_lvl(rospy.INFO)
    rospy.loginfo(f"Starting node {Nodes.PATH_PLANNER.value}")

    pathPlannerConfig = createMapRef(
        rospy.get_param("map_path", default=f"{PATH}/maps/FinalGridMapv2cleaned.png")
    )

    # Starts the global PRM
    plannerSub = rospy.Subscriber(
        Topics.TARGET.value,
        PoseStamped,
        runPRM,
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
    rospy.Subscriber(
        Topics.OBSTACLE_MARGIN.value, Int16, setObstacleMargin, queue_size=10
    )

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
        plannerSub.unregister()
        plannerSub = rospy.Subscriber(
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
    except Exception as e:
        print(e)
        rospy.loginfo(f"Shutdown node {Nodes.PATH_PLANNER.value}")
