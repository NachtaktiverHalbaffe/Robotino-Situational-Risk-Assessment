#!/usr/bin/env python3
from real_robot_navigation.move_utils import modify_map
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path

from autonomous_operation.PRM import Edge, Node, apply_PRM, apply_PRM_init, get_traj_edges
from autonomous_operation.object_detection import Obstacle
from prototype.msg import ObstacleList
from utils.constants import Topics, Nodes
from utils.create_map_ref import createMapRef
from utils.ros_logger import set_rospy_log_lvl
from real_robot_navigation.move_utils_cords import (
    get_amcl_from_pixel_location,
    get_base_info,
    get_pixel_location_from_acml,
)

THRESHOLD_EDGE = 6

publisherGlobal = rospy.Publisher(Topics.GLOBAL_PATH.value, Path, queue_size=10)
publisherLocal = rospy.Publisher(Topics.LOCAL_PATH.value, Path, queue_size=10)

global obstacles
obstacles = None
global PRMNodes
PRMNodes = []
global newObstacles
newObstacles = False


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
        tmpObstacle.append(Obstacle(tmpCorners))
    obstacles = tmpObstacle

    newObstacles = True


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
    global PRMNodes
    global newObstacles
    global pathPlannerConfig
    # Get target from geometry message
    xTarget = targetMessage.pose.position.x
    yTarget = targetMessage.pose.position.y
    # Current position of robotino
    xCurrent = currentPoint.pose.pose.position.x
    yCurrent = currentPoint.pose.pose.position.y
    rospy.logdebug(f"[Path Planner] Starting PRM with target ({xTarget},{yTarget})")

    base_info, _ = get_base_info()
    all_obst = pathPlannerConfig[1]
    # Convert target and goal to node coordinates
    start = get_pixel_location_from_acml(*(xCurrent, yCurrent), *base_info)
    start = [int(start[0]), int(start[1])]
    goal = get_pixel_location_from_acml(*(xTarget, yTarget), *base_info)
    goal = [int(goal[0]), int(goal[1])]

    if obstacles != None:
        all_obst += obstacles

    map_ref = modify_map(pathPlannerConfig[0], [], all_obst, convert_back_to_grey=True)

    # TODO modify PRM so it find the nearest node as start and goal
    # if len(PRMNodes) != 0 and not newObstacles:
    if False:
        traj, _, PRMNodes, _ = apply_PRM(
            map_ref=pathPlannerConfig[0],
            obstacles=all_obst,
            nodes=PRMNodes,
            start=start,
            goal=goal,
        )
    else:
        traj, _, PRMNodes, _ = apply_PRM_init(
            map_ref=map_ref,
            obstacles=all_obst,
            start=start,
            goal=goal,
        )
        newObstacles = False
    edges = get_traj_edges(traj)

    rospy.logdebug(f"[Path Planner] PRM finished running")
    # Construct path message
    path = Path()
    path.header.frame_id = "map"
    i = -1
    for node in traj:
        if i == -1:
            # Convert node back into amcl form
            node = get_amcl_from_pixel_location(node.x, node.y, *base_info)
            # Create a pose => Poses are the Nodes-equivalent in ROS's path
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = node[0]
            pose.pose.position.y = node[1]
            pose.pose.position.z = 0
            # Append pose to path
            path.poses.append(pose)
        elif float(edges[i].length) > THRESHOLD_EDGE:
            # Convert node back into amcl form
            node = get_amcl_from_pixel_location(node.x, node.y, *base_info)
            # Create a pose => Poses are the Nodes-equivalent in ROS's path
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = node[0]
            pose.pose.position.y = node[1]
            pose.pose.position.z = 0
            # Append pose to path
            path.poses.append(pose)
        else:
            rospy.logdebug(f"[Path Planner] Skipped edge with length {edges[i]}")

        i += 1

    # Publish path
    publisher = rospy.Publisher(pubTopic, Path, queue_size=10)
    try:
        rospy.logdebug(f"[Path Planner] Publishing path to {pubTopic}")
        publisher.publish(path)
    except:
        print(f"[Path Planner] Error, cant publish path to topic {pubTopic}")


def planner():
    """
    Runs the node itself and subscribes to all necessary topics. This is basically a adapter for the PRM\
    node and holds some utils stuff for path planning
    """
    global pathPlannerConfig
    rospy.init_node(Nodes.PATH_PLANNER.value)
    set_rospy_log_lvl(rospy.INFO)
    rospy.loginfo(f"Starting node {Nodes.PATH_PLANNER.value}")
    pathPlannerConfig = createMapRef(rospy.get_param("~map_ref"))
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
