#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path

from PRM import apply_PRM_init, Node
from msg import ObstacleList
from constants import Topics, Nodes


# --------------------------- Global variables ---------------------------------
# Holds the current pose. Type: geometry_msgs.PoseWithCovarianceStamped
global currentPoint_acml
# Holds a list of the current identified obstacles. Type: msgs.ObstacleList
global obstacles

# ------------------------------ Constants --------------------------------------
MAP_REF = "/path/to/map_ref"


def runPRM(targetMessage: Point):
    """
    Runs the PRM to get a trajectory

    Args:
        targetMessage (geometry_msgs.Point): Target point to which the robotino should navigate. Is given by the subscriber who calls this function
        currenPoint and obstacles are taken from an global variable

    Returns:
        Path is published to topic "/path_raw"
    """
    # Get target from geometry message
    xTarget = targetMessage.x
    yTarget = targetMessage.y
    # Current position of robotino
    global currentPoint_acml
    xCurrent = currentPoint_acml.pose.pose.position.x
    yCurrent = currentPoint_acml.pose.pose.position.y
    # Obstacles
    global obstacles

    # TODO choose right PRM func
    traj, _, _, edges = apply_PRM_init(
        map_ref=MAP_REF,
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
    publisher = rospy.Publisher(Path, Topics.PATH.value)
    try:
        publisher.publish(path)
    except:
        print(f"Error, cant publish path to topic {Topics.PATH.value}")


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
    # Starts the PRM
    rospy.Subscriber(Topics.TARGET.value, Point, runPRM)
    # Sets the currentPoint_acml global variable
    rospy.Subscriber(Topics.ACML.value, PoseWithCovarianceStamped, setCurrentPose)
    # Sets the obstacles global variable
    rospy.Subscriber(Topics.OBSTACLES.value, ObstacleList, setObstacles)

    # Prevents python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    planner()
