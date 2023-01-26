#!/usr/bin/env python3
import time
import rospy
import actionlib
import message_filters
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from std_msgs.msg import Bool
from nav_msgs.msg import Path

from utils.constants import Topics, Nodes
from utils.ros_logger import set_rospy_log_lvl

global feedbackValue
feedbackValue = []


def moveBaseCallback(data):
    global feedbackValue

    # print(data)
    feedbackValue = []
    feedbackValue.append(data.base_position.pose.position.x)
    feedbackValue.append(data.base_position.pose.position.y)
    feedbackValue.append(data.base_position.pose.position.z)


def moveBaseClient(path: Path):
    """
    Executes a global strategy. It's basically driving a trajectory and using a local planner if a hazard\
    is detected which wasn't mitigated by the risk evaluation.

    Args:
        path (nav_msgs.Path): The path to drive
    """
    global feedbackValue

    rospy.logdebug("[Strategy Planner] Starting navigation with move_base client")
    for node in path.poses[1:]:
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # TODO Create first message
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        # feedback = MoveBaseFeedback()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = node.pose.position.x
        goal.target_pose.pose.position.y = node.pose.position.y
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation.w = 1.0
        # Move 0.5 meters forward along the x axis of the "map" coordinate frame
        # No rotation of the mobile base frame w.r.t. map frame
        #  yaw

        client.wait_for_server()
        # Sends the goal to the action server.
        client.send_goal(goal, feedback_cb=moveBaseCallback)

        while not rospy.is_shutdown():
            if feedbackValue:
                if (abs(feedbackValue[0] - goal.target_pose.pose.position.x) < 0.15) and (
                    abs(feedbackValue[1] - goal.target_pose.pose.position.y) < 0.15
                ):
                    client.cancel_goal()
                    break

        result = client.get_result()


def execute(path: Path, isLocalPlanner=False):
    """
    Executes a global strategy. It's basically driving a trajectory and using a local planner if a hazard\
    is detected which wasn't mitigated by the risk evaluation.

    Args:
        path (nav_msgs.Path): The path to drive
        isLocalPlanner (bool, optional): If it executes a local (True) or global (False) path. Defaults to False
    """
    global currentTarget
    global isInLocalMode
    nodes = path.poses
    publisher = rospy.Publisher(Topics.NAV_POINT.value, Point)

    for node in nodes:
        targetPoint = Point()
        targetPoint.x = node.pose.position.x
        targetPoint.y = node.pose.position.y
        currentTarget = targetPoint
        try:
            publisher.publish(targetPoint)
            response = rospy.wait_for_message(Topics.NAVIGATION_RESPONSE.value, Bool)
            if response:
                # Navigated successfully to node, continue with next node
                continue
            elif isInLocalMode and not isLocalPlanner:
                # Current navigation was aborted to drive around a obstacle
                # => wait until target is reached to seek global path again
                response = rospy.wait_for_message(Topics.NAVIGATION_RESPONSE.value, Bool)
                if response:
                    # Local navigation finished => Continue global navigation
                    publisher.publish(targetPoint)
                    response = rospy.wait_for_message(Topics.NAVIGATION_RESPONSE.value, Bool)
                    if response:
                        # Navigated successfully to node, continue with next node
                        continue
                else:
                    # Local navigation also failed => Abort task
                    break
            else:
                # Navigation failed, aborting task
                return

        except:
            return


def detectedHazard():
    """ 
    Identifies the hazard and plans the action to avoid them (usually detecting obstacles over infrared sensors)\
    => Is basically a part of a local planner
    """
    global currentTarget
    global isInLocalMode
    isInLocalMode = False
    # Distance threshold under which a hazard is detected
    THRESHOLD_DISTANCE = 8
    # Time which a hazard must be present to be detected as a passive obstacle
    WAIT_TIME = 4

    while not rospy.is_shutdown():
        try:
            rawReadings = rospy.wait_for_message(Topics.IR_SENSORS.value, PointCloud)
            dist = _fetchIRReadings(rawReadings)

            if np.min(dist) < THRESHOLD_DISTANCE:
                # Detected hazard ==> stop until problem is resolved
                rospy.loginfo("[Strategy Planner] IR sensors detected hazard. Emergency breaking now")
                rospy.Publisher(Topics.EMERGENCY_BRAKE.value, Bool).publish(True)

                # Wait if hazard resolves itself
                time.sleep(WAIT_TIME)
                rawReadings = rospy.wait_for_message(Topics.IR_SENSORS.value, PointCloud)
                dist = _fetchIRReadings(rawReadings)

                if np.min(dist) > THRESHOLD_DISTANCE:
                    # Hazard isn't detected anymore => Is most likely a active obstacle which
                    # isn't here anymore e.g. another Robotino
                    # ===> Slow down as avoidance measure
                    rospy.Publisher(Topics.EMERGENCY_BRAKE.value, Bool).publish(False)
                    continue
                else:
                    # Hazard still detected => Is most likely a passive obstacle
                    # ===> Drive around (start PRM with next target Node as target)
                    # TODO Add Obstacle to map
                    isInLocalMode = True
                    rospy.Publisher(Topics.LOCAL_TARGET.value, Point).publish(currentTarget)
        except:
            return


def chooseStrategy(path: Path):
    """ """


def strategyPlanner(runWithRiskEstimation=True):
    """
    Runs the node itself and subscribes to all necessary topics.

    Args:
        runWithRiskEstimation (bool, optional): If it should run after risk estimation (True) or after a\
                                                                            trajectory is planned (False). Defaults to True 
    """
    rospy.init_node(Nodes.STRATEGY_PLANNER.value)
    set_rospy_log_lvl(rospy.INFO)
    rospy.loginfo(f"Starting node {Nodes.STRATEGY_PLANNER.value}")
    if not runWithRiskEstimation:
        # Just drive the trajectory
        # TODO Change to move_base
        rospy.Subscriber(Topics.GLOBAL_PATH.value, Path, moveBaseClient, queue_size=1)
        # rospy.Subscriber(Topics.GLOBAL_PATH.value, Path, execute)
        # rospy.Subscriber(Topics.LOCAL_PATH.value, Path, execute, callback_args=[True])
    else:
        # Start driving when the risk values come in
        pathSub = message_filters.Subscriber(Topics.GLOBAL_PATH.value, Path)

        t1 = message_filters.ApproximateTimeSynchronizer([pathSub], queue_size=10, slop=0.5)
        t1.registerCallback(chooseStrategy)

    # detectedHazard()
    rospy.spin()


def _fetchIRReadings(rawReadings: PointCloud):
    """
    Converts the raw readings from the IR sensors to distances and appends them all to a list

    Args:
        rawReadings (sensor_msgs.PointCloud): The sensor readings

    Returns:
        dist: List of the distances where each item is the distance from a sensor \
                => Has fixed length of 8
    """
    dist = []
    # Sensor 0 forward
    dist.append(np.sqrt(rawReadings.points[0].x ** 2 + rawReadings.points[0].y ** 2))
    # Sensor 1 40° rotated anti-clockwise
    dist.append(np.sqrt(rawReadings.points[1].x ** 2 + rawReadings.points[1].y ** 2))
    # Sensor 2 80° rotated anti-clockwise
    dist.append(np.sqrt(rawReadings.points[2].x ** 2 + rawReadings.points[2].y ** 2))
    # Sensor 3 120° rotated anti-clockwise
    dist.append(np.sqrt(rawReadings.points[3].x ** 2 + rawReadings.points[3].y ** 2))
    # Sensor 4 160° rotated anti-clockwise
    dist.append(np.sqrt(rawReadings.points[4].x ** 2 + rawReadings.points[4].y ** 2))
    # Sensor 5 200° rotated anti-clockwise
    dist.append(np.sqrt(rawReadings.points[5].x ** 2 + rawReadings.points[5].y ** 2))
    # Sensor 6 240° rotated anti-clockwise
    dist.append(np.sqrt(rawReadings.points[6].x ** 2 + rawReadings.points[6].y ** 2))
    # Sensor 7 280° rotated anti-clockwise
    dist.append(np.sqrt(rawReadings.points[7].x ** 2 + rawReadings.points[7].y ** 2))
    # Sensor 8 320° rotated anti-clockwise
    dist.append(np.sqrt(rawReadings.points[8].x ** 2 + rawReadings.points[8].y ** 2))

    return dist


if __name__ == "__main__":
    try:
        strategyPlanner(runWithRiskEstimation=False)
    except:
        rospy.loginfo(f"Shutting down node {Nodes.STRATEGY_PLANNER.value}")
