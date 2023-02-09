#!/usr/bin/env python3
import time
import yaml
import rospy
import actionlib
import dynamic_reconfigure.client
import numpy as np
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseGoal,
    MoveBaseActionFeedback,
    MoveBaseActionResult,
    MoveBaseFeedback,
)
from std_msgs.msg import Bool
from nav_msgs.msg import Path

from prototype.msg import Risk, ProbabilityRL, ProbabilitiesRL
from utils.constants import Topics, Nodes
from utils.ros_logger import set_rospy_log_lvl
from utils.navigation_utils import trajToPath, pathToTraj
from utils.risk_estimation_utils import closestNode, getIntersection
from utils.evalmanager_client import EvalManagerClient
from risk_estimation.crash_and_remove import calculate_collosion_order
from real_robot_navigation.move_utils_cords import (
    get_base_info,
    get_pixel_location_from_acml,
)

feedbackValue = []
doneFeedback = -1
trajectory = []
evalManagerClient = EvalManagerClient()


def _moveBaseCallback(data: MoveBaseFeedback):
    global feedbackValue

    feedbackValue = []
    feedbackValue.append(data.base_position.pose.position.x)
    feedbackValue.append(data.base_position.pose.position.y)
    feedbackValue.append(data.base_position.pose.position.z)


def _moveBaseActionCallback(data: MoveBaseActionFeedback):
    pass


def _moveBaseActionDoneCallback(data: MoveBaseActionResult):
    global doneFeedback
    doneFeedback = int(data.status.status)
    print(data)


def reconfigureMovebase(lidarEnabled: Bool):
    """
    Updates the configuration of move_base. It basically adds or removes the\
    LIDAR from the used sensors in the obstacle cost layer

    Args:
        lidarEnabled (Bool): If lidas is enabled
    """
    # Load correct configuration from config file
    if lidarEnabled.data:
        rospy.set_param("/move_base/local_costmap/obstacle_layer/observation_sources", "laser_scan_sensor ir_sensor")
    else:
        rospy.set_param("/move_base/local_costmap/obstacle_layer/observation_sources", "ir_sensor")

    # Update parameters of move_base
    rospy.logdebug(f"[Strategy Planner] Reconfigured move_base. Use LIDAR: {lidarEnabled.data}")


def _setTrajectory(traj: Path):
    """Setter for trajectory"""
    global trajectory
    trajectory = []
    trajectory = pathToTraj(traj)


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


def moveBaseClientPath(path: Path) -> MoveBaseActionResult:
    """
    Executes a global strategy. It's basically driving a trajectory and using a local planner if a hazard\
    is detected which wasn't mitigated by the risk evaluation.

    Args:
        path (nav_msgs.Path): The path to drive
    """
    global feedbackValue
    global doneFeedback

    rospy.logdebug("[Strategy Planner] Starting navigation with move_base client")
    for node in path.poses[1:]:
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = node.pose.position.x
        goal.target_pose.pose.position.y = node.pose.position.y
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation.w = 1.0

        client.wait_for_server()
        # Sends the goal to the action server.
        client.send_goal(goal, feedback_cb=_moveBaseCallback)
        while not rospy.is_shutdown():
            doneFeedback = -1
            if feedbackValue:
                if (
                    (abs(feedbackValue[0] - goal.target_pose.pose.position.x) < 0.1)
                    and (abs(feedbackValue[1] - goal.target_pose.pose.position.y) < 0.1)
                    or doneFeedback >= 2
                ):
                    client.cancel_goal()
                    break

        time.sleep(0.5)
        if doneFeedback != 3 and doneFeedback != 2:
            # Error behaviour if move base didnt reach node
            print("[Strategy Planner] Error")
            continue

        doneFeedback = -1

    return doneFeedback


def moveBaseClient(pose: PoseStamped) -> MoveBaseActionResult:
    """
    Drives to a node. It's basically driving a trajectory and using a local planner if a hazard\
    is detected which wasn't mitigated by the risk evaluation.

    Args:
        path (nav_msgs.Path): The path to drive
    """
    global feedbackValue
    rospy.logdebug("[Strategy Planner] Starting navigation with move_base client")

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pose.pose.position.x
    goal.target_pose.pose.position.y = pose.pose.position.y
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.w = 1.0

    client.wait_for_server()
    # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb=_moveBaseCallback)

    while not rospy.is_shutdown():
        if feedbackValue:
            if (
                (abs(feedbackValue[0] - goal.target_pose.pose.position.x) < 0.1)
                and (abs(feedbackValue[1] - goal.target_pose.pose.position.y) < 0.1)
                or doneFeedback >= 2
            ):
                client.cancel_goal()
                break

    return client.get_result()


def detectHazard():
    """
    Identifies the hazard/collision
    """
    global currentTarget
    # Distance threshold under which a hazard is detected
    THRESHOLD_DISTANCE = 2

    while not rospy.is_shutdown():
        try:
            rawReadings = rospy.wait_for_message(Topics.IR_SENSORS.value, PointCloud)
            dist = _fetchIRReadings(rawReadings)

            if np.min(dist) < THRESHOLD_DISTANCE:
                # Detected hazard ==> stop until problem is resolved
                rospy.loginfo("[Strategy Planner] IR sensors detected hazard. Emergency breaking now")
                evalManagerClient.evalLogCollision()
                rospy.Publisher(Topics.EMERGENCY_BRAKE.value, Bool).publish(True)
        except:
            return


def chooseStrategy(riskEstimation: Risk):
    """ 
    Plans the behaviour of the robot depending on the given parameters

    Args:
        riskEstimation (custom risk-message): The risk parameters. Gets passed by the subscriber who\
                                                                          calls this function as a callback function
    """
    global trajectory
    COST_CRASH = 200
    COST_PRODUCTION_STOP = 100
    COST_COLLISIONSTOP = COST_CRASH + COST_PRODUCTION_STOP

    ###################################################
    # ----------------- Get risk parameters -----------------------
    ###################################################
    probabilities = riskEstimation.probs_rl
    for traj in riskEstimation.trajectories:
        trajectories = pathToTraj(traj)
    cummProbabilites = []
    criticalNodes = set()
    for prob in probabilities:
        probs = []
        for singleProb in prob:
            probability = singleProb.probability
            nodeIndex = singleProb.nodeIndex
            criticalNodes.add(nodeIndex)
            probs.append((probability, nodeIndex))
    cummProbabilites.append(calculate_collosion_order(probs))

    ##################################################
    # ----------------- Calculate general risk --------------------
    ##################################################
    # Risk of collision
    riskCollision = []
    # Risk of creating a production stop
    riskStop = COST_PRODUCTION_STOP
    # Risk of collision and creating a production stop
    riskCollisionStop = []
    # List of nodes where the trajectory is potentially intersected by trajectories of other robots
    intersectedNodes = []

    rospy.logdebug("[Strategy Planner] Starting expectation value calculation")
    for i in range(len(trajectories)):
        # Overall risk of collision
        riskCollision.append(np.multiply(COST_CRASH, cummProbabilites[i]))
        # Risk of colilsion and productionstop
        collisionStopProbs = 0
        for nodeNr in criticalNodes:
            # TODO Determine if critical sector crosses paths of other Robotinos
            robotEdge = [[0, 0], [1, 1]]
            for edge in robotEdge:
                criticalIntersection, isIntersected = getIntersection(
                    trajectories[nodeNr - 1],
                    trajectories[nodeNr],
                    edge[0],
                    edge[1],
                )
                if isIntersected:
                    intersectedNodes.append(nodeNr)
                    collisionStopProbs += np.multiply(COST_COLLISIONSTOP, cummProbabilites[i])

        collisionStopProbs.append(collisionStopProbs)
        rospy.logdebug(
            f"[Strategy Planner] Finished global risk estimation.\nRisk collision: {riskCollision[i]}\nRisk collision\&stop: {collisionStopProbs[i]}\nRisk combined: {riskCollision[i]+collisionStopProbs[i]}"
        )
    # Logging in Evaluationmanager
    evalManagerClient.evalLogRisk(np.average(riskCollision) + np.average(riskCollisionStop))

    #####################################################
    # ------------------- Behaviour planning ------------------------
    #####################################################
    if np.average(riskCollision) + np.average(riskCollisionStop) < riskStop:
        for i in range(len(trajectory.poses)):
            if i in criticalNodes:
                # Critical sector => Executing  under safety constraints
                rospy.logdebug("[Strategy Planner] Entering critical sector")
                localCollisionrisk = 0
                localCollisionStopRisk = 0

                # Choosing the trajectory which fits the best to the current localization and using this risk for decision making
                currentPose = rospy.wait_for_message(Topics.LOCALIZATION.value, PoseWithCovarianceStamped)
                currentPose = get_pixel_location_from_acml(
                    currentPose.pose.pose.position.x, currentPose.pose.pose.position.y, *get_base_info()
                )
                currentPose = np.array(currentPose)
                minDist = np.inf
                for j in range(len(trajectories)):
                    node = np.array((trajectories[j][i][0], trajectories[j][i][1]))
                    dist = np.linalg.norm(node - currentPose)

                    if dist < minDist:
                        minDist = dist
                        localCollisionrisk = np.multiply(COST_CRASH, probabilities[j].probabilities.probability)
                        if j in intersectedNodes:
                            localCollisionrisk = np.multiply(
                                COST_COLLISIONSTOP, probabilities[j].probabilities.probability
                            )
                # Logging in Evaluationmanager
                evalManagerClient.evalLogCriticalSector(
                    x=trajectory[i].poses.pose.position.x,
                    y=trajectory[i].poses.pose.position.y,
                    localRisk=localCollisionrisk + localCollisionStopRisk,
                )

                ############################################################
                #  ------------------------- Behaviour execution -------------------------
                ############################################################
                if localCollisionrisk + localCollisionStopRisk < riskStop:
                    # Risk is reasonable low enough => Navigate
                    result = moveBaseClient(trajectory.poses[i])
                    if result.status != 3:
                        # move base failed
                        evalManagerClient.evalLogStop()
                        return
                else:
                    # Risk is too high => Don't navigate
                    rospy.logdebug(
                        f"[Strategy Planner] Aborting navigation, risk is too high. Local risk:{localCollisionrisk + localCollisionStopRisk}"
                    )
                    break
            else:
                # Uncritical sector => Just executing navigation
                rospy.logdebug("[Strategy Planner] Moving in uncritical sector")
                # Logging in Evaluationmanager
                evalManagerClient.evalLogCriticalSector(
                    x=trajectory[i].poses.pose.position.x,
                    y=trajectory[i].poses.pose.position.y,
                )
                result = moveBaseClient(trajectory.poses[i])
                if result.status != 3:
                    # move base failed
                    evalManagerClient.evalLogStop()
                    return


def strategyPlanner(runWithRiskEstimation=True):
    """
    Runs the node itself and subscribes to all necessary topics.

    Args:
        runWithRiskEstimation (bool, optional): If it should run after risk estimation (True) or after a\
                                                                            trajectory is planned (False). Defaults to True 
    """
    rospy.init_node(Nodes.STRATEGY_PLANNER.value)
    set_rospy_log_lvl(rospy.DEBUG)
    rospy.loginfo(f"Starting node {Nodes.STRATEGY_PLANNER.value}")

    rospy.Subscriber(Topics.LIDAR_ENABLED.value, Bool, reconfigureMovebase, queue_size=10)
    rospy.Subscriber(Topics.MOVE_BASE_FEEDBACK.value, MoveBaseActionFeedback, _moveBaseActionCallback, queue_size=10)
    rospy.Subscriber(Topics.MOVE_BASE_RESULT.value, MoveBaseActionResult, _moveBaseActionDoneCallback, queue_size=10)
    if not runWithRiskEstimation:
        # Just drive the trajectory
        rospy.Subscriber(Topics.GLOBAL_PATH.value, Path, moveBaseClientPath, queue_size=1)
    else:
        # Start driving when the risk values come in
        rospy.Subscriber(Topics.GLOBAL_PATH.value, Path, _setTrajectory, queue_size=1)
        rospy.Subscriber(Topics.RISK_ESTIMATION_RL.value, Risk, chooseStrategy, queue_size=1)

    # detectHazard()
    rospy.spin()


if __name__ == "__main__":
    try:
        strategyPlanner(runWithRiskEstimation=False)
    except:
        rospy.loginfo(f"Shutting down node {Nodes.STRATEGY_PLANNER.value}")
