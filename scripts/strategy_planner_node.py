#!/usr/bin/env python3
import time
import rospy
import actionlib
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseGoal,
    MoveBaseActionResult,
    MoveBaseFeedback,
)
from std_msgs.msg import Bool, String, Int16
from nav_msgs.msg import Path
from threading import Event

from prototype.msg import (
    Risk,
    PRMNavigationGoal,
    PRMNavigationResult,
    PRMNavigationFeedback,
    PRMNavigationAction,
)
from utils.constants import Topics, Nodes, CommonPositions
from utils.ros_logger import set_rospy_log_lvl
from utils.evalmanager_client import EvalManagerClient
from utils.navigation_utils import navigateToPoint
from risk_estimation.crash_and_remove import calculate_collosion_order


doneFeedback = -1
"""Feedback from move_base. 2 and 3 are success, greater than 4 a failure"""
trajectory = []
"""The trajectory the robotino plans to drive. Given by path planner node"""
feedbackValueMoveBase = []
feedbackValuePRMNavigation = PRMNavigationFeedback()
"""Feedback of the move_base during execution"""
emergencyStop = Event()
"""An event flag indicating if all current navigation tasks should be aborted"""
fallbackMode = False
"""If robotino has detected a anomaly or lidar breakdown and is now executing in fallback mode"""
nrOfAttempt = 1
"""Number of the current attempt in planning and executing a strategy for a given trajecotry and risk estimation"""
detectedAngle = 0
"""The last known angle of the robotino. Needed when the robotino enters fallback mode"""

# Publisher
useErrorDistPub = rospy.Publisher(Topics.USE_ERRORDIST.value, Bool, queue_size=10)
targetPub = rospy.Publisher(Topics.TARGET.value, PoseStamped, queue_size=10)
responsePub = rospy.Publisher(
    Topics.STRATEGY_PLANNER_RESPONSE.value, String, queue_size=10
)
obstacleMarginPub = rospy.Publisher(
    Topics.OBSTACLE_MARGIN.value, Int16, queue_size=10, latch=True
)
emergencyBrakePub = rospy.Publisher(
    Topics.EMERGENCY_BRAKE.value, Bool, queue_size=10, latch=True
)


def _moveBaseCallback(data: MoveBaseFeedback):
    global feedbackValueMoveBase

    feedbackValueMoveBase = []
    feedbackValueMoveBase.append(data.base_position.pose.position.x)
    feedbackValueMoveBase.append(data.base_position.pose.position.y)
    feedbackValueMoveBase.append(data.base_position.pose.position.z)


def _prmNavigationCallback(data: PRMNavigationFeedback):
    global feedbackValuePRMNavigation
    feedbackValuePRMNavigation = data


def _moveBaseActionDoneCallback(data: MoveBaseActionResult):
    global doneFeedback
    doneFeedback = int(data.status.status)


def _setTrajectory(traj: Path):
    """Setter for trajectory"""
    global trajectory
    trajectory = traj


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
    offset = 0.22
    # Sensor 0 forward
    dist.append(
        np.sqrt(rawReadings.points[0].x ** 2 + rawReadings.points[0].y ** 2) - offset
    )
    # Sensor 1 40° rotated anti-clockwise
    dist.append(
        np.sqrt(rawReadings.points[1].x ** 2 + rawReadings.points[1].y ** 2) - offset
    )
    # Sensor 2 80° rotated anti-clockwise
    dist.append(
        np.sqrt(rawReadings.points[2].x ** 2 + rawReadings.points[2].y ** 2) - offset
    )
    # Sensor 3 120° rotated anti-clockwise
    dist.append(
        np.sqrt(rawReadings.points[3].x ** 2 + rawReadings.points[3].y ** 2) - offset
    )
    # Sensor 4 160° rotated anti-clockwise
    dist.append(
        np.sqrt(rawReadings.points[4].x ** 2 + rawReadings.points[4].y ** 2) - offset
    )
    # Sensor 5 200° rotated anti-clockwise
    dist.append(
        np.sqrt(rawReadings.points[5].x ** 2 + rawReadings.points[5].y ** 2) - offset
    )
    # Sensor 6 240° rotated anti-clockwise
    dist.append(
        np.sqrt(rawReadings.points[6].x ** 2 + rawReadings.points[6].y ** 2) - offset
    )
    # Sensor 7 280° rotated anti-clockwise
    dist.append(
        np.sqrt(rawReadings.points[7].x ** 2 + rawReadings.points[7].y ** 2) - offset
    )
    # Sensor 8 320° rotated anti-clockwise
    dist.append(
        np.sqrt(rawReadings.points[8].x ** 2 + rawReadings.points[8].y ** 2) - offset
    )

    return dist


def moveBaseClientPath(path: Path) -> MoveBaseActionResult:
    """
    Client for the move_base node. It's basically driving a trajectory and using a local planner if a hazard\
    is detected which wasn't mitigated by the risk evaluation.

    Args:
        path (nav_msgs.Path): The path to drive
    """

    rospy.logdebug("[Strategy Planner] Starting navigation with move_base client")
    for node in path.poses[1:]:
        response = moveBaseClient(node)
        if response >= 4:
            # Error behaviour if move base didnt reach node
            print("[Strategy Planner] Error")
            break

    return response


def moveBaseClient(pose: PoseStamped) -> int:
    """
    Drives to a node. It's basically driving a trajectory and using a local planner if a hazard\
    is detected which wasn't mitigated by the risk evaluation.

    Args:
        path (nav_msgs.Path): The path to drive
    """
    global feedbackValueMoveBase
    global emergencyStop
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

    try:
        client.wait_for_server(timeout=rospy.Duration(secs=5))
        # Sends the goal to the action server.
        client.send_goal(goal, feedback_cb=_moveBaseCallback)
    except:
        client.cancel_goal()
        return 4

    while not rospy.is_shutdown():
        doneFeedback = -1

        if emergencyStop.is_set():
            rospy.logwarn(
                "[Strategy Planner] Move_base client aborted operation because of emergency stop"
            )
            # client.cancel_all_goals()
            client.cancel_goal()
            return 2

        if feedbackValueMoveBase:
            if (
                (abs(feedbackValueMoveBase[0] - goal.target_pose.pose.position.x) < 0.1)
                and (
                    abs(feedbackValueMoveBase[1] - goal.target_pose.pose.position.y)
                    < 0.1
                )
                or doneFeedback >= 2
            ):
                client.cancel_goal()
                break

    time.sleep(0.5)

    return doneFeedback


def navigate(pose: PoseStamped, positionAssumption: PoseStamped, angleAssumption):
    """
    Uses the PRM navigation method to navigate. It basically rotates towards the target and then drives forward.\
    This method doesn't use the lidar and works with position assumptions

    Args:
        pose (PoseStamped): The target to which the robotino should drive
        positionAssumption (PoseStamped): The current position of the robotino
        angleassumption (float): The current angle of the robotino
    """
    global emergencyStop
    global feedbackValuePRMNavigation

    target = Point()
    target.x = pose.pose.position.x
    target.y = pose.pose.position.y

    start = Point()
    start.x = positionAssumption.pose.position.x
    start.y = positionAssumption.pose.position.y
    client = actionlib.SimpleActionClient(Nodes.CONTROL.value, PRMNavigationAction)
    # Creates a new goal with the MoveBaseGoal constructor
    goal = PRMNavigationGoal()
    goal.angleAssumption = angleAssumption
    goal.positionAssumption = start
    goal.target = target

    # client.wait_for_server()
    # # Sends the goal to the action server.
    # client.send_goal(goal, feedback_cb=_prmNavigationCallback)

    # while not rospy.is_shutdown():
    #     if emergencyStop.is_set():
    #         rospy.logwarn(
    #             "[Strategy Planner] Move_base client aborted operation because of emergency stop"
    #         )
    #         # client.cancel_all_goals()
    #         client.cancel_goal()
    #         return feedbackValuePRMNavigation.currentAngle

    # result = client.get_result()
    # angleChange = result.currentAngle

    angleChange, _ = navigateToPoint(target, start, angleAssumption)

    new_rotation = angleAssumption + angleChange
    if new_rotation < (-np.pi):
        new_rotation = new_rotation + np.pi * 2
    if new_rotation > (np.pi):
        new_rotation = new_rotation - np.pi * 2

    return new_rotation


def detectHazard(
    rawReadings: PointCloud,
    evalLoggingEnabled: bool = False,
):
    """
    Uses the IR readings to activate a emergency stop if robotino is too near to an obstacle

    Args:
        rawReadings (PointCloud): The readings from the IR sensors
        evalLoggingEnabled (bool, optional): If evaluation metrics should be send to evaluationmanager (mainly used in Felix Brugger
                                        thesis, not neccessary for normal operation). Defaults to False
    """
    global emergencyStop
    # Distance threshold under which a hazard is detected
    THRESHOLD_COLLISIONDETECTION_DISTANCE = 0.07
    THRESHOLD_EMERGENCYSTOP_DISTANCE = 0.15

    dist = _fetchIRReadings(rawReadings)
    # print(np.min(dist))
    if np.min(dist) < THRESHOLD_COLLISIONDETECTION_DISTANCE:
        # Detected collision
        rospy.loginfo("[Strategy Planner] IR sensors detected collision")
        if evalLoggingEnabled:
            EvalManagerClient().evalLogCollision()
        # rospy.Publisher(Topics.EMERGENCY_BRAKE.value, Bool).publish(True)
    elif np.min(dist) < THRESHOLD_EMERGENCYSTOP_DISTANCE:
        rospy.loginfo(
            "[Strategy Planner] IR sensors detected obstacle and activates emergencybreak"
        )
        emergencyStop.set()
        if evalLoggingEnabled:
            EvalManagerClient().evalLogStop()


def activateFallbackBehaviour(enabled: Bool, reason: str = "lidarbreakdown"):
    """
    It activates/ deactives fallback behaviour and starts fallback Behaviour strategies

    Args:
        enabled (bool): If fallback behaviour should be enabled
        reason (str, optional): The reason why the fallback behaviour is activated.\
                                Either "lidarbreakdown" or "anomalyDetected". Defaults to first one
    """
    global emergencyStop
    global fallbackMode

    if enabled.data and not fallbackMode:
        if "lidar" in reason:
            rospy.logwarn(
                "[Strategy Planner] EMERGENCY-STOP: LIDAR has breakdown. Executing fallback behaviour"
            )
        else:
            rospy.logwarn(
                "[Strategy Planner] EMERGENCY-STOP: Anomaly detected. Executing fallback behaviour"
            )
        # Set event flag so currently exectuing startegies are aborted
        emergencyStop.set()
        emergencyBrakePub.publish(True)
        # Tell strategy planner that its in fallbackMode
        fallbackMode = True
        # Start fallback behaviour after a short time to let all nodes set the necessary parameters
        executeFallbackMeasures()
    elif not enabled.data:
        rospy.loginfo("[Strategy Planner] EMERGENCY-STOP has been reset")
        # Clear event flag so strategy planner can run under normal behaviour
        emergencyStop.clear()
        # Tell strategy planner that it is can run under normal behaviour again
        fallbackMode = False


def executeFallbackMeasures():
    """
    Executes the behaviour when detecting a lidar anomaly or breakdown.
    The strategy is to drive to a safe place when the risk is high enough
    """
    global emergencyStop
    global detectedAngle
    # Release emergency stop flag so strategy planner can run again if risk values come in

    # Start the path planning process to navigate to safe spot
    try:
        # Avoid obstacles in path planning
        obstacleMarginPub.publish(4)
        fallbackPose = rospy.wait_for_message(
            Topics.FALLBACK_POSE.value, PoseWithCovarianceStamped
        )
        detectedAngle = fallbackPose.pose.pose.orientation.z
        targetPub.publish(CommonPositions.SAFE_SPOT.value)
    except Exception as e:
        rospy.logwarn(f"[Strategy Planner] Couldn't publish safe spot as target: {e}")

    try:
        # Activate usage of own error distribution so it uses the last known error values for risk estimation
        useErrorDistPub.publish(True)
    except Exception as e:
        rospy.logwarn(f"[Strategy Planner] Couldn't publish safe spot as target: {e}")


def fallbackLocalStrategy(
    iterationNr,
    criticalNodes: list,
    criticalSectors,
    angleAssumption: float,
    riskStop: float = 100,
    evalLoggingEnabled: bool = False,
):
    """
    Executes the fallback strategy, which effectiffely is purely a local strategy because the global strategy\
    is to drive as far as possible and reasonable under risk estimation (=> Excuting local strategy as far as possible)

    Args:
        iterationNr (int): Nr of node in trajectory to which should be driven
        criticalNodes (list): A list of node indexes where a collision could potentially happen
        critivalSectors (list of CriticalSector): All known critical sectors in trajectory
        angleAssumption (float): The assumed current angle of the ego-robotino
        riskStop (float, optional): The threshold risk which determines if robotino should drive or stop 
        evalLoggingEnabled (bool, optional): If evaluation metrics should be send to evaluationmanager (mainly used in Felix Brugger
                                            thesis, not neccessary for normal operation). Defaults to False
    Returns:
        navResponse (bool): If navigation was successful
        angleAssumption (float): The assumed new angle of the robot after navigation
    """
    global trajectory
    # global nrOfAttempt
    global emergencyStop

    emergencyStop.clear()
    if iterationNr in criticalNodes:
        # Critical sector => Executing under safety constraints
        rospy.logdebug("[Strategy Planner] Entering critical sector")
        localRisk = []

        for sectors in criticalSectors:
            for node in sectors.sectors:
                if node.node == iterationNr:
                    localRisk.append(node.risk)
                    break

        if evalLoggingEnabled:
            # Logging in Evaluationmanager
            EvalManagerClient().evalLogCriticalSector(
                x=trajectory.poses[iterationNr].pose.position.x,
                y=trajectory.poses[iterationNr].pose.position.y,
                localRisk=np.average(localRisk),
            )

        # Check if emergencystop is set before start moving
        if emergencyStop.is_set():
            responsePub.publish("Error: Trajectory execution did fail")
            rospy.logwarn(
                "[Strategy Planner] Emergency stop detected. Aborting strategy execution"
            )
            return

        if np.average(localRisk) <= riskStop:
            # Risk is reasonable low enough => Navigate
            angleAssumption = navigate(
                trajectory.poses[iterationNr],
                positionAssumption=trajectory.poses[iterationNr - 1],
                angleAssumption=angleAssumption,
            )
    else:
        # Uncritical sector => Just executing navigation
        rospy.logdebug("[Strategy Planner] Moving in uncritical sector")

        if emergencyStop.is_set():
            rospy.logwarn(
                "[Strategy Planner] Emergency stop detected. Aborting strategy execution"
            )
            return False, angleAssumption

        if evalLoggingEnabled:
            # Logging in Evaluationmanager
            EvalManagerClient().evalLogUncriticalSector(
                x=trajectory.poses[iterationNr].pose.position.x,
                y=trajectory.poses[iterationNr].pose.position.y,
            )

        angleAssumption = navigate(
            trajectory.poses[iterationNr],
            positionAssumption=trajectory.poses[iterationNr - 1],
            angleAssumption=angleAssumption,
        )

    return True, angleAssumption


def standardLocalStrategy(
    iterationNr: int,
    criticalNodes: set,
    criticalSectors,
    riskStop: float = 100,
    useMoveBaseClient=True,
    assumedPosition: PoseWithCovarianceStamped = None,
    evalLoggingEnabled: bool = False,
):
    """
    Executes the standard local strategy, which effectiffely uses the move_base client to navigate to the node

    Args:
        iterationNr (int): Nr of node in trajectory to which should be driven
        criticalNodes (list): A list of node indexes where a collision could potentially happen
        critivalSectors (list of CriticalSector): All known critical sectors in trajectory
        riskStop (float, optional): The threshold risk which determines if robotino should drive or stop
        evalLoggingEnabled (bool, optional): If evaluation metrics should be send to evaluationmanager (mainly used in Felix Brugger
                                            thesis, not neccessary for normal operation). Defaults to False
    Returns:
        navResponse (bool): If navigation was successful
    """
    global trajectory
    global emergencyStop
    obstacleMarginPub.publish(2)

    if iterationNr in criticalNodes:
        # Critical sector => Executing under safety constraints
        rospy.logdebug("[Strategy Planner] Entering critical sector")
        localRisk = []

        for sectors in criticalSectors:
            for node in sectors.sectors:
                if node.node == iterationNr:
                    localRisk.append(node.risk)
                    break

        if evalLoggingEnabled:
            # Logging in Evaluationmanager
            EvalManagerClient().evalLogCriticalSector(
                x=trajectory.poses[iterationNr].pose.position.x,
                y=trajectory.poses[iterationNr].pose.position.y,
                localRisk=np.average(localRisk),
            )

        # Check if emergencystop is set before start moving
        if emergencyStop.is_set():
            responsePub.publish("Error: Trajectory execution did fail")
            rospy.logwarn(
                "[Strategy Planner] Emergency stop detected. Aborting strategy execution"
            )
            return False

        if np.average(localRisk) <= riskStop:
            # Risk is reasonable low enough => Navigate
            if useMoveBaseClient:
                result = moveBaseClient(trajectory.poses[iterationNr])
                if result >= 4:
                    # move base failed
                    EvalManagerClient().evalLogStop()
                    return False
            else:
                angleAssumption = assumedPosition.pose.pose.orientation.z
                poseAssumption = PoseStamped()
                poseAssumption.pose.position.x = assumedPosition.pose.pose.position.x
                poseAssumption.pose.position.y = assumedPosition.pose.pose.position.y
                _ = navigate(
                    trajectory.poses[iterationNr],
                    poseAssumption,
                    angleAssumption=angleAssumption,
                )
            # Check if emergencystop is set before start moving
            if emergencyStop.is_set():
                responsePub.publish("Error: Trajectory execution did fail")
                rospy.logwarn(
                    "[Strategy Planner] Emergency stop detected. Aborting strategy execution"
                )
                return False
        else:
            # Risk is too high => Don't navigate
            rospy.logdebug(
                f"[Strategy Planner] Aborting navigation, risk is too high. Local risk:{np.average(localRisk)}"
            )
            if evalLoggingEnabled:
                EvalManagerClient().evalLogStop()
            return False
    else:
        # Uncritical sector => Just executing navigation
        rospy.logdebug("[Strategy Planner] Moving in uncritical sector")
        if evalLoggingEnabled:
            # Logging in Evaluationmanager
            EvalManagerClient().evalLogUncriticalSector(
                x=trajectory.poses[iterationNr].pose.position.x,
                y=trajectory.poses[iterationNr].pose.position.y,
            )

        if emergencyStop.is_set():
            responsePub.publish("Error: Trajectory execution did fail")
            rospy.logwarn(
                "[Strategy Planner] Emergency stop detected. Aborting strategy execution"
            )
            return False

        if useMoveBaseClient:
            result = moveBaseClient(trajectory.poses[iterationNr])
            if result >= 4:
                # move base failed
                if evalLoggingEnabled:
                    EvalManagerClient().evalLogStop()
                return False
        else:
            angleAssumption = assumedPosition.pose.pose.orientation.z
            poseAssumption = PoseStamped()
            poseAssumption.pose.position.x = assumedPosition.pose.pose.position.x
            poseAssumption.pose.position.y = assumedPosition.pose.pose.position.y
            _ = navigate(
                trajectory.poses[iterationNr],
                poseAssumption,
                angleAssumption=angleAssumption,
            )

    return True


def fallbackGlobalStrategy(
    riskGlobal: list, riskStop: float = 100, MAX_ATTEMPTS: int = 3
):
    """
    The global strategy when operating under fallback behaviour. Basically the same as standard global strategy,
    but the robotino drives as far as possible without risk, even if global risk is above threshold after maximum number of replannings

    Args:
        riskGlobal (list of float): The global risks from the risk estimator
        riskStop (float, optional): The threshold under which the risk is viable
        MAX_ATTEMTPS (int, optional): The maximum number of replannings. After this, this navigation is cancelled
    """
    global trajectory
    global nrOfAttempt

    if (np.average(riskGlobal) > riskStop) and (nrOfAttempt < MAX_ATTEMPTS):
        rospy.loginfo(
            f"[Strategy Planner] Risk too high. Restarting risk estimation. Risk: {np.average(riskGlobal)}"
        )
        # Retry risk estimation
        nrOfAttempt = nrOfAttempt + 1
        obstacleMarginPub.publish(2 * nrOfAttempt)
        targetPub.publish(CommonPositions.SAFE_SPOT.value)
        return False
    elif (np.average(riskGlobal) > riskStop) and (nrOfAttempt >= MAX_ATTEMPTS):
        rospy.loginfo(
            f"[Strategy Planner] Risk too high and maximal number of attemps exceeded. Navigate as far as possible, but don't enter critical sectors. Risk: {np.average(riskGlobal)}"
        )
        # Consider navigation finally too risky
        return True

    return True


def standardGlobalStrategy(
    riskGlobal: list,
    riskStop: float = 100,
    MAX_ATTEMPTS: int = 3,
    evalLoggingEnabled: bool = False,
):
    """
    The global strategy when operating under normal circumstances. Just checks if risk is under threshold,
    otherwise restart replanning with increased margin to obstacles until maximal number of replannings is reached

    Args:
        riskGlobal (list of float): The global risks from the risk estimator
        riskStop (float, optional): The threshold under which the risk is viable
        MAX_ATTEMTPS (int, optional): The maximum number of replannings. After this, this navigation is cancelled
        evalLoggingEnabled (bool, optional): If evaluation metrics should be send to evaluationmanager (mainly used in Felix Brugger
                                            thesis, not neccessary for normal operation). Defaults to False
    """
    global trajectory
    global nrOfAttempt

    if (np.average(riskGlobal) > riskStop) and (nrOfAttempt < MAX_ATTEMPTS):
        rospy.loginfo(
            f"[Strategy Planner] Risk too high. Restarting risk estimation. Risk: {np.average(riskGlobal)}"
        )
        # Retry risk estimation
        nrOfAttempt = nrOfAttempt + 1
        obstacleMarginPub.publish(2 * nrOfAttempt)
        goalPose = trajectory.poses[len(trajectory.poses) - 1]
        targetPub.publish(goalPose)
        return False
    elif (np.average(riskGlobal) > riskStop) and (nrOfAttempt >= MAX_ATTEMPTS):
        rospy.loginfo(
            f"[Strategy Planner] Risk too high and maximal number of attemps exceeded. Abort navigation task. Risk: {np.average(riskGlobal)}"
        )
        # Consider navigation finally too risky
        nrOfAttempt = 1
        obstacleMarginPub.publish(2 * nrOfAttempt)
        if evalLoggingEnabled:
            EvalManagerClient().evalLogStartedTask()
            EvalManagerClient().evalLogRisk(np.average(riskGlobal))
            EvalManagerClient().evalLogStop()
        return False

    return True


def chooseStrategy(
    riskEstimation: Risk,
    useMoveBase=True,
    evalLoggingEnabled: bool = False,
):
    """ 
    Plans the behaviour of the robot depending on the given parameters

    Args:
        riskEstimation (custom risk-message): The risk parameters. Gets passed by the subscriber who\
                                             calls this function as a callback function
        evalLoggingEnabled (bool, optional): If evaluation metrics should be send to evaluationmanager (mainly used in Felix Brugger
                                             thesis, not neccessary for normal operation). Defaults to False
    """
    global trajectory
    global emergencyStop
    global fallbackMode
    global detectedAngle
    riskGlobal = riskEstimation.globalRisks
    criticalSectors = riskEstimation.criticalSectors

    if not fallbackMode:
        rospy.loginfo(
            f"[Strategy Planner] Received risk estimation. Start strategy planning"
        )
    else:
        rospy.loginfo(
            f"[Strategy Planner] Received risk estimation. Start strategy planning with fallback behaviour"
        )

    criticalNodes = set()
    for sectors in criticalSectors:
        for node in sectors.sectors:
            criticalNodes.add(node.node)

    ################################################
    # ----------- Global estimation ---------------
    ################################################

    if not fallbackMode:
        responseGlobalStrat = standardGlobalStrategy(
            riskGlobal, riskStop=100, MAX_ATTEMPTS=2
        )

        if responseGlobalStrat == False:
            return False
    else:
        responseGlobalStrat = fallbackGlobalStrategy(
            riskGlobal, riskStop=100, MAX_ATTEMPTS=2
        )

        if responseGlobalStrat == False:
            return False

    #####################################################
    # ------- Local estimation & behaviour execution ----
    #####################################################
    angleAssumption = 0
    emergencyStop.clear()
    emergencyBrakePub.publish(False)
    if evalLoggingEnabled:
        EvalManagerClient().evalLogStartedTask()
        EvalManagerClient().evalLogRisk(np.average(riskGlobal))

    for i in range(len(trajectory.poses)):
        # We assume that we havn't to navigate to start node because were already there
        if i == 0:
            # Purely for logging in evaluationmanager
            collisionProbs = []
            rawProbs = riskEstimation.probs_rl
            for probs in rawProbs:
                for singleProb in probs.probabilities:
                    collisionProbs.append(
                        (singleProb.probability, singleProb.nodeIndex)
                    )
            cummulativeProb = calculate_collosion_order(collisionProbs)
            if evalLoggingEnabled:
                EvalManagerClient().evalLogCollisionProb(cummulativeProb)
            continue
        # Set initial angle from which the angle starts gets started to be interpolated
        # depending on the navigation responses
        if i == 1 and fallbackMode:
            if (i in criticalNodes) and fallbackMode:
                rospy.loginfo(
                    "[Strategy Planner] First sector is critical sector. Aborting navigation"
                )
                return False
            angleAssumption = detectedAngle

        # ---- Standard strategy -----
        if not fallbackMode:
            if useMoveBase == False:
                posMsg = rospy.wait_for_message(
                    Topics.LOCALIZATION.value, PoseWithCovarianceStamped
                )
                angleAssumption = posMsg.pose.pose.orientation.z

                responseLocalStrat = standardLocalStrategy(
                    i,
                    criticalNodes,
                    criticalSectors,
                    useMoveBaseClient=False,
                    assumedPosition=posMsg,
                )

                if responseLocalStrat == False:
                    return False
            else:
                responseLocalStrat = standardLocalStrategy(
                    i,
                    criticalNodes,
                    criticalSectors,
                )

                if responseLocalStrat == False:
                    return False
        # ---- Fallback strategy ----
        else:
            responseLocalStrat, angleAssumption = fallbackLocalStrategy(
                i, criticalNodes, criticalSectors, angleAssumption
            )

        if responseLocalStrat == False:
            return False

        # Check if emergencystop is set before starting navigation to next node
        if emergencyStop.is_set():
            responsePub.publish("Error: Trajectory execution did fail")
            rospy.logwarn(
                "[Strategy Planner] Emergency stop detected. Aborting strategy execution"
            )
            return False

    try:
        if evalLoggingEnabled:
            EvalManagerClient().evalLogSuccessfulTask()
        responsePub.publish("Success")
    except Exception as e:
        print(e)

    rospy.loginfo("[Strategy Planner] Finished strategy execution")


def strategyPlanner(
    runWithRiskEstimation=True,
    evalLoggingEnabled: bool = False,
):
    """
    Runs the node itself and subscribes to all necessary topics.

    Args:
        runWithRiskEstimation (bool, optional): If it should run after risk estimation (True) or after a\
                                                trajectory is planned (False). Defaults to True 
        evalLoggingEnabled (bool, optional): If evaluation metrics should be send to evaluationmanager (mainly used in Felix Brugger
                                             thesis, not neccessary for normal operation). Defaults to False
    """
    rospy.init_node(Nodes.STRATEGY_PLANNER.value)
    set_rospy_log_lvl(rospy.DEBUG)
    rospy.loginfo(f"Starting node {Nodes.STRATEGY_PLANNER.value}")

    if not runWithRiskEstimation:
        # Just drive the trajectory
        rospy.Subscriber(
            Topics.GLOBAL_PATH.value, Path, moveBaseClientPath, queue_size=10
        )
    else:
        # ROS setter for trajectory, its executet when risk values come in
        rospy.Subscriber(Topics.GLOBAL_PATH.value, Path, _setTrajectory, queue_size=10)
        # Start driving when the risk values come in
        rospy.Subscriber(
            Topics.RISK_ESTIMATION_RL.value,
            Risk,
            chooseStrategy,
            queue_size=10,
            callback_args=[evalLoggingEnabled],
        )

    # For fallback behaviour
    rospy.Subscriber(
        Topics.ANOMALY_DETECTED.value,
        Bool,
        activateFallbackBehaviour,
        queue_size=10,
        callback_args=["anomaly detected"],
    )
    rospy.Subscriber(
        Topics.LIDAR_BREAKDOWN.value, Bool, activateFallbackBehaviour, queue_size=10
    )
    # rospy.Subscriber(Topics.IR_SENSORS.value, PointCloud, detectHazard, queue_size=10)

    # Callbacks of move_base client
    rospy.Subscriber(
        Topics.MOVE_BASE_RESULT.value,
        MoveBaseActionResult,
        _moveBaseActionDoneCallback,
        queue_size=10,
    )

    rospy.spin()


if __name__ == "__main__":
    # try:
    strategyPlanner(runWithRiskEstimation=True, evalLoggingEnabled=False)
# except Exception as e:
#     print(e)
#     rospy.loginfo(f"Shutting down node {Nodes.STRATEGY_PLANNER.value}")
