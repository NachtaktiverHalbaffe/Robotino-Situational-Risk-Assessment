#!/usr/bin/env python3
import time
import rospy
import actionlib
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
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
from threading import Event

from prototype.msg import Risk
from utils.constants import Topics, Nodes, CommonPositions
from utils.ros_logger import set_rospy_log_lvl
from utils.evalmanager_client import EvalManagerClient


feedbackValue = []
doneFeedback = -1
trajectory = []
evalManagerClient = EvalManagerClient()
emergencyStop = Event()
fallbackMode = False
nrOfAttempt = 0
useErrorDistPub = rospy.Publisher(Topics.USE_ERRORDIST.value, Bool, queue_size=10)
targetPub = rospy.Publisher(Topics.TARGET.value, PoseStamped, queue_size=10)
navPub = rospy.Publisher(Topics.NAV_POINT.value, Point, queue_size=10)


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


def reconfigureMovebase(lidarEnabled: Bool):
    """
    Updates the configuration of move_base. It basically adds or removes the\
    LIDAR from the used sensors in the obstacle cost layer

    Args:
        lidarEnabled (Bool): If lidas is enabled
    """
    # Load correct configuration from config file
    if lidarEnabled.data:
        rospy.set_param(
            "/move_base/local_costmap/obstacle_layer/observation_sources",
            "laser_scan_sensor ir_sensor",
        )
    else:
        rospy.set_param(
            "/move_base/local_costmap/obstacle_layer/observation_sources", "ir_sensor"
        )

    # Update parameters of move_base
    rospy.logdebug(
        f"[Strategy Planner] Reconfigured move_base. Use LIDAR: {lidarEnabled.data}"
    )


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
    Executes a global strategy. It's basically driving a trajectory and using a local planner if a hazard\
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


def moveBaseClient(pose: PoseStamped) -> MoveBaseActionResult:
    """
    Drives to a node. It's basically driving a trajectory and using a local planner if a hazard\
    is detected which wasn't mitigated by the risk evaluation.

    Args:
        path (nav_msgs.Path): The path to drive
    """
    global feedbackValue
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

    client.wait_for_server()
    # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb=_moveBaseCallback)

    while not rospy.is_shutdown():
        doneFeedback = -1

        if emergencyStop.is_set():
            rospy.logwarn(
                "[Strategy Planner] Move_base client aborted operation because of emergency stop"
            )
            client.cancel_all_goals()
            return 4

        if feedbackValue:
            if (
                (abs(feedbackValue[0] - goal.target_pose.pose.position.x) < 0.1)
                and (abs(feedbackValue[1] - goal.target_pose.pose.position.y) < 0.1)
                or doneFeedback >= 2
            ):
                client.cancel_goal()
                break

    time.sleep(0.5)

    return doneFeedback


def navigate(pose: PoseStamped):
    navMsg = Point()
    navMsg.x = pose.pose.position.x
    navMsg.y = pose.pose.position.y
    navPub.publish(navMsg)

    response = rospy.wait_for_message(Topics.NAVIGATION_RESPONSE.value, Bool)

    return response


def detectHazard():
    """
    Identifies the hazard/collision
    """
    global currentTarget
    # Distance threshold under which a hazard is detected
    THRESHOLD_DISTANCE = 0.1

    while not rospy.is_shutdown():
        try:
            rawReadings = rospy.wait_for_message(Topics.IR_SENSORS.value, PointCloud)
            dist = _fetchIRReadings(rawReadings)
            if np.min(dist) < THRESHOLD_DISTANCE:
                # Detected hazard ==> stop until problem is resolved
                rospy.loginfo(
                    "[Strategy Planner] IR sensors detected hazard. Emergency breaking now"
                )
                evalManagerClient.evalLogCollision()
                # rospy.Publisher(Topics.EMERGENCY_BRAKE.value, Bool).publish(True)
        except:
            return


def activateFallbackBehaviour(enabled: Bool):
    """
    It activates/ deactives fallback behaviour and starts fallback Behaviour strategies

    Args:
        lidarEnabled:
    """
    global emergencyStop
    global fallbackMode

    if enabled.data:
        rospy.logwarn("[Strategy Planner] EMERGENCY-STOP: LIDAR has breakdown")
        # Set event flag so currently exectuing startegies are aborted
        emergencyStop.set()
        # Tell strategy planner that its in fallbackMode
        fallbackMode = True
        # Start fallback behaviour after a short time to let all nodes set the necessary parameters
        time.sleep(3)
        executeAnomalyStrategy()
    else:
        rospy.loginfo("[Strategy Planner] EMERGENCY-STOP has been reset")
        # Clear event flag so strategy planner can run under normal behaviour
        emergencyStop.clear()
        # Tell strategy planner that it is can run under normal behaviour again
        fallbackMode = False


def executeAnomalyStrategy():
    """
    Executes the behaviour when detecting a lidar anomaly or breakdown.
    The strategy is to drive to a safe place when the risk is high enough.
    """
    global emergencyStop
    # Release emergency stop flag so strategy planner can run again if risk values come in
    emergencyStop.clear()
    try:
        # Activate usage of own error distribution so it uses the last known error values for risk estimation
        useErrorDistPub.publish(True)
    except Exception as e:
        rospy.logwarn(f"[Strategy Planner] Couldn't publish safe spot as target: {e}")

    # Start the path planning process to navigate to safe spot
    try:
        targetPub.publish(CommonPositions.SAFE_SPOT.value)
    except Exception as e:
        rospy.logwarn(f"[Strategy Planner] Couldn't publish safe spot as target: {e}")


def chooseStrategy(riskEstimation: Risk):
    """ 
    Plans the behaviour of the robot depending on the given parameters

    Args:
        riskEstimation (custom risk-message): The risk parameters. Gets passed by the subscriber who\
                                                                          calls this function as a callback function
    """
    global nrOfAttempt
    global trajectory
    global emergencyStop
    global fallbackMode

    MAX_ATTEMPTS = 3

    riskGlobal = riskEstimation.globalRisks
    criticalSectors = riskEstimation.criticalSectors
    # risk is simply the cost of a stop because probability is assumed to be 1
    riskStop = 100

    rospy.loginfo(
        f"[Strategy Planner] Received risk estimation. Start strategy planning"
    )
    criticalNodes = set()
    for sectors in criticalSectors:
        for node in sectors.sectors:
            criticalNodes.add(node.node)

    ################################################
    # ----------- Global estimation ---------------
    ################################################
    if (
        (np.average(riskGlobal) > riskStop)
        and (nrOfAttempt < MAX_ATTEMPTS)
        and not fallbackMode
    ):
        rospy.loginfo(
            f"[Strategy Planner] Risk too high. Restarting risk estimation. Risk: {np.average(riskGlobal)}"
        )
        # Retry risk estimation
        nrOfAttempt = nrOfAttempt + 1
        goalPose = trajectory.poses[len(trajectory.poses) - 1]
        targetPub.publish(goalPose)
        return
    elif (
        (np.average(riskGlobal) > riskStop)
        and (nrOfAttempt >= MAX_ATTEMPTS)
        and not fallbackMode
    ):
        rospy.loginfo(
            f"[Strategy Planner] Risk too high and maximal number of attemps exceeded. Abort navigation task. Risk: {np.average(riskGlobal)}"
        )
        # Retry risk estimation
        nrOfAttempt = 0
        evalManagerClient.evalLogStop()
        return

    #####################################################
    # ------- Local estimation & behaviour execution ----
    #####################################################
    for i in range(len(trajectory.poses)):
        if fallbackMode:
            # Wait little time so camera based localization can settle a little bit
            time.sleep(5)

        if i in criticalNodes:
            # Critical sector => Executing under safety constraints
            rospy.logdebug("[Strategy Planner] Entering critical sector")
            localRisk = []

            for sectors in criticalSectors:
                for node in sectors.sectors:
                    if node.node == i:
                        localRisk.append(node.risk)
                        break

            # Logging in Evaluationmanager
            evalManagerClient.evalLogCriticalSector(
                x=trajectory.poses[i].pose.position.x,
                y=trajectory.poses[i].pose.position.y,
                localRisk=np.average(localRisk),
            )

            # Check if emergencystop is set before start moving
            if emergencyStop.is_set():
                rospy.logwarn(
                    "[Strategy Planner] Emergency stop detected. Aborting strategy execution"
                )
                return

            if np.average(localRisk) < riskStop:
                # Risk is reasonable low enough => Navigate
                if not fallbackMode:
                    result = moveBaseClient(trajectory.poses[i])
                    if result >= 4:
                        # move base failed
                        evalManagerClient.evalLogStop()
                        return
                else:
                    response = navigate(trajectory.poses[i])
                    if not response:
                        evalManagerClient.evalLogStop()
                        return
            else:
                # Risk is too high => Don't navigate
                rospy.logdebug(
                    f"[Strategy Planner] Aborting navigation, risk is too high. Local risk:{np.average(localRisk)}"
                )
                evalManagerClient.evalLogStop()
                break

            if emergencyStop.is_set():
                rospy.logwarn(
                    "[Strategy Planner] Emergency stop detected. Aborting strategy execution"
                )
                return
        else:
            # Uncritical sector => Just executing navigation
            if emergencyStop.is_set():
                rospy.logwarn(
                    "[Strategy Planner] Emergency stop detected. Aborting strategy execution"
                )
                return
            rospy.logdebug("[Strategy Planner] Moving in uncritical sector")
            # Logging in Evaluationmanager
            evalManagerClient.evalLogUncriticalSector(
                x=trajectory.poses[i].pose.position.x,
                y=trajectory.poses[i].pose.position.y,
            )

            if not fallbackMode:
                result = moveBaseClient(trajectory.poses[i])
                if result >= 4:
                    # move base failed
                    evalManagerClient.evalLogStop()
                    return
            else:
                response = navigate(trajectory.poses[i])
                if not response:
                    evalManagerClient.evalLogStop()
                    return

            if emergencyStop.is_set():
                rospy.logwarn(
                    "[Strategy Planner] Emergency stop detected. Aborting strategy execution"
                )
                return

    rospy.loginfo("[Strategy Planner] Finished strategy execution")


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
            Topics.RISK_ESTIMATION_RL.value, Risk, chooseStrategy, queue_size=10
        )
        # Start fallback behaviour
        rospy.Subscriber(
            Topics.EMERGENCY_BRAKE.value, Bool, activateFallbackBehaviour, queue_size=10
        )

    # rospy.Subscriber(
    #     Topics.LIDAR_ENABLED.value, Bool, reconfigureMovebase, queue_size=10
    # )
    # Callbacks of move_base client
    rospy.Subscriber(
        Topics.MOVE_BASE_FEEDBACK.value,
        MoveBaseActionFeedback,
        _moveBaseActionCallback,
        queue_size=10,
    )
    rospy.Subscriber(
        Topics.MOVE_BASE_RESULT.value,
        MoveBaseActionResult,
        _moveBaseActionDoneCallback,
        queue_size=10,
    )

    # detectHazard()
    rospy.spin()


if __name__ == "__main__":
    try:
        strategyPlanner(runWithRiskEstimation=True)
    except Exception as e:
        print(e)
        rospy.loginfo(f"Shutting down node {Nodes.STRATEGY_PLANNER.value}")
