#!/usr/bin/env python3
import numpy as np
import pandas as pd
import rospy
import os
import logging
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, Int16, String

from risk_estimation import config
from risk_estimation.eval import mains
from utils.evalmanager_client import EvalManagerClient
from utils.constants import Topics, Nodes, Paths
from utils.risk_estimation_utils import (
    getIntersection,
    visualizeCommonTraj,
    getCommonTraj,
)
from utils.cv_utils import obstaclesMsgToObstacles
from utils.ros_logger import set_rospy_log_lvl
from utils.navigation_utils import pathToTraj
from autonomous_operation.PRM import loadWSMarkers
from risk_estimation.crash_and_remove import (
    calculate_collosion_order,
    run_crash_and_remove,
)
from prototype.msg import (
    Risk,
    ProbabilityRL,
    ProbabilitiesRL,
    CriticalSector,
    CriticalSectors,
    ObstacleList,
)

logger = logging.getLogger(__name__)
PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../", ""))

CONFIG = config.configs[0]
ENV_NAME = "robo_navigation-v01"
# This are the configurable measures how long the risk estimation will take, but also the quality
ATTEMPTS = 10
EXPAND_LENGTH = 2

# If to use bruteforce risk estimation
useBrute = False
# If to use the baseline risk estimation implemented by abdul rehman
useSOTA = False
# Known obstacles detected by obstacle detection
obstacleMsg = ObstacleList()
# The path to custom error distribution data
useCustomErrorDist = False
errorDistrDistPath = Paths.ERRORDIST_DIST.value
errorDistrAnglePath = Paths.ERRORDIST_ANGLE.value

publisherRL = rospy.Publisher(
    Topics.RISK_ESTIMATION_RL.value, Risk, queue_size=10, latch=True
)


def setUseBrute(enabled: Bool):
    global useBrute
    useBrute = enabled.data


def setUseSOTA(enabled: Bool):
    global useSOTA
    useSOTA = enabled.data


def setObstacle(obstacleList: ObstacleList):
    global obstacleMsg
    obstacleMsg = obstacleList


def setPathErrorDist(path: String):
    global errorDistrDistPath
    print(f"Set path: {path.data}")
    errorDistrDistPath = path.data


def setPathErrorAngle(path: String):
    global errorDistrAnglePath
    print(f"Set path: {path.data}")
    errorDistrAnglePath = path.data


def setUseCustomErroDist(isUsed: Bool):
    global useCustomErrorDist
    useCustomErrorDist = isUsed.data


def __getErrorDistPaths():
    global useCustomErrorDist
    global errorDistrDistPath
    global errorDistrAnglePath

    if useCustomErrorDist:
        return errorDistrDistPath, errorDistrAnglePath, False
    else:
        return None, None, True


def estimateRiskOfObjects(nrOfRuns: Int16, operationMode="commonTasks"):
    """
    It estimates how often the known obstacles are involved in collisions in the simulation. For this, multiple risk estimations are 
    run and the results are analysed in a matter how often a obstacle collidied relative to the number of total runs.

    Args:
        nrOfRuns (Int16): The number of risk estimations that should be run to get the risk of the obstacles.\
                          IMPORTANT: When operationMode "commonTasks" is used, it specifies the number of runs\
                          PER task, so each of the 6 tasks is run nrOfRuns times
        operationMode (str, optional): How the risk estimation should be run. With "commonTasks" (default), it\
                                       runs a estimation with each common used navigation task between workstation.\
                                       With "random" it generates random start & goal points each time
                          
    Returns:
    """
    global obstacleMsg
    # Parameters for probability estimator&fault injector
    AMOUNT_OF_EXPLORATION = 1

    ###################################################
    # --------------- Run probability estimator ------------------
    ###################################################
    # Get obstacles
    obstacles = obstaclesMsgToObstacles(obstacleMsg)

    errDistDistPath, errDistAnglePath, useLIDAR = __getErrorDistPaths()
    nrTotalRuns = 0
    runs = []
    rospy.loginfo(
        "[Risk Estimator] Starting Probability Estimator&Fault Injector for determining risk of objects"
    )
    if "commontasks" in operationMode.lower():
        # Dirty fix: Sometimes loading markers fails randomly => retry loading
        try:
            wsNodes = loadWSMarkers()
        except:
            wsNodes = loadWSMarkers()

        commonTasks = [
            [wsNodes[0], wsNodes[1]],
            [wsNodes[0], wsNodes[2]],
            [wsNodes[0], wsNodes[3]],
            [wsNodes[1], wsNodes[2]],
            [wsNodes[1], wsNodes[3]],
            [wsNodes[2], wsNodes[3]],
        ]
        nrTotalRuns = nrOfRuns.data * len(commonTasks) * AMOUNT_OF_EXPLORATION
        # Run probability estimator for each common task
        for task in commonTasks:
            for _ in range(nrOfRuns.data):
                estimation = run_crash_and_remove(
                    configs=CONFIG,
                    env_name=ENV_NAME,
                    use_brute_force_baseline=False,
                    replay_on=False,
                    attempts=ATTEMPTS,
                    amount_of_exploration=AMOUNT_OF_EXPLORATION,
                    expand_length=EXPAND_LENGTH,
                    obstacles=obstacles,
                    start=task[0],
                    goal=task[1],
                    invertMap=True,
                    errorDistrDistPath=errDistDistPath,
                    errorDistrAnglePath=errDistAnglePath,
                    useLidar=useLIDAR,
                )
                runs.append(estimation)
    else:
        nrTotalRuns = nrOfRuns.data * AMOUNT_OF_EXPLORATION
        # Run nrOfRuns times a probability estimation with random start- and goalpoints
        for _ in range(nrOfRuns.data):
            estimation = run_crash_and_remove(
                configs=CONFIG,
                env_name=ENV_NAME,
                use_brute_force_baseline=False,
                replay_on=False,
                attempts=ATTEMPTS,
                amount_of_exploration=AMOUNT_OF_EXPLORATION,
                expand_length=EXPAND_LENGTH,
                obstacles=obstacles,
                invertMap=True,
                errorDistrDistPath=errDistDistPath,
                errorDistrAnglePath=errDistAnglePath,
                useLidar=useLIDAR,
            )
            runs.append(estimation)
    rospy.loginfo(
        "[Risk Estimator] Finished Probability Estimator&Fault Injector for determining risk of objects"
    )

    #######################################################
    # ----- Get how often the obstacles collided ----------
    #######################################################
    # In this dict the results are saved
    criticalObstacles = {
        "box": {
            "collisions": 0,
            "risks": [],
            "relativeOccurence": 0,
        },
        "klapp": {
            "collisions": 0,
            "risks": [],
            "relativeOccurence": 0,
        },
        "hocker": {
            "collisions": 0,
            "risks": [],
            "relativeOccurence": 0,
        },
        "robotino": {
            "collisions": 0,
            "risks": [],
            "relativeOccurence": 0,
        },
        "generic": {
            "collisions": 0,
            "risks": [],
            "relativeOccurence": 0,
        },
        "ws1": {
            "collisions": 0,
            "risks": [],
            "relativeOccurence": 0,
        },
        "ws2": {
            "collisions": 0,
            "risks": [],
            "relativeOccurence": 0,
        },
        "ws3": {
            "collisions": 0,
            "risks": [],
            "relativeOccurence": 0,
        },
        "ws4": {
            "collisions": 0,
            "risks": [],
            "relativeOccurence": 0,
        },
        "ws5": {
            "collisions": 0,
            "risks": [],
            "relativeOccurence": 0,
        },
        "ws6": {
            "collisions": 0,
            "risks": [],
            "relativeOccurence": 0,
        },
    }

    for run in runs:
        # Filter out probabilities where a single collision was detected twice
        filteredProbs = []
        filteredObstacles = []

        for i in range(len(run["rl_prob"])):
            isAlreadyPresent = False
            for item in filteredProbs:
                if item[0] == run["rl_prob"][i][0]:
                    isAlreadyPresent = True
                    break
            if not isAlreadyPresent:
                filteredProbs.append(run["rl_prob"][i])
                filteredObstacles.append(run["collided_obs"][i])

        if len(filteredProbs) != 0:
            for i in range(len(filteredProbs)):
                try:
                    risk = filteredProbs[i][0]
                    for obstacle in filteredObstacles[i]:
                        criticalObstacles[obstacle.label]["collisions"] = (
                            criticalObstacles[obstacle.label]["collisions"] + 1
                        )
                        criticalObstacles[obstacle.label]["risks"].append(risk)
                except:
                    continue
    # Set collision potential in all known obstacle objects
    for obstacle in obstacles:
        obstacle.collisionPotential = (
            criticalObstacles[obstacle.label]["collisions"] / nrTotalRuns
        )

    for key in criticalObstacles.keys():
        criticalObstacles[key]["relativeOccurence"] = (
            criticalObstacles[key]["collisions"] / nrTotalRuns
        )

    # estimationObstacles = sorted(criticalObstacles.items(), key=lambda t: t[1])[-1][0]
    rospy.loginfo(
        f"[Risk Estimator] Analyzed obstacles and their potential to leading to a collison:{criticalObstacles} in {nrTotalRuns} simulations"
    )
    # Save
    df_currentRun = pd.DataFrame(criticalObstacles)
    df_oldRuns = pd.read_csv(Paths.RISK_ESTIMATION_OBSTACLES.value)
    df_save = pd.concat([df_oldRuns, df_currentRun])
    df_save.to_csv(Paths.RISK_ESTIMATION_OBSTACLES.value)
    return criticalObstacles


def estimateBaseline(globalPath: Path):
    """
    Runs the risk estimation itself.

    Args:
        globalPath (navigation_msgs.Path): The planned trajectory. Gets usually passed by the subscriber

    Returns:
        Publishes a risk message to the topic "/risk_estimation"
    """
    global obstacleMsg
    global useSOTA

    if not useSOTA:
        return
    rospy.loginfo("[Risk Estimator] Started risk estimation with SOTA")
    # Get obstacles
    obstacles = obstaclesMsgToObstacles(obstacleMsg)

    done, risk = mains(
        mode=False,
        mcts_eval="IDA",
        combined_eval=False,
        initTraj=pathToTraj(globalPath),
        obstacles=obstacles,
    )
    rospy.loginfo(f"[Risk Estimator] Finished risk estimation with SOTA. Risk: {risk}")

    msg = Float32
    msg.data = risk

    publisher = rospy.Publisher(
        Topics.RISK_ESTIMATION_SOTA.value, Float32, queue_size=10, latch=True
    )

    try:
        publisher.publish(msg)
    except Exception as e:
        rospy.logerr(f"[Risk Estimator] Couldn't publish SOTA risk message: {e}")


def estimateRisk(globalPath: Path):
    """
    Runs the risk estimation itself.

    Args:
        globalPath (navigation_msgs.Path): The planned trajectory. Gets usually passed by the subscriber

    Returns:
        Publishes a risk message to the topic "/risk_estimation"
    """
    global useBrute
    global obstacleMsg
    # Parameters for probability estimator&fault injector
    AMOUNT_OF_EXPLORATION = 1
    # Parameters for risk calculation
    COST_COLLISION = 200
    COST_PRODUCTION_STOP = 100
    COST_COLLISIONSTOP = COST_COLLISION + COST_PRODUCTION_STOP

    ###################################################
    # --------------- Run probability estimator ------------------
    ###################################################
    traj = pathToTraj(globalPath)
    # Get obstacles
    obstacles = obstaclesMsgToObstacles(obstacleMsg)
    errDistDistPath, errDistAnglePath, useLIDAR = __getErrorDistPaths()
    if useBrute:
        rospy.loginfo(
            "[Risk Estimator] Starting Probability Estimator&Fault Injector with brute force"
        )
        estimation = run_crash_and_remove(
            configs=CONFIG,
            env_name=ENV_NAME,
            use_brute_force_baseline=True,
            replay_on=False,
            attempts=ATTEMPTS,
            amount_of_exploration=AMOUNT_OF_EXPLORATION,
            initialTraj=traj,
            obstacles=obstacles,
            invertMap=True,
            expand_length=EXPAND_LENGTH,
            errorDistrDistPath=errDistDistPath,
            errorDistrAnglePath=errDistAnglePath,
            useLidar=useLIDAR,
        )
    else:
        rospy.loginfo(
            "[Risk Estimator] Starting Probability Estimator&Fault Injector without brute force"
        )
        estimation = run_crash_and_remove(
            configs=CONFIG,
            env_name=ENV_NAME,
            use_brute_force_baseline=False,
            replay_on=False,
            attempts=ATTEMPTS,
            amount_of_exploration=AMOUNT_OF_EXPLORATION,
            initialTraj=traj,
            obstacles=obstacles,
            invertMap=True,
            expand_length=EXPAND_LENGTH,
            errorDistrDistPath=errDistDistPath,
            errorDistrAnglePath=errDistAnglePath,
            useLidar=useLIDAR,
        )
    rospy.loginfo("[Risk Estimator] Probability Estimator&Fault Injector finished")
    rospy.logdebug(
        f"[Risk Estimator] Probability params:\nNr of iterations:{len(estimation['rl_prob'])}\nProbabilities:{estimation['rl_prob']}"
    )

    ###################################################
    # --------------------- Calculate  Risks ------------------------
    ###################################################
    # Create ros message
    riskMsg = Risk()
    riskMsg.trajectory = globalPath

    rospy.loginfo("[Strategy Planner] Starting risk calculation")
    riskCollision = []
    riskCollisionStop = []
    commonTraj = getCommonTraj()
    for i in range(len(estimation["rl_prob"])):
        ###########################################
        # ---------------- Process one run ------------------
        ###########################################
        # Filter out probabilities where a single collision was detected twice
        filteredProbs = []
        for prob in estimation["rl_prob"][i]:
            isAlreadyPresent = False
            for item in filteredProbs:
                if item[0] == prob[0]:
                    isAlreadyPresent = True
                    break
            if not isAlreadyPresent:
                filteredProbs.append(prob)
        print(f"Filtered Probs: {filteredProbs}")
        # The raw collision probabilities of the probability estimator are also send with the risk message if needed
        # by receiver of message
        probabilitiesMsg = ProbabilitiesRL()
        criticalSectorsMsg = CriticalSectors()
        if useBrute:
            riskMsg.probs_brute.append(estimation["brute_prob"][i])

        # Calculate risk of collision => given by probability estimator
        riskCollision.append(
            np.multiply(calculate_collosion_order(filteredProbs), COST_COLLISION)
        )

        ###########################################
        # ------------ Process critical sectors --------------
        ###########################################
        # Sector: A sector is a edge inside a path => sector consists of start and end node
        # Critical sector: A sector where the probability estimator estimated a possible collision.
        # The given nodeNr is the end node of the sector
        riskCollisionStopProb = 0
        for probability in filteredProbs:
            nodeNr = probability[1]
            rawProb = probability[0]
            # Critical sector is a custom ros message, but is always part of the custom ros message CriticalSectors
            # which is basically a list of all the Critical sectors in one run
            criticalSector = CriticalSector()
            criticalSector.node = nodeNr
            # One raw collision probability => gets appended ti
            singleProbMsg = ProbabilityRL()
            singleProbMsg.nodeIndex = nodeNr
            singleProbMsg.probability = rawProb
            probabilitiesMsg.probabilities.append(singleProbMsg)

            # Calculate risk of collision in critical sector
            criticalSector.risk_collision = np.multiply(rawProb, COST_COLLISION)

            # Calculate risk of collision and stop
            for edge in commonTraj:
                startTraj = [
                    float(globalPath.poses[nodeNr - 1].pose.position.x),
                    float(globalPath.poses[nodeNr - 1].pose.position.y),
                ]
                endTraj = [
                    float(globalPath.poses[nodeNr].pose.position.x),
                    float(globalPath.poses[nodeNr].pose.position.y),
                ]
                point, isIntersected = getIntersection(
                    startTraj,
                    endTraj,
                    edge[0],
                    edge[1],
                )
                if isIntersected:
                    print(f"Intersected at {point}")
                    risk = np.multiply(COST_COLLISIONSTOP, rawProb)
                    criticalSector.isIntersected = True
                    criticalSector.risk_collisionStop = risk
                    riskCollisionStopProb += risk
                    break
                else:
                    criticalSector.isIntersected = False
                    criticalSector.risk_collisionStop = 0

            criticalSector.risk = (
                criticalSector.risk_collisionStop + criticalSector.risk_collision
            )
            criticalSectorsMsg.sectors.append(criticalSector)

        riskCollisionStop.append(riskCollisionStopProb)
        riskMsg.criticalSectors.append(criticalSectorsMsg)
        riskMsg.probs_rl.append(probabilitiesMsg)
        # Calculate global risk
        riskMsg.globalRisks.append(riskCollision[i] + riskCollisionStop[i])
        rospy.logdebug(
            f"[Risk Estimator] Finished global risk estimation for run {i}.\nRisk collision: {riskCollision[i]}\nRisk collision&stop: {riskCollisionStop[i]}\nRisk combined: {riskCollision[i]+riskCollisionStop[i]}"
        )

    #######################################
    # ------ Finishing&sending risk message -----
    #######################################
    # Logging in Evaluationmanager
    EvalManagerClient().evalLogRisk(
        np.average(riskCollision) + np.average(riskCollisionStop)
    )
    # Publish ros message
    rospy.loginfo(
        f"[Risk Estimator] Finished global risk estimation.\nRisk collision: {np.average(riskCollision)}\nRisk collision&stop: {np.average(riskCollisionStop)}\nRisk combined: {np.average(riskCollision)+np.average(riskCollisionStop)}"
    )
    try:
        publisherRL.publish(riskMsg)
    except Exception as e:
        rospy.logwarn(f"[Risk Estimator] Couldn't publish message. Exception: {e}")


def riskEstimator():
    """
    Runs the node itself. This node is responsible for estimating a risk for a given trajectory
    """
    rospy.init_node(Nodes.RISK_ESTIMATOR.value)
    set_rospy_log_lvl(rospy.DEBUG)
    rospy.loginfo(f"Starting node {Nodes.RISK_ESTIMATOR.value}")

    # Risk estimators
    rospy.Subscriber(Topics.GLOBAL_PATH.value, Path, estimateRisk, queue_size=1)
    rospy.Subscriber(Topics.GLOBAL_PATH.value, Path, estimateBaseline, queue_size=1)
    rospy.Subscriber(
        Topics.NR_OF_RUNS.value, Int16, estimateRiskOfObjects, queue_size=1
    )
    # Selecting which additional estimator to use (beside the risk estimation used by prototype)
    rospy.Subscriber(Topics.BRUTEFORCE_ENABLED.value, Bool, setUseBrute, queue_size=10)
    rospy.Subscriber(Topics.SOTA_ENABLED.value, Bool, setUseSOTA, queue_size=10)
    # ROS "setter"
    rospy.Subscriber(Topics.OBSTACLES.value, ObstacleList, setObstacle, queue_size=10)
    rospy.Subscriber(
        Topics.PATH_ERRORDISTR_ANGLE.value, String, setPathErrorAngle, queue_size=10
    )
    rospy.Subscriber(
        Topics.PATH_ERRORDIST_DIST.value, String, setPathErrorDist, queue_size=10
    )
    rospy.Subscriber(
        Topics.USE_ERRORDIST.value, Bool, setUseCustomErroDist, queue_size=10
    )

    # while not rospy.is_shutdown():
    #     visualizeCommonTraj()
    #     rospy.Rate(1).sleep()

    # Prevents python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    try:
        riskEstimator()
    except Exception as e:
        print(e)
        rospy.loginfo(f"Shutting down node {Nodes.RISK_ESTIMATOR.value}")
