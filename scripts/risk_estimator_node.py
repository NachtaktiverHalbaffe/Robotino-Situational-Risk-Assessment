#!/usr/bin/env python3
import json
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
    CriticalObjects,
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

INIT_CRITICALOBS = {
    "box": {
        "collisions": 0,
        "risks": [],
        "averageRisk": 0,
        "relativeOccurence": 0,
        "obstacleRisk": 0,
    },
    "klapp": {
        "collisions": 0,
        "risks": [],
        "averageRisk": 0,
        "relativeOccurence": 0,
        "obstacleRisk": 0,
    },
    "hocker": {
        "collisions": 0,
        "risks": [],
        "averageRisk": 0,
        "relativeOccurence": 0,
        "obstacleRisk": 0,
    },
    "robotino": {
        "collisions": 0,
        "risks": [],
        "averageRisk": 0,
        "relativeOccurence": 0,
        "obstacleRisk": 0,
    },
    "generic": {
        "collisions": 0,
        "risks": [],
        "averageRisk": 0,
        "relativeOccurence": 0,
        "obstacleRisk": 0,
    },
    "geofenced": {
        "collisions": 0,
        "risks": [],
        "averageRisk": 0,
        "relativeOccurence": 0,
        "obstacleRisk": 0,
    },
    "ws1": {
        "collisions": 0,
        "risks": [],
        "averageRisk": 0,
        "relativeOccurence": 0,
        "obstacleRisk": 0,
    },
    "ws2": {
        "collisions": 0,
        "risks": [],
        "averageRisk": 0,
        "relativeOccurence": 0,
        "obstacleRisk": 0,
    },
    "ws3": {
        "collisions": 0,
        "risks": [],
        "averageRisk": 0,
        "relativeOccurence": 0,
        "obstacleRisk": 0,
    },
    "ws4": {
        "collisions": 0,
        "risks": [],
        "averageRisk": 0,
        "relativeOccurence": 0,
        "obstacleRisk": 0,
    },
    "ws5": {
        "collisions": 0,
        "risks": [],
        "averageRisk": 0,
        "relativeOccurence": 0,
        "obstacleRisk": 0,
    },
    "ws6": {
        "collisions": 0,
        "risks": [],
        "averageRisk": 0,
        "relativeOccurence": 0,
        "obstacleRisk": 0,
    },
}

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
obstacleMargin = 0

publisherRL = rospy.Publisher(
    Topics.RISK_ESTIMATION_RL.value,
    Risk,
    queue_size=10,  # latch=True
)
publisherObstacleRisk = rospy.Publisher(
    Topics.CRITICAL_OBJECTS.value, CriticalObjects, queue_size=10
)


def setObstacleMargin(margin: Int16):
    global obstacleMargin
    obstacleMargin = margin.data


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


def __markObstaclesWithCollisionRisk(
    obstacleRiskDict: dict, markInRviz: bool = True, markInBB: bool = True
):
    rvizMsg = CriticalObjects()
    bbData = dict()
    for key in obstacleRiskDict.keys():
        entry = obstacleRiskDict[key]
        if entry["collisions"] > 0:
            # Obstacle has a risk of collision
            # -- Mark so Rviz visualizes it --
            if markInRviz:
                rvizMsg.labels.append(key)
                rvizMsg.averageRisks.append(entry["averageRisk"])
                rvizMsg.relativeOccurence.append(entry["relativeOccurence"])
                rvizMsg.obstacleRisk.append(entry["obstacleRisk"])
            # -- Mark so bounding boxes in object detection gets marked --
            if markInBB:
                bbData[key] = {
                    "averageRisk": entry["averageRisk"],
                    "relativeOccurence": entry["relativeOccurence"],
                    "obstacleRisk": entry["obstacleRisk"],
                }

    if markInBB:
        print(f"{PATH}/maps/criticalObjects.json")
        with open(f"{PATH}/maps/criticalObjects.json", "w") as jsonfile:
            json.dump(bbData, jsonfile, sort_keys=True, indent=4)
    if markInRviz:
        try:
            publisherObstacleRisk.publish(rvizMsg)
        except Exception as e:
            rospy.logerr(f"[Risk Estimator] Couldn't publish critical objects: {e}")


def __getUniqueCollisons(
    estimationDict: dict, initialCriticalObjects: dict = None, nrTotalRuns: int = 1
):
    """
    In the current probability estimator and fault injector collisions can be detected and so calculated twice.\
    This function filters all collisions so each collision is unique and creates a dict with the obstacles with which\
    a collision happend.

    Args:
        estimationDict (dict): A single estimation from the probability estimator& Fault injector (crash_remove.py)
        initialCriticalObjects (dict, optional): A initial statistic of obstacles and how often they where involved in a collision.\
                                                 If given, the new collisions get added to this statistic
        nrTotalRuns (int): Number of total runs in the entire estimation. Needed to calculated the relativeOccurence of collisions of obstacles

    Returns:
        filteredProbs (list of tuple): The filtered collision probabilities. Tuple schema is as follows: (probability, node number where collision happend)
        criticalObstacles (dict): A statistic which obstacles where involved in a collision
    """
    # In this dict the results are saved
    if initialCriticalObjects == None:
        criticalObstacles = INIT_CRITICALOBS
    else:
        criticalObstacles = initialCriticalObjects

    # Filter out probabilities where a single collision was detected twice
    filteredProbs = []
    filteredObstacles = []
    # Run through one run
    for i in range(len(estimationDict["rl_prob"])):
        # Skip iteration if no collision happend
        if len(estimationDict["rl_prob"][i]) == 0:
            continue
        singleExplorationProbs = []
        singleExplorationObs = []
        # Iterate through all collisions
        for j in range(len(estimationDict["collided_obs"][i])):
            isAlreadyPresent = False
            # Iterate through already discovered collisions
            for k in range(len(singleExplorationProbs)):
                if (
                    singleExplorationObs[k].label
                    == estimationDict["collided_obs"][i][j].label
                    or singleExplorationProbs[k][0]
                    == estimationDict["rl_prob"][i][j][0]
                    or singleExplorationProbs[k][1]
                    == estimationDict["rl_prob"][i][j][1]
                ):
                    isAlreadyPresent = True
                    break
            # Add collision if it's not already present
            if not isAlreadyPresent:
                singleExplorationProbs.append(estimationDict["rl_prob"][i][j])
                singleExplorationObs.append(estimationDict["collided_obs"][i][j])

        filteredProbs.append(singleExplorationProbs)
        filteredObstacles.append(singleExplorationObs)

    # Count number of collisions with each obstacle and write metrics into criticalObstacles
    # Iterate through all iterations
    for i in range(len(filteredProbs)):
        # Iterate through one exploration
        for j in range(len(filteredProbs[i])):
            risk = filteredProbs[i][j]
            # Iterate through all collisions in this one exploration
            for obstacle in filteredObstacles[i]:
                criticalObstacles[obstacle.label]["collisions"] = (
                    criticalObstacles[obstacle.label]["collisions"] + 1
                )
                criticalObstacles[obstacle.label]["risks"].append(risk)

    for key in criticalObstacles.keys():
        # Calculate relativeOccurence of collisions (only meaningful if nrTotalRuns != 0)
        criticalObstacles[key]["relativeOccurence"] = (
            criticalObstacles[key]["collisions"] / nrTotalRuns
        )
        # Calculate average collision probability
        if len(criticalObstacles[key]["risks"]) != 0:
            cummRisk = 0
            for probability in criticalObstacles[key]["risks"]:
                cummRisk = cummRisk + probability[0]
            criticalObstacles[key]["averageRisk"] = cummRisk / len(
                criticalObstacles[key]["risks"]
            )

        # calculate risk which originates from obstacle
        criticalObstacles[key]["obstacleRisk"] = (
            criticalObstacles[key]["averageRisk"]
            * criticalObstacles[key]["relativeOccurence"]
        )

    return filteredProbs, criticalObstacles


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
            # Now inverse direction
            [wsNodes[1], wsNodes[0]],
            [wsNodes[2], wsNodes[0]],
            [wsNodes[3], wsNodes[0]],
            [wsNodes[2], wsNodes[1]],
            [wsNodes[2], wsNodes[1]],
            [wsNodes[3], wsNodes[2]],
        ]
        nrTotalRuns = nrOfRuns.data * len(commonTasks) * AMOUNT_OF_EXPLORATION
        # Run probability estimator for each common task
        for task in commonTasks:
            for _ in range(nrOfRuns.data):
                try:
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
                except:
                    continue
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
    criticalObstacles = INIT_CRITICALOBS
    for run in runs:
        _, criticalObstacles = __getUniqueCollisons(run, criticalObstacles, nrTotalRuns)

    # estimationObstacles = sorted(criticalObstacles.items(), key=lambda t: t[1])[-1][0]
    rospy.loginfo(
        f"[Risk Estimator] Analyzed obstacles and their potential to leading to a collison:{criticalObstacles} in {nrTotalRuns} simulations"
    )
    # Save

    df_currentRun = pd.DataFrame(criticalObstacles)
    try:
        df_oldRuns = pd.read_csv(Paths.RISK_ESTIMATION_OBSTACLES.value)
        df_oldRuns.append(df_currentRun).to_csv(
            Paths.RISK_ESTIMATION_OBSTACLES.value, index=False
        )
    except:
        df_currentRun.to_csv(Paths.RISK_ESTIMATION_OBSTACLES.value, index=False)

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
    global obstacleMargin
    # Parameters for probability estimator&fault injector
    AMOUNT_OF_EXPLORATION = 1
    # Parameters for risk calculation
    COST_COLLISION = 200
    COST_PRODUCTION_STOP = 100
    COST_COLLISIONSTOP = COST_COLLISION + COST_PRODUCTION_STOP
    # Quantification error we assume on the risk estimation
    QUANTIFIED_ESTIMATION_ERROR = 0.08

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
            obstacleMargin=obstacleMargin,
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
            obstacleMargin=obstacleMargin,
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

    # Filter out probabilities where a single collision was detected twice
    filteredProbs, criticalObs = __getUniqueCollisons(estimation)
    if len(filteredProbs) == 0:
        riskCollision.append(COST_COLLISION * QUANTIFIED_ESTIMATION_ERROR)
        riskCollisionStop.append(COST_COLLISIONSTOP * QUANTIFIED_ESTIMATION_ERROR)

    for i in range(len(filteredProbs)):
        ###########################################
        # ---------------- Process one run ------------------
        ###########################################

        # The raw collision probabilities of the probability estimator are also send with the risk message if needed
        # by receiver of message
        probabilitiesMsg = ProbabilitiesRL()
        criticalSectorsMsg = CriticalSectors()
        if useBrute:
            riskMsg.probs_brute.append(estimation["brute_prob"][i])

        # Calculate risk of collision => given by probability estimator
        riskCollision.append(
            np.multiply(calculate_collosion_order(filteredProbs[i]), COST_COLLISION)
        )

        ###########################################
        # ------------ Process critical sectors --------------
        ###########################################
        # Sector: A sector is a edge inside a path => sector consists of start and end node
        # Critical sector: A sector where the probability estimator estimated a possible collision.
        # The given nodeNr is the end node of the sector
        riskCollisionStopProb = 0
        collStopProb = []
        for probability in filteredProbs[i]:
            # Skip risk estimation if no collision happened

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
                    # print(f"Intersected at {point}")
                    collStopProb.append(probability)
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

        riskCollisionStop.append(
            calculate_collosion_order(collStopProb) * COST_COLLISIONSTOP
        )
        # riskCollisionStop.append(
        #     riskCollisionStopProb + COST_COLLISIONSTOP * QUANTIFIED_DEVIATION
        # )
        riskMsg.criticalSectors.append(criticalSectorsMsg)
        riskMsg.probs_rl.append(probabilitiesMsg)
        # Calculate global risk
        riskMsg.globalRisks.append(
            (riskCollision[i] + riskCollisionStop[i])
            + (riskCollision[i] + riskCollisionStop[i]) * QUANTIFIED_ESTIMATION_ERROR
        )
        # rospy.logdebug(
        #     f"[Risk Estimator] Finished global risk estimation for run {i}.\nRisk collision: {riskCollision[i]}\nRisk collision&stop: {riskCollisionStop[i]}\nRisk combined: {riskCollision[i]+riskCollisionStop[i]}"
        # )

    #######################################
    # ------ Finishing&sending risk message -----
    #######################################
    # Logging in Evaluationmanager
    # Publish ros message
    rospy.loginfo(
        f"[Risk Estimator] Finished global risk estimation with risk.\nRisk collision: {np.average(riskCollision)}\nRisk collision&stop: {np.average(riskCollisionStop)}\nRisk combined: {np.average(riskCollision)+np.average(riskCollisionStop)}"
    )
    try:
        publisherRL.publish(riskMsg)
        __markObstaclesWithCollisionRisk(criticalObs)
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
    rospy.Subscriber(
        Topics.OBSTACLE_MARGIN.value, Int16, setObstacleMargin, queue_size=10
    )
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
