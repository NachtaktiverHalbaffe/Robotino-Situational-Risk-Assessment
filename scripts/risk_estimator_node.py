#!/usr/bin/env python3
import numpy as np
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32

from risk_estimation import config
from risk_estimation.eval import mains
from utils import evalmanager_client
from utils.constants import Topics, Nodes
from utils.risk_estimation_utils import getIntersection, visualizeCommonTraj, getCommonTraj
from utils.ros_logger import set_rospy_log_lvl
from utils.navigation_utils import pathToTraj
from risk_estimation.crash_and_remove import calculate_collosion_order, run_crash_and_remove
from prototype.msg import Risk, ProbabilityRL, ProbabilitiesRL, CriticalSector, CriticalSectors, ObstacleList
from autonomous_operation.object_detection import Obstacle

CONFIG = config.configs[0]
ENV_NAME = "robo_navigation-v01"
ATTEMPTS = 3
AMOUNT_OF_EXPLORATION = 1

useBrute = False


def estimateBaseline(globalPath: Path):
    """
    Runs the risk estimation itself.

    Args:
        globalPath (navigation_msgs.Path): The planned trajectory. Gets usually passed by the subscriber

    Returns:
        Publishes a risk message to the topic "/risk_estimation"
    """
    rospy.loginfo("[Risk Estimator] Started risk estimation with SOTA")
    # Get obstacles
    obstacleMsg = rospy.wait_for_message(Topics.OBSTACLES.value, ObstacleList)
    obstacles = []
    for obstacle in obstacleMsg.obstacles:
        # Create corners
        tmpCorners = []
        for corner in obstacle.corners:
            tmpCorners.append((corner.x, corner.y))
        # Create Obstacle
        obstacles.append(Obstacle(tmpCorners))

    done, risk = mains(
        mode=False, mcts_eval="IDA", combined_eval=False, initTraj=pathToTraj(globalPath), obstacles=obstacles
    )
    rospy.loginfo(f"[Risk Estimator] Finished risk estimation with SOTA. Risk: {risk}")

    msg = Float32
    msg.data = risk

    publisher = rospy.Publisher(Topics.RISK_ESTIMATION_SOTA.value, Float32, queue_size=10, latch=True)

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
    COST_COLLISION = 200
    COST_PRODUCTION_STOP = 100
    COST_COLLISIONSTOP = COST_COLLISION + COST_PRODUCTION_STOP

    ###################################################
    # --------------- Run probability estimator ------------------
    ###################################################
    traj = pathToTraj(globalPath)
    # Get obstacles
    obstacleMsg = rospy.wait_for_message(Topics.OBSTACLES.value, ObstacleList)
    obstacles = []
    for obstacle in obstacleMsg.obstacles:
        # Create corners
        tmpCorners = []
        for corner in obstacle.corners:
            tmpCorners.append((corner.x, corner.y))
        # Create Obstacle
        obstacles.append(Obstacle(tmpCorners))

    if useBrute:
        rospy.loginfo("[Risk Estimator] Starting Probability Estimator&Fault Injector with brute force")
        estimation = run_crash_and_remove(
            configs=CONFIG,
            env_name=ENV_NAME,
            use_brute_force_baseline=True,
            replay_on=False,
            attempts=ATTEMPTS,
            amount_of_exploration=AMOUNT_OF_EXPLORATION,
            initialTraj=traj,
            obstacles=obstacles,
        )
    else:
        rospy.loginfo("[Risk Estimator] Starting Starting Probability Estimator&Fault Injector without brute force")
        estimation = run_crash_and_remove(
            configs=CONFIG,
            env_name=ENV_NAME,
            use_brute_force_baseline=False,
            replay_on=False,
            attempts=ATTEMPTS,
            amount_of_exploration=AMOUNT_OF_EXPLORATION,
            initialTraj=traj,
            obstacles=obstacles,
        )
    rospy.loginfo("[Risk Estimator] Starting Probability Estimator&Fault Injector finished")
    rospy.logdebug(
        f"[Risk Estimator] Probability params:\nNr of iterations:{len(estimation['rl_prob'])}\nProbabilities:{estimation['rl_prob']}"
    )

    ###################################################
    # --------------------- Calculate  Risks ------------------------
    ###################################################
    # Create ros message
    riskMsg = Risk()
    riskMsg.trajectory = estimation["traj"][0]

    rospy.loginfo("[Strategy Planner] Starting risk calculation")
    riskCollision = []
    riskCollisionStop = []
    commonTraj = getCommonTraj()
    for i in range(len(estimation["rl_prob"])):
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
        riskCollision.append(np.multiply(calculate_collosion_order(estimation["rl_prob"][i]), COST_COLLISION))

        ###########################################
        # ------------ Process critical sectors --------------
        ###########################################
        # Sector: A sector is a edge inside a path => sector consists of start and end node
        # Critical sector: A sector where the probability estimator estimated a possible collision.
        # The given nodeNr is the end node of the sector
        riskCollisionStopProb = 0
        for probability in estimation["rl_prob"][i]:
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
                _, isIntersected = getIntersection(
                    [estimation["traj"][i][nodeNr - 1].x, estimation["traj"][i][nodeNr - 1].y],
                    [estimation["traj"][i][nodeNr].x, estimation["traj"][i][nodeNr].y],
                    edge[0],
                    edge[1],
                )
                if isIntersected:
                    risk = np.multiply(COST_COLLISIONSTOP, rawProb)
                    criticalSector.isIntersected = True
                    criticalSector.risk_collisionStop = risk
                    riskCollisionStopProb += risk
                    break
                else:
                    criticalSector.isIntersected = False
                    criticalSector.risk_collisionStop = 0

            criticalSector.risk = criticalSector.risk_collisionStop + criticalSector.risk_collision
            riskCollisionStop.append(riskCollisionStopProb)
            criticalSectorsMsg.sectors.append(criticalSectorsMsg)

        riskMsg.criticalSectors.append(criticalSectorsMsg)

        # Calculate global risk
        riskMsg.globalRisks = riskCollision[i] + riskCollisionStop[i]
        rospy.logdebug(
            f"[Strategy Planner] Finished global risk estimation for run {i}.\nRisk collision: {riskCollision[i]}\nRisk collision\&stop: {riskCollisionStop[i]}\nRisk combined: {riskCollision[i]+riskCollisionStop[i]}"
        )
    rospy.loginfo(
        f"[Strategy Planner] Finished global risk estimation.\nRisk collision: {np.average(riskCollision)}\nRisk collision\&stop: {np.average(riskCollisionStop)}\nRisk combined: {np.average(riskCollision)+np.average(riskCollisionStop)}"
    )
    #######################################
    # ------ Finishing&sending risk message -----
    #######################################
    # Logging in Evaluationmanager
    evalmanager_client.evalLogRisk(np.average(riskCollision) + np.average(riskCollisionStop))
    # Publish ros message
    publisherRL = rospy.Publisher(Topics.RISK_ESTIMATION_RL.value, Risk, queue_size=10, latch=True)
    try:
        publisherRL.publish(riskMsg)
    except Exception as e:
        rospy.logwarn(f"[Risk Estimator] Couldn't publish message. Exception: {e}")


def setUseBrute(enabled: Bool):
    global useBrute
    useBrute = enabled.data


def riskEstimator():
    """
    Runs the node itself. This node is responsible for estimating a risk for a given trajectory
    """
    rospy.init_node(Nodes.RISK_ESTIMATOR.value)
    set_rospy_log_lvl(rospy.DEBUG)
    rospy.loginfo(f"Starting node {Nodes.RISK_ESTIMATOR.value}")

    rospy.Subscriber(Topics.GLOBAL_PATH.value, Path, estimateRisk, queue_size=1)
    rospy.Subscriber(Topics.BRUTEFORCE_ENABLED.value, Bool, setUseBrute, queue_size=10)

    sotaSub = rospy.Subscriber(Topics.GLOBAL_PATH.value, Path, estimateBaseline, queue_size=1)
    sotaSub.unregister()
    while not rospy.is_shutdown():
        visualizeCommonTraj()
        useBaseline = rospy.wait_for_message(Topics.SOTA_ENABLED.value, Bool)
        if not useBaseline.data:
            try:
                sotaSub.unregister()
            except:
                # No Subscriber registered
                pass
        else:
            sotaSub = rospy.Subscriber(Topics.GLOBAL_PATH.value, Path, estimateBaseline, queue_size=1)
        rospy.Rate(1).sleep()

    # Prevents python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    try:
        riskEstimator()
    except Exception as e:
        print(e)
        rospy.loginfo(f"Shutting down node {Nodes.RISK_ESTIMATOR.value}")
