#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from prototype.msg import Risk, ProbabilityRL, ProbabilitiesRL
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32
from gym.envs.registration import register

from risk_estimation import config
from risk_estimation.eval import mains
from utils.constants import Topics, Nodes
from utils.ros_logger import set_rospy_log_lvl
from utils.navigation_utils import trajToPath, pathToTraj
from risk_estimation.crash_and_remove import run_crash_and_remove

CONFIG = config.configs[0]
ENV_NAME = "robo_navigation-v01"

useBrute = False

kwargs_robo = {"config": {}}
register(
    id="robo_navigation-single_dimension-v0",
    entry_point="risk_estimation.env_gym.envs.robo_navigation:RoboEnv_gym_2",
    kwargs=kwargs_robo,
    max_episode_steps=1000,
    reward_threshold=2000000,
)


def estimateBaseline(globalPath: Path):
    """
    Runs the risk estimation itself.

    Args:
        globalPath (navigation_msgs.Path): The planned trajectory. Gets usually passed by the subscriber

    Returns:
        Publishes a risk message to the topic "/risk_estimation"
    """
    rospy.loginfo("[Risk Estimator] Started risk estimation with SOTA")
    done, risk = mains(mode=False, mcts_eval="IDA", combined_eval=False, initTraj=pathToTraj(globalPath))
    rospy.loginfo(f"[Risk Estimator] Finished risk estimation with SOTA. Risk: {risk}")

    msg = Float32
    msg.data = risk

    publisher = rospy.Publisher(Topics.RISK_ESTIMATION_SOTA.value, Float32, queue_size=10, latch=True)

    try:
        publisher.publish(msg)
    except Exception as e:
        rospy.logerr(f"[Risk Estimator] Couldn't publish SOTA risk message: {e}")


def estimate(globalPath: Path):
    """
    Runs the risk estimation itself.

    Args:
        globalPath (navigation_msgs.Path): The planned trajectory. Gets usually passed by the subscriber

    Returns:
        Publishes a risk message to the topic "/risk_estimation"
    """
    global useBrute
    rospy.loginfo("[Risk Estimator] Starting risk estimation")

    traj = pathToTraj(globalPath)
    if useBrute:
        estimation = run_crash_and_remove(
            configs=CONFIG,
            env_name=ENV_NAME,
            use_brute_force_baseline=True,
            replay_on=False,
            attempts=3,
            amount_of_exploration=200,
            initialTraj=traj,
        )
    else:
        estimation = run_crash_and_remove(
            configs=CONFIG,
            env_name=ENV_NAME,
            use_brute_force_baseline=False,
            replay_on=False,
            attempts=3,
            amount_of_exploration=200,
            initialTraj=traj,
        )
    rospy.loginfo("[Risk Estimator] Finished risk estimation")
    rospy.logdebug(
        f"[Risk Estimator] Probability params:\nNr of iterations:{len(estimation['rl_prob'])}\nProbabilities:{estimation['rl_prob']}"
    )

    # Create ros message
    msgRlRisk = Risk()
    msgBruteRisk = Risk()
    for i in range(len(estimation["rl_prob"])):
        if len(estimation["rl_prob"][i]) != 0:
            probsRl = ProbabilitiesRL()
            for prob in estimation["rl_prob"][i]:
                probRl = ProbabilityRL()
                probRl.probability = prob[0]
                probRl.nodeIndex = prob[1]
                probsRl.probabilities.append(probRl)
            msgRlRisk.probs_rl.append(probsRl)
            msgRlRisk.nr_nodes.append(estimation["len_traj"][i])
            msgRlRisk.length_traj.append(estimation["len_traj_list_in_m"][i])
            msgRlRisk.wall_discards.append(estimation["wall_discards"][i])
            msgRlRisk.trajectories.append(trajToPath(estimation["traj"][i]))

            if useBrute:
                msgBruteRisk.probs_brute.append(estimation["brute_prob"][i])
                msgBruteRisk.nr_nodes.append(estimation["len_traj"][i])
                msgBruteRisk.length_traj.append(estimation["len_traj_list_in_m"][i])
                msgBruteRisk.wall_discards.append(estimation["wall_discards"][i])
                msgBruteRisk.trajectories.append(trajToPath(estimation["traj"][i]))

    # Publish ros message
    publisherRL = rospy.Publisher(Topics.RISK_ESTIMATION_RL.value, queue_size=10, latch=True)
    publisherBrute = rospy.Publisher(Topics.RISK_ESTIMATION_BRUTE.value, queue_size=10, latch=True)
    try:
        publisherRL.publish(msgRlRisk)
        if useBrute:
            publisherBrute.publish(msgBruteRisk)
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

    rospy.Subscriber(Topics.GLOBAL_PATH.value, Path, estimate, queue_size=1)
    rospy.Subscriber(Topics.BRUTEFORCE_ENABLED.value, Bool, setUseBrute, queue_size=10)

    sotaSub = rospy.Subscriber(Topics.GLOBAL_PATH.value, Path, estimateBaseline, queue_size=1)
    sotaSub.unregister()
    while not rospy.is_shutdown():
        useBaseline = rospy.wait_for_message(Topics.SOTA_ENABLED.value, Bool)
        if not useBaseline.data:
            try:
                sotaSub.unregister()
            except:
                # No Subscriber registered
                pass
        else:
            sotaSub = rospy.Subscriber(Topics.GLOBAL_PATH.value, Path, estimateBaseline, queue_size=1)

    # Prevents python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    try:
        riskEstimator()
    except:
        rospy.loginfo(f"Shutting down node {Nodes.RISK_ESTIMATOR.value}")
