#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from prototype.msg import Risk
from geometry_msgs.msg import PoseStamped

from risk_estimation import config
from utils.constants import Topics, Nodes
from utils.ros_logger import set_rospy_log_lvl
from risk_estimation.crash_and_remove import run_crash_and_remove

CONFIG = config.configs[0]
ENV_NAME = "robo_navigation-v01"


def estimate(globalPath: Path):
    """
    Runs the risk estimation itself
    """
    traj = []
    for nodes in globalPath.poses:
        x = nodes.pose.position.x
        y = nodes.pose.position.y
        traj.append((x, y))

    rospy.loginfo("[Risk Estimator] Starting risk estimation")
    estimation = run_crash_and_remove(
        configs=CONFIG,
        env_name=ENV_NAME,
        use_brute_force_baseline=False,
        replay_on=False,
        load_location=False,
        attempts=3,
        amount_of_exploration=200,
        initialTraj=traj,
    )

    rospy.loginfo("[Risk Estimator] Finished risk estimation")
    publisher = rospy.Publisher(Topics.RISK_ESTIMATION.value, queue_size=10)

    msg = Risk()
    for i in range(len(estimation["rl_prob"])):
        msg.probs_rl.append(estimation["rl_prob"][i])
        msg.probs_brute.append(estimation["brute_prob"][i])
        msg.nr_nodes.append(estimation["len_traj"][i])
        msg.length_traj.append(estimation["len_traj_list_in_m"][i])
        msg.wall_discards.append(estimation["wall_discards"][i])
        # Trajectory has to be converted to ros path
        pathMsg = Path()
        for node in estimation["traj"][i]:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = node[0]
            pose.pose.position.y = node[1]
            pose.pose.position.z = 0
            pathMsg.poses.append(pose)
        msg.trajectories.append(pathMsg)

    try:
        publisher.publish(msg)
    except Exception as e:
        rospy.logwarn(f"[Risk Estimator] Couldn't publish message. Exception: {e}")


def riskEstimator():
    """
    Runs the node itself
    """
    rospy.init_node(Nodes.RISK_ESTIMATOR.value)
    set_rospy_log_lvl(rospy.INFO)
    rospy.loginfo(f"Starting node {Nodes.RISK_ESTIMATOR.value}")

    rospy.Subscriber(Topics.GLOBAL_PATH.value, Path, estimate, queue_size=10)

    # Prevents python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    try:
        riskEstimator()
    except:
        rospy.loginfo(f"Shutting down node {Nodes.RISK_ESTIMATOR.value}")
