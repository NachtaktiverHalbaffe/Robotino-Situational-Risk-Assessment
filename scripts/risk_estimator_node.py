#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path

from utils.constants import Topics, Nodes
from utils.ros_logger import set_rospy_log_lvl
from risk_estimation.eval import mains


def estimate(globalPath: Path):
    """
    Runs the risk estimation itself
    """


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
