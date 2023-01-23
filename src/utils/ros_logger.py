import rospy
import logging


def set_rospy_log_lvl(log_level):
    logger = logging.getLogger("rosout")
    logger.setLevel(rospy.impl.rosout._rospy_to_logging_levels[log_level])
