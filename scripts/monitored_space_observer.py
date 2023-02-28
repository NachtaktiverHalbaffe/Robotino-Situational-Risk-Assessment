#!/usr/bin/env python3
from real_robot_navigation.move_utils_cords import (
    get_base_info,
    get_pixel_location_from_acml,
)
import rospy
import csv
import os
import collections
import numpy as np
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseWithCovarianceStamped

from utils.constants import Topics, Nodes, Paths
from utils.ros_logger import set_rospy_log_lvl

PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../", ""))

# The path to custom error distribution data
errorDistrDistPath = Paths.ERRORDIST_DIST.value
errorDistrAnglePath = Paths.ERRORDIST_ANGLE.value

distPub = rospy.Publisher(
    Topics.PATH_ERRORDIST_DIST.value,
    String,
    queue_size=10,
)
anglePub = rospy.Publisher(Topics.PATH_ERRORDISTR_ANGLE.value, String, queue_size=10)
anomalyPub = rospy.Publisher(Topics.ANOMALY_DETECTED.value, Bool, queue_size=10)


def setPathErrorDist(path: String):
    global errorDistrDistPath
    errorDistrDistPath = path.data


def setPathErrorAngle(path: String):
    global errorDistrAnglePath
    errorDistrAnglePath = path.data


def __dumpErrorToCSV(savePath, value):
    with open(
        savePath,
        "a",
    ) as f1:
        write = csv.writer(f1)
        write.writerow([value])


def __clearStandardCSVs():
    """
    Emptys the standard error distribution files which are used in normal operation
    """
    f = open(Paths.ERRORDIST_ANGLE.value, "w+")
    f.close()
    f = open(Paths.ERRORDIST_DIST.value, "w+")
    f.close()


def _executeAnomalyMeasures(errorValue: float, anomalySource: str):
    """
    Behaviour when a anomaly is detected => set ememrgency Break and publish currently used error values

    Args:
        errorValue (float): The value of the deviation
        anomalySource (str): The name of the source where the anomaly originates from
    """
    global errorDistrDistPath
    global errorDistrAnglePath

    rospy.logwarn(
        f"[Monitored Space Observer] Anomaly in {anomalySource} detected with error {errorValue}"
    )

    try:
        # Published the used csv for error distribution so risk estimation can use them
        distPub.publish(errorDistrDistPath)
        anglePub.publish(errorDistrAnglePath)
        # Deactivate LIDAR usage
        anomalyPub.publish(True)
        # Wait until first image localization image comes in
        rospy.wait_for_message(Topics.LOCALIZATION.value, PoseWithCovarianceStamped)
    except Exception as e:
        rospy.logerr(
            f"[Monitored Space Observer] Couldn't execute anomaly behaviour: {e}"
        )


def spaceObserver(savePath: str = f"{PATH}/logs/error_dist_csvs/localization_error"):
    """
    Tracks/saves the error distribution between localization with LIDAR (amcl) and camera

    Args:
    """
    global errorDistrDistPath
    global errorDistrAnglePath
    FIFO_LENGTH = 10
    THRES_X = 1.5
    THRES_Y = 1.5
    THRES_DIST = 1.5
    THRES_ANGLE = np.pi / 4

    xErrFIFO = collections.deque(FIFO_LENGTH * [0], FIFO_LENGTH)
    yErrFIFO = collections.deque(FIFO_LENGTH * [0], FIFO_LENGTH)
    distErrFIFO = collections.deque(FIFO_LENGTH * [0], FIFO_LENGTH)
    angleErrFIFO = collections.deque(FIFO_LENGTH * [0], FIFO_LENGTH)
    imageLoc = None
    amclPose = None
    while not rospy.is_shutdown():
        # anomalyPub.publish(False)
        try:
            imageLoc = rospy.wait_for_message(
                Topics.IMG_LOCALIZATION.value,
                PoseWithCovarianceStamped,
                timeout=rospy.Duration(secs=2),
            )
        except:
            # Timout
            if imageLoc == None:
                continue
        try:
            amclPose = rospy.wait_for_message(
                Topics.ACML.value,
                PoseWithCovarianceStamped,
                timeout=rospy.Duration(secs=2),
            )
        except:
            if amclPose == None:
                continue

        # Calculating error in localization in x,y, distance and angle
        xErr = float(amclPose.pose.pose.position.x - imageLoc.pose.pose.position.x)
        yErr = float(amclPose.pose.pose.position.y - imageLoc.pose.pose.position.y)

        distErr = float(np.sqrt(np.square(xErr) + np.square(yErr)))
        if xErr > 0:
            distErr = -1 * distErr
        angleErr = float(
            amclPose.pose.pose.orientation.z - imageLoc.pose.pose.orientation.z
        )

        # Dump raw error values to a csv file
        __dumpErrorToCSV(f"{savePath}_x.csv", xErr)
        __dumpErrorToCSV(f"{savePath}_y.csv", yErr)
        __dumpErrorToCSV(errorDistrDistPath, distErr)
        __dumpErrorToCSV(errorDistrAnglePath, angleErr)

        # Update fifo queue
        xErrFIFO.appendleft(xErr)
        yErrFIFO.appendleft(yErr)
        distErrFIFO.appendleft(distErr)
        angleErrFIFO.appendleft(angleErr)
        # Get median of these que's to determine anomaly
        xErrMedian = np.median(xErrFIFO)
        yErrMedian = np.median(yErrFIFO)
        distErrMedian = np.median(distErrFIFO)
        angleErrMedian = np.median(angleErrFIFO)

        # Anomaly detection
        # if np.abs(xErrMedian) > THRES_X:
        #     _executeAnomalyMeasures(xErrMedian, "x-space")
        # if np.abs(yErrMedian) > THRES_Y:
        #     _executeAnomalyMeasures(yErrMedian, "y-space")
        if np.abs(distErrMedian) > THRES_DIST:
            _executeAnomalyMeasures(distErrMedian, "distance-space")
            return
        if np.abs(angleErrMedian) > THRES_ANGLE:
            _executeAnomalyMeasures(angleErrMedian, "angle-space")
            return


def monitorSpace():
    """
    Runs the node itself. This node is responsible for tracking the deviation of measurements i.e. in this case\
    the difference between the localization with LIDAR (AMCL) and camera based localization
    """
    rospy.init_node(Nodes.MONITORED_SPACE_OBSERVER.value)
    set_rospy_log_lvl(rospy.INFO)
    rospy.loginfo(f"Starting node {Nodes.MONITORED_SPACE_OBSERVER.value}")

    rospy.Subscriber(
        Topics.PATH_ERRORDISTR_ANGLE.value, String, setPathErrorAngle, queue_size=10
    )
    rospy.Subscriber(
        Topics.PATH_ERRORDIST_DIST.value, String, setPathErrorDist, queue_size=10
    )

    __clearStandardCSVs()

    spaceObserver()


if __name__ == "__main__":
    try:
        monitorSpace()
    except:
        rospy.loginfo(f"Shutdown node {Nodes.MONITORED_SPACE_OBSERVER.value}")
