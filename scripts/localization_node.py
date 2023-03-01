#!/usr/bin/env python3
import collections
import csv
import time
import rospy
import os
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from tf.transformations import (
    quaternion_from_euler,
    euler_from_quaternion,
    quaternion_multiply,
)
from copy import deepcopy
from cv_bridge import CvBridge
from threading import Thread

from utils.constants import Topics, Nodes, Paths
from utils.ros_logger import set_rospy_log_lvl
from yolov7.detect_online import loaded_detect
from real_robot_navigation.move_utils import *
from real_robot_navigation.move_utils_cords import *
from utils.cv_utils import (
    get_obstacles_from_detection,
    best_match_workstation_index,
    initCV,
)


PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../", ""))
PRECISION = 3

locPublisher = rospy.Publisher(
    Topics.LOCALIZATION.value, PoseWithCovarianceStamped, queue_size=10, latch=True
)
cvPublisher = rospy.Publisher(
    Topics.IMG_LOCALIZATION.value, PoseWithCovarianceStamped, queue_size=10
)
fallbackPosePub = rospy.Publisher(
    Topics.FALLBACK_POSE.value, PoseWithCovarianceStamped, queue_size=10
)
amclPub = rospy.Publisher(Topics.ACML.value, PoseWithCovarianceStamped, queue_size=10)

img_glob = []
odom = Odometry()
lidarBreakdown = False
injectOffsetEnabled = False
fallbackPose = None
real_data = ["real_data", 0, 0, 0]


def setOdom(newOdom: Odometry):
    global odom
    odom = newOdom


def setInjectOffset(enabled: Bool):
    global injectOffsetEnabled
    injectOffsetEnabled = enabled.data


def setImage(rawImage: Image):
    global img_glob
    global Image_data
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(rawImage, "rgb8")
    # Resize Image to 640 * 480 - YOLO was trained in this size
    width = int(rawImage.width * 0.80)
    height = int(rawImage.height * 0.80)
    dim = (width, height)
    img_resized = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)

    # The unmodified image
    img_glob = deepcopy(cv_image)
    Image_data = [
        "Image_Data",
        dim,  # dimensions of resized image
        img_resized,  # image data
        rawImage.header.stamp,
    ]


def setRealData(acmlData: PoseWithCovarianceStamped):
    global real_data
    real_data = [
        "real_data",
        acmlData.pose.pose.position.x,
        acmlData.pose.pose.position.y,
        acmlData.pose.pose.orientation.z,  # this will be a value e[-1, 1] and can be converted [-pi, pi] with angle=arcsin(z)*2
        acmlData.header.stamp,
    ]


def setUseLidar(isUsed: Bool):
    global lidarBreakdown
    global real_data
    global fallbackPose

    lidarBreakdown = isUsed.data

    if lidarBreakdown:
        print("ffssdfsdf")
        rospy.logdebug_throttle(10, "[Localization] Use LIDAR")
        # Use last LIDAR Data as fallback pose because it is
        # assumed that a lidar breakdown happens instantly and so last lidar data should be okay
        # poseMsg = PoseWithCovarianceStamped()
        # poseMsg.pose.pose.position.x = real_data[1]
        # poseMsg.pose.pose.position.y = real_data[2]
        # poseMsg.pose.pose.orientation.z = real_data[3]
        # poseMsg.pose.pose.orientation.w = 1
        poseMsg = rospy.wait_for_message(
            Topics.AMCL_SOURCE.value, PoseWithCovarianceStamped
        )
        fallbackPose = poseMsg
        fallbackPosePub.publish(poseMsg)

    else:
        rospy.logdebug_throttle(10, "[Localization] Use camera")


def createOdom(currentPose: PoseWithCovarianceStamped):
    """
    It creates a odometry message where the pose is taken from the localization (either amcl or computer vision bases localization
    """
    odomMsg = Odometry()
    odomMsg.header.frame_id = "map"
    odomMsg.pose.pose.position = currentPose.pose.pose.position
    odomMsg.pose.pose.orientation = currentPose.pose.pose.orientation

    # twistMsg = rospy.wait_for_message(Topics.ODOM.value, Twist)
    odomMsg.twist = odom.twist

    publisher = rospy.Publisher(Topics.ODOM.value, Odometry, queue_size=10)
    try:
        publisher.publish(odomMsg)
    except Exception as e:
        rospy.logwarn(f"[Localization] Couldn't publish odometry: {e}")


def injectOffset():
    global injectOffsetEnabled

    while not rospy.is_shutdown():
        amclMsg = PoseWithCovarianceStamped()
        amclSourceMsg = rospy.wait_for_message(
            Topics.AMCL_SOURCE.value, PoseWithCovarianceStamped
        )

        if injectOffsetEnabled:
            amclMsg.pose.pose.position.x = amclSourceMsg.pose.pose.position.x + 2
            amclMsg.pose.pose.position.x = amclSourceMsg.pose.pose.position.x + 2
        else:
            amclMsg = amclSourceMsg

        amclPub.publish(amclMsg)


def anomalyBehaviour(anomalyDetected: Bool, useAMCLasFAllback=True):
    global fallbackPose
    global lidarBreakdown

    if anomalyDetected.data:
        if not useAMCLasFAllback:
            print("Use image fallback")
            # Use next image localization data as fallback pose to start behaviour
            fallbackPose = rospy.wait_for_message(
                Topics.IMG_LOCALIZATION.value, PoseWithCovarianceStamped
            )
        else:
            print("Use amcl fallback")
            poseMsg = PoseWithCovarianceStamped()
            poseMsg.pose.pose.position.x = real_data[1]
            poseMsg.pose.pose.position.y = real_data[2]
            poseMsg.pose.pose.orientation.z = real_data[3]
            poseMsg.pose.pose.orientation.w = 1
            fallbackPose = poseMsg
        fallbackPosePub.publish(fallbackPose)
        lidarBreakdown = True


def localiseCam():
    """
    Uses the camera to localise the Robotino on the map. Uses the camera based\
    localization from Kai Binder and is only integrated here.
    """
    # For localization
    global img_glob
    global real_data
    global config
    # For interpolating position if camera based localization isn't possible
    dc_obstacles_ws = deepcopy(config["obstacles_ws"])
    # corners of the workstations on the map
    corners_map_all = [obstacle.corners for obstacle in dc_obstacles_ws]

    FIFO_LENGTH = 2
    xLocFIFO = collections.deque(FIFO_LENGTH * [0], FIFO_LENGTH)
    yLocFIFO = collections.deque(FIFO_LENGTH * [0], FIFO_LENGTH)
    angleLocFIFO = collections.deque(FIFO_LENGTH * [0], FIFO_LENGTH)
    last_known_loc = ["init_data", 0, 0, 0]
    while not rospy.is_shutdown():
        # rotation of the workstations in the gridmap TODO remove this line and get it into the obstables?
        rots_ws = [0, 0, 0.85, 1.57, 2.19, 2.19]
        # last_known_locs and the map TODO not sure if deepopy ws_map is needed
        if len(img_glob) != 0:
            img_local = deepcopy(img_glob)
        else:
            continue

        # Use last known location as location assumption if at least it located once with camera-based localization, otherwise use amcl data as initial pose
        if not lidarBreakdown:
            if len(real_data) != 0:
                location_assumption = deepcopy(real_data)
                location_assumption = offset_to_robo(location_assumption)
            else:
                continue
        else:
            location_assumption = deepcopy(last_known_loc)
            location_assumption = offset_to_robo(last_known_loc)

        # getting the detection based on the newest image
        localise_dict = loaded_detect(
            img_local, *config["conf_network"], node="localization"
        )
        # if there was a detection
        if localise_dict:
            detected_workstation_dist, rotation_detected = (
                localise_dict["birds_eye"],
                localise_dict["rotation"],
            )

            # turning the detected corners into an obstacle
            detected_obst = get_obstacles_from_detection(
                detected_workstation_dist,
                location_assumption,
                config["base_info"],
                label=localise_dict["label"],
            )
            # comparing detected obstacles with workstations on map to find correct one
            detection_corners = list(map(tuple, zip(*detected_workstation_dist)))
            index_smallest_dist_ws = best_match_workstation_index(
                corners_map_all,
                detected_obst,
                detection_corners,
                rots_ws,
                location_assumption,
                rotation_detected,
                config["base_info"],
                config["map_config"],
            )

            # calculating rotation from detected and cord transforms
            detected_rotation = (
                -(
                    rots_ws[index_smallest_dist_ws]
                    - rotation_detected
                    + np.pi
                    - config["map_config"]["rot"]
                )
                + 2 * np.pi
            )
            # basicly just transpose for the list
            corners_detec_2 = list(map(tuple, zip(*detected_workstation_dist)))
            # we get the detected localisation be subtracting the detected distances of a workstation from the actual one
            # NOTE The -3 means we only go through once, all should be the same, useful for debug
            for i in range(len(corners_detec_2) - 3):
                # change cords so we have the distance from the ws on the map
                corner = convert_cam_to_robo(
                    corners_detec_2[i][1], -corners_detec_2[i][0], detected_rotation
                )
                # change cords
                ws_map = get_amcl_from_pixel_location(
                    *corners_map_all[index_smallest_dist_ws][i], *config["base_info"]
                )
                # subtracting the detected distance from the workstation on the map
                loc_detec = (ws_map[0] - corner[0], ws_map[1] - corner[1])

                last_known_loc = [
                    "cv_data",
                    round(float(loc_detec[0]), PRECISION),
                    round(float(loc_detec[1]), PRECISION),
                    round(float(detected_rotation), PRECISION),
                ]

            # Create message to publish
            locMsg = PoseWithCovarianceStamped()
            locMsg.header.frame_id = "map"
            # Position
            locMsg.pose.pose.position.x = loc_detec[0]
            locMsg.pose.pose.position.y = loc_detec[1]
            locMsg.pose.pose.position.z = 0
            # Orientation
            # quaternion = quaternion_from_euler(0, 0, np.sin(detected_rotation / 2))
            quaternion = quaternion_from_euler(0, 0, detected_rotation)
            locMsg.pose.pose.orientation.x = quaternion[0]
            locMsg.pose.pose.orientation.y = quaternion[1]
            locMsg.pose.pose.orientation.z = quaternion[2]
            locMsg.pose.pose.orientation.w = quaternion[3]
            # Publish message
            try:
                rospy.logdebug(
                    f"Publishing camera-based localization: x:{loc_detec[0]} y:{loc_detec[0]} yaw:{np.sin(detected_rotation / 2)}"
                )
                cvPublisher.publish(locMsg)
            except:
                rospy.logerr(
                    f"Couldn't publish camera based location to topic {Topics.IMG_LOCALIZATION.value}"
                )
                break


def localization():
    """
    Runs the node itself and subscribes to all necessary topics. This is basically a adapter for the work\
    from Kai Binder which gets reused/integrated here.
    """
    global lidarBreakdown
    global config
    global fallbackPose

    rospy.init_node(Nodes.LOCALIZATION.value)
    set_rospy_log_lvl(rospy.INFO)
    rospy.loginfo(f"Starting node {Nodes.LOCALIZATION.value}")
    config = initCV(
        rospy.get_param(
            "~weights_path", default=f"{PATH}/src/yolov7/weights/statedict_ws_tiny5.pt"
        ),
        rospy.get_param("map_path", default=f"{PATH}/maps/FinalGridMapv2cleaned.png"),
    )
    # Saves the acml data to a global variable so the localization can use them
    rospy.Subscriber(
        Topics.AMCL_SOURCE.value, PoseWithCovarianceStamped, setRealData, queue_size=25
    )
    # ROS "setter"
    rospy.Subscriber(Topics.LIDAR_BREAKDOWN.value, Bool, setUseLidar, queue_size=10)
    rospy.Subscriber(Topics.IMAGE_RAW.value, Image, setImage, queue_size=25)
    rospy.Subscriber(Topics.ODOM_ROBOTINO.value, Odometry, setOdom, queue_size=25)
    rospy.Subscriber(Topics.INJECT_OFFSET.value, Bool, setInjectOffset, queue_size=10)
    rospy.Subscriber(
        Topics.ANOMALY_DETECTED.value, Bool, anomalyBehaviour, queue_size=1
    )

    # Publish localization
    msg = PoseWithCovarianceStamped()
    Thread(target=localiseCam).start()
    Thread(target=injectOffset).start()
    # Thread(target=anomalyDetector).start()
    while not rospy.is_shutdown():
        # ------ Check if LIDAR is running ------
        try:
            # ------ Take the right localization value -------
            if lidarBreakdown:
                if fallbackPose != None:
                    fallbackPosePub.publish(fallbackPose)

                msg = rospy.wait_for_message(
                    Topics.IMG_LOCALIZATION.value, PoseWithCovarianceStamped, timeout=1
                )
            else:
                msg = rospy.wait_for_message(
                    Topics.ACML.value, PoseWithCovarianceStamped, timeout=1
                )
            # ------ Publish the localization used by the Robotino ------
            # createOdom(msg)
            locPublisher.publish(msg)

        except rospy.ROSInterruptException:
            return
        except Exception as e:
            rospy.logdebug(
                f"Could publish localization message to topic {Topics.LOCALIZATION.value}: Error occured.\nException: {e} "
            )


if __name__ == "__main__":
    try:
        localization()
    except Exception as e:
        print(e)
        rospy.loginfo(f"Shutdown node {Nodes.LOCALIZATION.value}")
