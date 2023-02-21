#!/usr/bin/env python3
import time
import rospy
import numpy as np
import pandas as pd
import os
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import (
    quaternion_from_euler,
    euler_from_quaternion,
    quaternion_multiply,
)
from copy import deepcopy
from cv_bridge import CvBridge
from threading import Thread

from utils.constants import Topics, Nodes
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
    Topics.LOCALIZATION.value, PoseWithCovarianceStamped, queue_size=10
)
last_known_loc = ["init_data", 0, 0, 0]
last_update = time.time()
img_glob = []
odom = Odometry()


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
    global useLidar
    useLidar = isUsed.data

    if useLidar:
        rospy.logdebug_throttle(10, "[Localization] Use LIDAR")
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


def anomalyDetector(savePath: str = "{PATH}/logs/error_dist_csvs/localization_error"):
    """
    Tracks/saves the error distribution between localization with LIDAR (amcl) and camera

    Args:
        acmlPose (PoseWithCovarianceStamped): Position determined by LIDAR
        imageLoc (PoseWithCovarianceStamped): Position determined by camera based localization
    """
    while not rospy.is_shutdown():
        imageLoc = rospy.wait_for_message(
            Topics.IMG_LOCALIZATION.value, PoseWithCovarianceStamped
        )
        amclPose = rospy.wait_for_message(Topics.ACML.value, PoseWithCovarianceStamped)

        # Calculating error in localization in x,y and angle
        xErr = amclPose.pose.pose.position.x - imageLoc.pose.pose.position.x
        yErr = amclPose.pose.pose.position.y - imageLoc.pose.pose.position.y
        distError = np.sqrt(np.sum(np.square(xErr), np.square(yErr)))
        angleErr = amclPose.pose.pose.orientation.z - imageLoc.pose.pose.orientation.z

        # Dump raw xError into csv file
        data = pd.read_csv(f"{savePath}_x.csv")
        data.append(pd.DataFrame(xErr), ignore_index=True)
        data.to_csv(f"{savePath}_x.csv")
        # Dump raw yError into csv file
        data = pd.read_csv(f"{savePath}_y.csv")
        data.append(pd.DataFrame(yErr), ignore_index=True)
        data.to_csv(f"{savePath}_y.csv")
        # Dump raw distError into csv file
        data = pd.read_csv(f"{savePath}_dist.csv")
        data.append(pd.DataFrame(distError), ignore_index=True)
        data.to_csv(f"{savePath}_dist.csv")
        # Dump raw angleError into csv file
        data = pd.read_csv(f"{savePath}_angle.csv")
        data.append(pd.DataFrame(angleErr), ignore_index=True)
        data.to_csv(f"{savePath}_angle.csv")


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
    global last_known_loc
    global last_update
    global odom
    last_known_odom = odom
    dc_obstacles_ws = deepcopy(config["obstacles_ws"])

    publisher = rospy.Publisher(
        Topics.IMG_LOCALIZATION.value, PoseWithCovarianceStamped, queue_size=10
    )
    while not rospy.is_shutdown():
        loc_detec = [0, 0]
        detected_rotation = 0

        # rotation of the workstations in the gridmap TODO remove this line and get it into the obstables?
        rots_ws = [0, 0, 0.85, 1.57, 2.19, 2.19]
        # last_known_locs and the map TODO not sure if deepopy ws_map is needed
        if len(img_glob) != 0:
            img_local = deepcopy(img_glob)
        else:
            continue

        # Use last known location as location assumption if at least it located once with camera-based localization, otherwise use amcl data as initial pose
        if (
            not "init_data" in last_known_loc[0].lower()
            and not "cv_data" in last_known_loc[0].lower()
        ):
            location_assumption = deepcopy(last_known_loc)
            if not "interpolated_data" in location_assumption[0].lower():
                location_assumption = offset_to_robo(last_known_loc)
        else:
            if len(real_data) != 0:
                location_assumption = deepcopy(real_data)
                location_assumption = offset_to_robo(location_assumption)
            else:
                continue

        # corners of the workstations on the map
        corners_map_all = [obstacle.corners for obstacle in dc_obstacles_ws]

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

        if loc_detec[0] != 0 and loc_detec[1] != 0 and detected_rotation != 0:
            last_known_loc = [
                "cv_data",
                round(float(loc_detec[0]), PRECISION),
                round(float(loc_detec[1]), PRECISION),
                round(float(detected_rotation), PRECISION),
            ]
        elif (
            "cv_data" in last_known_loc[0].lower()
            or "interpolated_data" in last_known_loc[0].lower()
        ):
            # Calculate position => add difference between last odom and currentodom to last known location
            loc_detec[0] = odom.pose.pose.position.x
            loc_detec[1] = odom.pose.pose.position.y
            loc_detec[0] = round(
                last_known_loc[1]
                + (loc_detec[0] - last_known_odom.pose.pose.position.x),
                PRECISION,
            )
            loc_detec[1] = round(
                last_known_loc[2]
                + (loc_detec[1] - last_known_odom.pose.pose.position.y),
                PRECISION,
            )
            # Calculate orientation=> add difference between last odom and currentodom to last known rotation
            quat = odom.pose.pose.orientation
            quatLastknown = last_known_odom.pose.pose.orientation
            _, _, detected_rotation = euler_from_quaternion(
                [quat.x, quat.y, quat.z, quat.w]
            )
            _, _, last_known_rotation = euler_from_quaternion(
                [quatLastknown.x, quatLastknown.y, quatLastknown.z, quatLastknown.w]
            )
            detected_rotation = round(
                last_known_loc[3] + (detected_rotation - last_known_rotation), PRECISION
            )

            last_known_loc = [
                "interpolated_data",
                loc_detec[0],
                loc_detec[1],
                detected_rotation,
            ]

        last_known_odom = odom
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
            publisher.publish(locMsg)
        except:
            rospy.logerr(
                f"Couldn't publish camera based location to topic {Topics.IMG_LOCALIZATION.value}"
            )
            break
        last_update = time.time()


def setOdom(newOdom: Odometry):
    global odom
    odom = newOdom


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


def localization():
    """
    Runs the node itself and subscribes to all necessary topics. This is basically a adapter for the work\
    from Kai Binder which gets reused/integrated here.
    """
    global useLidar
    global config
    useLidar = True

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
        Topics.ACML.value, PoseWithCovarianceStamped, setRealData, queue_size=25
    )
    rospy.Subscriber(Topics.LIDAR_ENABLED.value, Bool, setUseLidar, queue_size=10)
    # Saves the image to a global variable so localization can use the image in its own thread
    rospy.Subscriber(Topics.IMAGE_RAW.value, Image, setImage, queue_size=25)
    rospy.Subscriber(Topics.ODOM_ROBOTINO.value, Odometry, setOdom, queue_size=25)

    # For determining localization mode
    locMode = "lidar"
    # Publish localization
    msg = PoseWithCovarianceStamped()
    Thread(target=localiseCam).start()
    Thread(target=anomalyDetector).start()
    while not rospy.is_shutdown():
        # ------ Check if LIDAR is running ------
        if useLidar:
            locMode = "lidar"
        else:
            # Lidar-process wasn't found
            locMode = "camera"
        try:
            # ------ Take the right localization value -------
            if locMode.lower() == "camera":
                msg = rospy.wait_for_message(
                    Topics.IMG_LOCALIZATION.value, PoseWithCovarianceStamped, timeout=1
                )
            elif locMode.lower() == "lidar":
                msg = rospy.wait_for_message(
                    Topics.ACML.value, PoseWithCovarianceStamped, timeout=1
                )
            else:
                rospy.logwarn(
                    f'No valid mode specified for localization. Make shure to specify localization mode over the topic {Topics.LOCALIZATION_MODE.value} with either "camera" or "LIDAR"'
                )
                break
            # ------ Publish the localization used by the Robotino ------
            createOdom(msg)
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
