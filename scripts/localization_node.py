#!/usr/bin/env python3
import time
import rospy

import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String, Bool
from tf.transformations import quaternion_from_euler
from copy import deepcopy
from cv_bridge import CvBridge

from utils.constants import Topics, Nodes
from utils.ros_logger import set_rospy_log_lvl
from yolov7.detect_online import get_conf_and_model, loaded_detect
from real_robot_navigation.gridmap import get_obstacles_in_pixel_map
from real_robot_navigation.move_utils import *
from real_robot_navigation.move_utils_cords import *
from utils.cv_utils import (
    draw_map_location_acml,
    get_obstacles_from_detection,
    best_match_workstation_index,
    initCV,
)

locPublisher = rospy.Publisher(Topics.LOCALIZATION.value, PoseWithCovarianceStamped, queue_size=10)

# TODO Remove all non-relevant stuff for SLAM
def localiseCam():
    """
    Uses the camera to localise the Robotino on the map. Uses the localization from Kai Binder\
    and is only integrated here.
    """
    global img_glob
    global real_data
    global config
    acml_x = 0
    acml_y = 0
    acml_rot = 0
    publisher = rospy.Publisher(Topics.IMG_LOCALIZATION.value, PoseWithCovarianceStamped, queue_size=10)
    # rotation of the workstations in the gridmap TODO remove this line and get it into the obstables?
    rots_ws = [0, 0, 0.85, 1.57, 2.19, 2.19]

    # TODO This is for offline work, remove when finished with the calibration
    # real_data = ['fake',0.6,2.2,0.999] # 2.5 instead of 2 gets close
    # real_data = ['fake',-3.6,2.2,-0.0999] # 2.5 instead of 2 gets close
    # img_glob = cv2.imread('./yolov7/data/bb/img1.png')
    # while True:

    # copy newest image, the current postion, the workstations and the map TODO not sure if deepopy ws_map is needed
    img_local = deepcopy(img_glob)
    dc_obstacles_ws = deepcopy(config["obstacles_ws"])
    local_acml_location = deepcopy(real_data)
    local_acml_location = offset_to_robo(local_acml_location)

    # copy map and create a drawing frame
    map_ref_loc = deepcopy(config["map_ref"]).convert("RGB")
    map_ref_loc_draw = ImageDraw.Draw(map_ref_loc)

    # TODO see if needed
    # convert the amcl pose rotation to radians
    # amcl_rot_radians = 2 * np.arcsin(local_acml_location[3])
    # acml_x, acml_y, acml_rot = draw_map_location_acml(
    #     local_acml_location[1],
    #     local_acml_location[2],rospy.get_param("~weights_path"
    #     amcl_rot_radians,
    #     map_ref_loc_draw,
    #     base_info,
    #     acml_x,
    #     acml_y,
    #     acml_rot,
    # )

    # corners of the workstations on the map
    corners_map_all = [obstacle.corners for obstacle in dc_obstacles_ws]

    # getting the detection based on the newest image
    localise_dict = loaded_detect(img_local, *config["conf_network"])
    # if there was a detection
    if localise_dict:
        detected_workstation_dist, rotation_detected = (
            localise_dict["birds_eye"],
            localise_dict["rotation"],
        )

        # print(detected_workstation_dist)
        # turning the detected corners into an obstacle
        detected_obst = get_obstacles_from_detection(
            detected_workstation_dist, local_acml_location, config["base_info"]
        )
        # comparing detected obstacles with workstations on map to find correct one
        detection_corners = list(map(tuple, zip(*detected_workstation_dist)))
        index_smallest_dist_ws = best_match_workstation_index(
            corners_map_all,
            detected_obst,
            detection_corners,
            rots_ws,
            local_acml_location,
            rotation_detected,
            config["base_info"],
        )

        # now we want to show the matched ws in blue, the acml location in green => Visualization
        # map_ref_loc = modify_map(
        #     map_ref_loc,
        #     [],
        #     [config["obstacles_ws"][index_smallest_dist_ws]],
        #     color=(255, 0, 0),
        #     convert_back_to_grey=False,
        # )rospy.get_param("~weights_path"
        # map_ref_loc = modify_map(
        #     map_ref_loc,
        #     [],
        #     [detected_obst],
        #     color=(0, 255, 0),
        #     convert_back_to_grey=False,
        # )
        # map_ref_loc_draw = ImageDraw.Draw(map_ref_loc)

        # calculating rotation from detected and cord transforms
        # detected_rotation = -(rots_ws[index_smallest_dist_ws]-rotation_detected+np.pi-1.204)
        detected_rotation = (
            -(rots_ws[index_smallest_dist_ws] - rotation_detected + np.pi - config["map_config"]["rot"]) + 2 * np.pi
        )
        # basicly just transpose for the list
        corners_detec_2 = list(map(tuple, zip(*detected_workstation_dist)))
        # we get the detected localisation be subtracting the detected distances of a workstation from the actual one
        # NOTE The -3 means we only go through once, all should be the same, useful for debug
        for i in range(len(corners_detec_2) - 3):
            # change cords so we have the distance from the ws on the map
            corner = convert_cam_to_robo(corners_detec_2[i][1], -corners_detec_2[i][0], detected_rotation)
            # change cords
            ws_map = get_amcl_from_pixel_location(*corners_map_all[index_smallest_dist_ws][i], *config["base_info"])
            # subtracting the detected distance from the workstation on the map
            loc_detec = (ws_map[0] - corner[0], ws_map[1] - corner[1])

        # acml_x, acml_y, acml_rot = draw_map_location_acml(
        #     *loc_detec,
        #     detected_rotation,
        #     map_ref_loc_draw,
        #     config["base_info"],)rospy.get_param("~weights_path"
        #     color_outer=(0, 255, 0),
        # )

        # Create message to publish
        locMsg = PoseWithCovarianceStamped()
        # Position
        locMsg.pose.pose.position.x = loc_detec[0]
        locMsg.pose.pose.position.y = loc_detec[1]
        locMsg.pose.pose.position.z = 0
        # Orientation
        quaternion = quaternion_from_euler(0, 0, np.sin(detected_rotation / 2))
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
            rospy.logerr(f"Couldn't publish camera based location to topic {Topics.IMG_LOCALIZATION.value}")


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

    # Run localisation
    localiseCam()


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


def localization():
    """
    Runs the node itself and subscribes to all necessary topics. This is basically a adapter for the work\
    from Kai Binder which gets reused/integrated here.
    """
    global useLidar
    global config
    useLidar = True

    rospy.init_node(Nodes.LOCALIZATION.value)
    set_rospy_log_lvl(rospy.DEBUG)
    rospy.loginfo(f"Starting node {Nodes.LOCALIZATION.value}")
    config = initCV(rospy.get_param("~weights_path"), rospy.get_param("map_path"))
    # Saves the image to a global variable so localization can use the image in its own thread
    rospy.Subscriber(Topics.IMAGE_RAW.value, Image, setImage, queue_size=1)
    # Saves the acml data to a global variable so the localization can use them
    rospy.Subscriber(Topics.ACML.value, PoseWithCovarianceStamped, setRealData, queue_size=10)
    rospy.Subscriber(Topics.LIDAR_ENABLED.value, Bool, setUseLidar, queue_size=10)

    # For determining localization mode
    locMode = "lidar"
    # Publish localization
    rate = rospy.Rate(500)
    msg = PoseWithCovarianceStamped()

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
                msg = rospy.wait_for_message(Topics.IMG_LOCALIZATION.value, PoseWithCovarianceStamped, timeout=1)
            elif locMode.lower() == "lidar":
                msg = rospy.wait_for_message(Topics.ACML.value, PoseWithCovarianceStamped, timeout=1)
            else:
                rospy.logwarn(
                    f'No valid mode specified for localization. Make shure to specify localization mode over the topic {Topics.LOCALIZATION_MODE.value} with either "camera" or "LIDAR"'
                )
                break
            # ------ Publish the localization used by the Robotino ------
            locPublisher.publish(msg)

        except rospy.ROSInterruptException:
            return
        except Exception as e:
            rospy.logdebug(
                f"Could publish localization message to topic {Topics.LOCALIZATION.value}: Error occured.\nException: {e} "
            )

    rate.sleep()


if __name__ == "__main__":
    try:
        localization()
    except:
        rospy.loginfo(f"Shutdown node {Nodes.LOCALIZATION.value}")
