#!/usr/bin/env python3
import csv
import rospy
import os
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, PolygonStamped
from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import PolygonArray
from copy import deepcopy

from prototype.msg import ObstacleMsg, ObstacleList
from utils.constants import Nodes, Topics
from utils.cv_utils import get_obstacles_from_detection, initCV
from utils.ros_logger import set_rospy_log_lvl
from yolov7.detect_online import loaded_detect
from real_robot_navigation.move_utils import *
from real_robot_navigation.move_utils_cords import *

img_glob = []
PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../", ""))


def visualizeObstacles():
    """
    Create Polygon messages and sends them to rviz. Is used to visualize the detected obstacles in rviz
    """
    global detectedObstacles
    msg = PolygonArray()
    msg.header.frame_id = "map"
    for obstacle in detectedObstacles:
        polygon = PolygonStamped()
        polygon.header.frame_id = "map"
        for corner in obstacle.corners:
            transformedCor = get_amcl_from_pixel_location(
                corner[0], corner[1], *config["base_info"]
            )
            polygon.polygon.points.append(
                Point(transformedCor[0], transformedCor[1], 0.2)
            )

        msg.polygons.append(polygon)
        msg.labels.append(obstacle.label)

    rospy.Publisher(Topics.OBSTACLES_VISU.value, PolygonArray, queue_size=10).publish(
        msg
    )


def detect(log_detection_error=True):
    """
    Detects objects by using the camera with a YOLOv7 neural network. Mainly uses\
    the work from of Kai Binder and is only integrated here into the prototype.
    """
    global img_glob
    global real_data
    global config
    global detectedObstacles

    PATH_ERROR_DIST = rospy.get_param(
        "~path_error_dist",
        default=f"{PATH}/logs/error_dist_csvs/error_dist_detec_22_12.csv",
    )
    PATH_ERROR_DIST_DOOR_CLOSED = rospy.get_param(
        "~path_error_dist_doorclosed",
        default=f"{PATH}/logs/error_dist_csvs/error_dist_detec_22_12_rot_door_closed.csv",
    )
    publisher = rospy.Publisher(Topics.OBSTACLES.value, ObstacleList, queue_size=10)
    # copy map and create a drawing frame
    map_ref = deepcopy(config["map_ref"]).convert("RGB")
    while not rospy.is_shutdown():
        if len(real_data) == 0 or len(img_glob) == 0:
            continue

        currentLocation = deepcopy(real_data)
        currentLocation = offset_to_robo(currentLocation)
        # ------------------------- Detection itself -----------------------------
        img_local = deepcopy(img_glob)
        detec_movables = loaded_detect(
            img_local, *config["conf_network"], False, node="object_detection"
        )
        detec_movables_obstacles = []
        rotations_detected = []
        index_names = []

        for detec_m in detec_movables:
            index_names.append(config["names_movables"].index(detec_m["label"]))
            detec_movables_obstacle = get_obstacles_from_detection(
                detec_m["birds_eye"],
                currentLocation,
                config["base_info"],
                label=detec_m["label"],
            )
            detec_movables_obstacles.append(detec_movables_obstacle)
            rotations_detected.append(detec_m["rotation"])

        if detec_movables_obstacles:
            objects_to_move = [config["obstacles_movable"][i] for i in index_names]
            map_ref = modify_map(
                map_ref,
                [],
                detec_movables_obstacles,
                color=(255, 255, 255),
                convert_back_to_grey=True,
                savePath=f"{PATH}/maps/map_obstacles.png",
            )

        # ----------- Determine error distribution and write it to a CSV ------------
        if log_detection_error and detec_movables_obstacles:
            for ground_truth, detection_assumption, rotation_detected in zip(
                objects_to_move, detec_movables_obstacles, rotations_detected
            ):
                # here we need to split stuff into x,y
                g_truth = np.array(ground_truth.corners)
                detec_assum = np.array(detection_assumption.corners)
                # summing over the corners and then subtracting before deviding by number of corners gives the distances of centers
                differences = (
                    np.sum(g_truth, axis=0) / 4 - np.sum(detec_assum, axis=0) / 4
                )
                differences_pyt = np.sqrt(np.sum(np.power(differences, 2)))
                with open(
                    PATH_ERROR_DIST,
                    "a",
                ) as f1:
                    write = csv.writer(f1)
                    write.writerow([differences[0], differences[1], differences_pyt])
                with open(
                    PATH_ERROR_DIST_DOOR_CLOSED,
                    "a",
                ) as f1:
                    write = csv.writer(f1)
                    old_rot_assumption = 2 * np.arcsin(currentLocation[3])
                    rot = (
                        -(
                            0
                            - rotation_detected.numpy()
                            + np.pi
                            - config["map_config"]["rot"]
                        )
                        + 2 * np.pi
                    )
                    rot_shift = float(
                        min(
                            abs(rot + np.pi - (old_rot_assumption + np.pi)),
                            abs(rot + 3 * np.pi - (old_rot_assumption + np.pi)),
                            abs(rot + np.pi - (old_rot_assumption + 3 * np.pi)),
                        )
                    )
                    error = min(
                        abs(rot_shift),
                        abs(rot_shift - np.pi / 2),
                        abs(rot_shift - np.pi),
                    )
                    # print(error)
                    write.writerow([error])

        # Create message
        msg = ObstacleList()
        for obstacle in detec_movables_obstacles:
            corners = []
            # Convert corners to ROS points
            for corner in obstacle.corners:
                cornerItem = Point()
                cornerItem.x = corner[0]
                cornerItem.y = corner[1]
                corners.append(cornerItem)
            #  Append Obstacle to message
            obstacleMsg = ObstacleMsg
            obstacleMsg.corners = corners
            obstacleMsg.label = obstacle.label
            msg.obstacles.append(obstacleMsg)
        # Publish message
        try:
            rospy.logdebug(f"[Object Detection] Publishing detected obstacles: {msg}")
            publisher.publish(msg)
        except:
            rospy.logwarn(
                f"[Object Detection] Couldn't publish Obstacles to topic {Topics.OBSTACLES.value} "
            )

        detectedObstacles = detec_movables_obstacles
        visualizeObstacles()

        rospy.Rate(25).sleep()


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


def objectDetection():
    """
    Runs the node itself and subscribes to all necessary topics. This is basically a adapter for the\
    object detection implemented by Kai Binder
    """
    global config
    rospy.init_node(Nodes.OBJECT_DETECTION.value)
    set_rospy_log_lvl(rospy.INFO)
    rospy.loginfo(f"Starting node {Nodes.OBJECT_DETECTION.value}")

    config = initCV(
        rospy.get_param(
            "~weights_path",
            default=f"{PATH}/src/yolov7/weights/statedict_tiny10_hocker.pt",
        ),
        rospy.get_param("map_path", default=f"{PATH}/maps/FinalGridMapv2cleaned.png"),
    )
    # Save one empty map at start for risk estimator with no obstacles in it
    modify_map(
        config["map_ref"],
        [],
        [],
        color=(255, 255, 255),
        convert_back_to_grey=True,
        savePath=f"{PATH}/maps/map_obstacles.png",
    )
    # Save the localization data to a global variable so the detection can use them
    rospy.Subscriber(
        Topics.LOCALIZATION.value, PoseWithCovarianceStamped, setRealData, queue_size=25
    )
    # Triggers to run the object detection
    rospy.Subscriber(Topics.IMAGE_RAW.value, Image, setImage, queue_size=25)
    detect()

    # Prevents python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    try:
        objectDetection()
    except Exception as e:
        print(e)
        rospy.loginfo(f"Shutdown node {Nodes.OBJECT_DETECTION.value}")
