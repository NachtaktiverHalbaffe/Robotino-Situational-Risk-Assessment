#!/usr/bin/env python3
import csv
import rospy
import sys, os
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from copy import deepcopy


sys.path.append(os.getcwd())
from thesis.msg import Obstacle, ObstacleList
from constants import Nodes, Topics
from real_robot_navigation.gridmap import get_obstacles_in_pixel_map
from utils.cv_utils import (
    draw_map_location_acml,
    get_obstacles_from_detection,
)
from yolov7.detect_online import get_conf_and_model, loaded_detect
from real_robot_navigation.move_utils import *
from real_robot_navigation.move_utils_cords import *


WEIGHTS_DETECTION = "yolov7/weights/tiny10_hocker.pt"
PATH_ERROR_DIST = "logs/error_dist_csvs/error_dist_detec_22_12.csv"
PATH_ERROR_DIST_DOOR_CLOSED = (
    "logs/error_dist_csvs/error_dist_detec_22_12_rot_door_closed.csv"
)


def detect(log_detection_error=True):
    """
    Detects objects by using the camera with a YOLOv7 neural network. Mainly uses\
    the work from of Kai Binder and is only integrated here into the prototype.
    """
    global img_glob
    global real_data
    publisher = rospy.Publisher(Topics.OBSTACLES.value, ObstacleList)

    # Loads the models
    conf_detection = get_conf_and_model(WEIGHTS_DETECTION)
    # gets the set of calibration data that needs to measured for each new png map
    base_info, map_config = get_base_info()
    # the gridmap is the locations of the workstations alligned with the 'grid' of the floortiles
    obstacles_ws, _ = get_obstacles_in_pixel_map(base_info)
    # the movable obstacles that need to be placed in the robots way
    obstacles_movable, names_movables = get_obstacles_in_pixel_map(base_info, "movable")
    # rotation of the workstations in the gridmap TODO remove this line and get it into the obstables?
    # rots_ws = [0, 0, 0.85, 1.57, 2.19, 2.19]
    # loading the png of the map and running the old detection system on it also configures the map to the right type
    map_path = map_config["path"]
    map_ref, obstacles = initialize_map(map_path)
    # removing the obstacles from the map, adding the workstations, removing old should not be needed if we have a new map, TODO new map add new obstacles such as klappbox, chair and box
    map_ref = modify_map(
        map_ref, obstacles, obstacles_ws, color=(255, 0, 255), convert_back_to_grey=True
    )
    # adding the movable objects
    map_ref = modify_map(
        map_ref, [], obstacles_movable, color=(255, 0, 0), convert_back_to_grey=True
    )

    local_acml_location = deepcopy(real_data)
    local_acml_location = offset_to_robo(local_acml_location)

    # copy map and create a drawing frame
    map_ref = deepcopy(map_ref).convert("RGB")
    map_ref_loc_draw = ImageDraw.Draw(map_ref)

    # ------------------------- Detection itself -----------------------------
    img_local = deepcopy(img_glob)
    detec_movables = loaded_detect(img_local, *conf_detection, True)
    detec_movables_obstacles = []
    rotations_detected = []
    index_names = []

    for detec_m in detec_movables:
        index_names.append(names_movables.index(detec_m["label"]))
        detec_movables_obstacle = get_obstacles_from_detection(
            detec_m["birds_eye"], local_acml_location, base_info
        )
        detec_movables_obstacles.append(detec_movables_obstacle)
        rotations_detected.append(detec_m["rotation"])

    if detec_movables_obstacles:
        objects_to_move = [obstacles_movable[i] for i in index_names]
        map_ref = modify_map(
            map_ref,
            [],
            detec_movables_obstacles,
            color=(0, 255, 255),
            convert_back_to_grey=False,
        )
        # map_ref_loc = modify_map(map_ref_loc,objects_to_move,detec_movables_obstacles,color=(0,255,255),convert_back_to_grey=False)
        map_ref_loc_draw = ImageDraw.Draw(map_ref)

    # ----------- Determine error distribution and write it to a CSV ------------
    if log_detection_error and detec_movables_obstacles:
        for ground_truth, detection_assumption, rotation_detected in zip(
            objects_to_move, detec_movables_obstacles, rotations_detected
        ):
            # here we need to split stuff into x,y
            g_truth = np.array(ground_truth.corners)
            detec_assum = np.array(detection_assumption.corners)
            # summing over the corners and then subtracting before deviding by number of corners gives the distances of centers
            differences = np.sum(g_truth, axis=0) / 4 - np.sum(detec_assum, axis=0) / 4
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
                old_rot_assumption = 2 * np.arcsin(local_acml_location[3])
                rot = (
                    -(0 - rotation_detected.numpy() + np.pi - map_config["rot"])
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
                print(error)
                write.writerow([error])

    # convert the amcl pose rotation to radians
    amcl_rot_radians = 2 * np.arcsin(local_acml_location[3])
    draw_map_location_acml(
        local_acml_location[1],
        local_acml_location[2],
        amcl_rot_radians,
        map_ref_loc_draw,
        base_info,
    )

    # map_ref_loc.save('./image/test_loc_circle.png')

    # Create message
    msg = ObstacleList()
    for obstacle in detec_movables_obstacles:
        obstacleItem = Obstacle()
        obstacleItem.corners = obstacle.corners
        msg.obstacles.append(obstacleItem)
    # Publish message
    try:
        publisher.publish(msg)
    except:
        rospy.logwarn(f"Couldn't publish Obstacles to topic {Topics.OBSTACLES.value} ")


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

    # Run object detection
    detect()


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
    rospy.init_node(Nodes.OBJECT_DETECTION.value)
    # Save the localization data to a global variable so the detection can use them
    rospy.Subscriber(
        Topics.LOCALIZATION.value, PoseWithCovarianceStamped, setRealData, queue_size=10
    )
    # Triggers to run the object detection
    rospy.Subscriber(Topics.IMAGE_RAW.value, Image, setImage, queue_size=1)

    # Prevents python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    objectDetection()
