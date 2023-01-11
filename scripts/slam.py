#!/usr/bin/env python3
import os, sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge

from constants import Topics, Nodes
from detect_online import get_conf_and_model, loaded_detect
from move_utils import *
from move_utils_cords import *

WEIGHTS_LOCALIZE = "yolov7/weights/ws_tiny5.pt"


def localise():
    """
    Uses the camera to localise the Robotino on the map. Uses the localization from Kai Binder\
    and is only integrated here.

    Args:
        Uses the global variable img_glob to get the

    """
    global acml_x
    global acml_y
    global acml_rot
    global img_glob
    global real_data
    global map_config

    conf_localise = get_conf_and_model(WEIGHTS_LOCALIZE)
    # gets the set of calibration data that needs to measured for each new png map
    base_info, map_config = get_base_info()
    # the gridmap is the locations of the workstations alligned with the 'grid' of the floortiles
    obstacles_ws, names_ws = get_obstacles_in_pixel_map(base_info)
    # the movable obstacles that need to be placed in the robots way
    obstacles_movable, names_movables = get_obstacles_in_pixel_map(base_info, "movable")
    # rotation of the workstations in the gridmap TODO remove this line and get it into the obstables?
    rots_ws = [0, 0, 0.85, 1.57, 2.19, 2.19]
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
    # map_ref.save('./image/test_loc.png') if you want to see/save the map at this point

    while not rospy.is_shutdown():
        # TODO This is for offline work, remove when finished with the calibration
        # real_data = ['fake',0.6,2.2,0.999] # 2.5 instead of 2 gets close
        # real_data = ['fake',-3.6,2.2,-0.0999] # 2.5 instead of 2 gets close
        # img_glob = cv2.imread('./yolov7/data/bb/img1.png')
        # while True:

        # copy newest image, the current postion, the workstations and the map TODO not sure if deepopy ws_map is needed
        img_local = deepcopy(img_glob)
        dc_obstacles_ws = deepcopy(obstacles_ws)
        local_acml_location = deepcopy(real_data)
        local_acml_location = offset_to_robo(local_acml_location)

        # copy map and create a drawing frame
        map_ref_loc = deepcopy(map_ref).convert("RGB")
        map_ref_loc_draw = ImageDraw.Draw(map_ref_loc)

        # convert the amcl pose rotation to radians
        amcl_rot_radians = 2 * np.arcsin(local_acml_location[3])
        draw_map_location_acml(
            local_acml_location[1],
            local_acml_location[2],
            amcl_rot_radians,
            map_ref_loc_draw,
            base_info,
        )

        # corners of the workstations on the map
        corners_map_all = [obstacle.corners for obstacle in dc_obstacles_ws]

        # getting the detection based on the newest image
        localise_dict = loaded_detect(img_local, *conf_localise)
        # if there was a detection
        if localise_dict:
            detected_workstation_dist, rotation_detected = (
                localise_dict["birds_eye"],
                localise_dict["rotation"],
            )
            # print(detected_workstation_dist)
            # turning the detected corners into an obstacle
            detected_obst = get_obstacles_from_detection(
                detected_workstation_dist, local_acml_location, base_info
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
                base_info,
            )

            # now we want to show the matched ws in blue, the acml location in green
            map_ref_loc = modify_map(
                map_ref_loc,
                [],
                [obstacles_ws[index_smallest_dist_ws]],
                color=(255, 0, 0),
                convert_back_to_grey=False,
            )
            map_ref_loc = modify_map(
                map_ref_loc,
                [],
                [detected_obst],
                color=(0, 255, 0),
                convert_back_to_grey=False,
            )
            map_ref_loc_draw = ImageDraw.Draw(map_ref_loc)

            # calculating rotation from detected and cord transforms
            # detected_rotation = -(rots_ws[index_smallest_dist_ws]-rotation_detected+np.pi-1.204)
            detected_rotation = (
                -(
                    rots_ws[index_smallest_dist_ws]
                    - rotation_detected
                    + np.pi
                    - map_config["rot"]
                )
                + 2 * np.pi
            )
            # basicly just transpose for the list
            corners_detec_2 = list(map(tuple, zip(*detected_workstation_dist)))
            # we get the detected localisation be subtracting the detected distances of a workstation from the actual one
            for i in range(
                len(corners_detec_2) - 3
            ):  # NOTE The -3 means we only go through once, all should be the same, useful for debug
                # change cords so we have the distance from the ws on the map
                corner = convert_cam_to_robo(
                    corners_detec_2[i][1], -corners_detec_2[i][0], detected_rotation
                )
                # change cords
                ws_map = get_amcl_from_pixel_location(
                    *corners_map_all[index_smallest_dist_ws][i], *base_info
                )
                # subtracting the detected distance from the workstation on the map
                loc_detec = (ws_map[0] - corner[0], ws_map[1] - corner[1])
            # print(loc_detec)
            print("adding to map")
            draw_map_location_acml(
                *loc_detec,
                detected_rotation,
                map_ref_loc_draw,
                base_info,
                color_FoV=(0, 0, 255),
                color_outer=(0, 255, 0)
            )

            print(detected_rotation, np.arcsin(local_acml_location[3]) * 2)


def setImage(rawImage: Image):
    global Image_data
    global img_glob
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

    # map_ref_loc.save('./image/test_loc_circle.png')
    cv2.imshow(
        "map", np.kron(np.asarray(map_ref_loc.convert("RGB")), np.ones((2, 2, 1)))
    )
    cv2.waitKey(1)
    # sleep(0.5)# in seconds


def slam():
    """
    Runs the node itself and subscribes to all necessary topics. This is basically a adapter for the work\
    from Kai Binder which gets reused/integrated here.
    """
    rospy.init_node(Nodes.SLAM.value)
    # Saves the image to a global variable so localization can use the image in its own thread
    rospy.Subscriber(Topics.IMAGE_RAW.value, Image, setImage)

    # Prevents python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    slam()
