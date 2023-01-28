"""
This is a modified version of move_localize_joystick from author Kai Binder. It only contains the 
necessary utils function needed for the localization. For this purposed, all global variables 
are moved to function arguments
"""
import os, sys
from pathlib import Path
import pickle
from real_robot_navigation.gridmap import get_obstacles_in_pixel_map

from yolov7.detect_online import get_conf_and_model

from real_robot_navigation.move_utils import *
from real_robot_navigation.move_utils_cords import *

PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../..", ""))

PATH_ERRORDIST_LOC_X1 = f"{PATH}/logs/error_dist_csvs/loc/error_dist_loc_x1.csv"
PATH_ERRORDIST_LOC_Y1 = f"{PATH}/logs/error_dist_csvs/loc/error_dist_loc_y1.csv"
PATH_ERRORDIST_LOC_ROT = f"{PATH}/logs/error_dist_csvs/loc/error_dist_loc_rot_abs_chair1.csv"
PATH_ERRORDIST_LOC_DIST = f"{PATH}/logs/error_dist_csvs/loc/error_dist_loc_dist1.csv"
PATH_ERRORDIST_LOC = f"{PATH}/logs/error_dist_csvs/loc/error_dist_loc1.csv"
PATH_ERRORDIST_LOC_FILTERED = f"{PATH}/logs/error_dist_csvs/loc/error_dist_loc_dist_filtered1.csv"

PATH_ERRORDIST_PRM_LOC_X1 = f"{PATH}/logs/error_dist_csvs/loc/error_dist_loc_x_PRM_end.csv"
PATH_ERRORDIST_PRM_LOC_Y1 = f"{PATH}/logs/error_dist_csvs/loc/error_dist_loc_y_PRM_end.csv"
PATH_ERRORDIST_PRM_LOC_ROT = f"{PATH}/logs/error_dist_csvs/loc/error_dist_loc_rot_abs_chair_PRM_end.csv"
PATH_ERRORDIST_PRM_LOC_DIST = f"{PATH}/logs/error_dist_csvs/loc/error_dist_loc_dist_PRM_end.csv"
PATH_ERRORDIST_PRM_LOC = f"{PATH}/logs/error_dist_csvs/loc/error_dist_loc_PRM_end.csv"
PATH_ERRORDIST_PRM_LOC_FILTERED = f"{PATH}/logs/error_dist_csvs/loc/error_dist_loc_dist_filtered_PRM_end.csv"


def get_dists_workstation(corners_map_all, obstacle):
    """
    This gets the distances each workstation has to the ws placed by the detection

    Args:
        corners_map_all: The corners of each workstation in a list
        obstacle: The detected ws as a distance from the camera

    Returns:
        dists: array of the distances
    """
    dists = []
    smallest_dist = 10e10
    for corners, i in zip(corners_map_all, range(len(corners_map_all))):
        # distances between a workstations corners and the detected workstations corners
        dist = np.array(corners) - np.array(obstacle.corners)
        # pythagoras, root not needed because we just use this to sort
        dist = np.power(dist, 2)
        dist = np.sum(dist)
        dist = np.sum(dist)
        dist = np.sqrt(dist)
        dists.append(dist)
        if dist < smallest_dist:
            smallest_dist = dist
    return np.array(dists)


def best_match_workstation_index(
    corners_map_all,
    obstacle,
    detection_corners,
    rots_ws,
    old_loc_assumption,
    rotation_detected,
    base_info,
    map_config,
):
    """
    Finds the workstation that best matches the detection based on the currently assumed postion

    Args:
        corners_map_all: the corners of the workstations in the map
        obstacle: the detected workstation based on the old location
        detection_corners: the distance from detected workstation to the camera
        rots_ws: the rotation of the workstations on the map
        old_loc_assumption: where we currently assume to be based on acml
        rotation_detected: the rotation the object detection infered
        base_info: this is the set of calibration data needed to correctly transform from acml to pixel cords

    Returns:
        detected_obst: the detected object as an Obstacle
    """
    dists_ws = get_dists_workstation(corners_map_all, obstacle)
    # arg_smallest_dist_rot, dists_rot = get_smalles_dist_rotations(corners_map_all, obstacle,old_loc_assumption)
    # TODO check if condition checking can be removed
    if old_loc_assumption[0] == "real_data" or True:
        old_rot_assumption = 2 * np.arcsin(old_loc_assumption[3])
    else:
        old_rot_assumption = local_acml_location[3]
    # calculating rotation from detected and cord transforms
    # detected_rotations = -(np.array(rots_ws)-rotation_detected.numpy()+np.pi-1.204)
    detected_rotations = -(np.array(rots_ws) - rotation_detected.numpy() + np.pi - map_config["rot"]) + 2 * np.pi
    dists_rot = deepcopy(detected_rotations)
    for rot, i in zip(detected_rotations, range(len(detected_rotations))):
        rot_shift = float(
            min(
                abs(rot + np.pi - (old_rot_assumption + np.pi)),
                abs(rot + 3 * np.pi - (old_rot_assumption + np.pi)),
                abs(rot + np.pi - (old_rot_assumption + 3 * np.pi)),
            )
        )
        dists_rot[i] = rot_shift
    # we get the detected localisation be subtracting the detected distances of a workstation from its location
    dists_loc = []
    for j in range(len(corners_map_all)):
        for i in range(
            len(detection_corners) - 3
        ):  # NOTE The -3 means we only go through once, all should be the same, useful for debug
            # change cords so we have the distance from the ws on the map
            detection_corner = convert_cam_to_robo(
                detection_corners[i][1], -detection_corners[i][0], detected_rotations[j]
            )
            # change cords
            ws_map = get_amcl_from_pixel_location(*corners_map_all[j][i], *base_info)
            # subtracting the detected distance from the workstation on the map
            loc_detec = (
                ws_map[0] - detection_corner[0],
                ws_map[1] - detection_corner[1],
            )
            # loc_detec_acml = (ws_map[0]-corner[0],ws_map[1]-corner[1])
            old_loc_assumption_p = get_pixel_location_from_acml(
                old_loc_assumption[1], old_loc_assumption[2], *base_info
            )
            x_shift = loc_detec[0] - old_loc_assumption[1]
            y_shift = loc_detec[1] - old_loc_assumption[2]
            dist_shift = float(np.sqrt(np.power(x_shift, 2) + np.power(y_shift, 2)))
            dists_loc.append(dist_shift)
    dists_loc = np.array(dists_loc)
    # weighted sum to give each metric the same importance
    dists = np.sum(np.array([dists_ws, dists_rot * 100, dists_loc * 20]), axis=0)
    arg_smallest_dist = np.argmin(dists)
    return arg_smallest_dist


def get_obstacles_from_detection(detected_workstation_dist, local_acml_location, base_info):
    """
    Rurns the info provided by the object detection into an obstacle

    Args:
        detected_workstation_dist: the distance from cam to each corner of the object
        local_acml_location: This is the current position in acml coordinate system
        base_info: this is the set of calibration data needed to correctly transform from acml to pixel cords

    Returns:
        detected_obst: the detected object as an Obstacle
    """
    corners_detec = list(map(tuple, zip(*detected_workstation_dist)))
    rot = 2 * np.arcsin(local_acml_location[3])
    for i in range(len(corners_detec)):
        corner = convert_cam_to_robo(corners_detec[i][1], -corners_detec[i][0], rot)
        corner = (
            corner[0] + local_acml_location[1],
            corner[1] + local_acml_location[2],
        )
        corner = get_pixel_location_from_acml(*corner, *base_info)
        corners_detec[i] = corner
    detected_obst = Obstacle(corners_detec)
    return detected_obst


def draw_map_location_acml(
    x,
    y,
    rot,
    draw_frame,
    base_info,
    acmlX,
    acmlY,
    acmlRot,
    color_FoV=(255, 0, 255),
    color_outer=(255, 0, 0),
):
    """This function draws the robot onto the pixelmap

    Args:
        x: the x cord in acml cord system
        y: the y cord in acml cord system
        ros: the the roation of the robot
        draw_frame: the image to draw on
        base_info: this is the set of calibration data needed to correctly transform from acml to pixel cords
        color_FoV: Color to draw the FoV window in
        color_outer: Color to draw the rest of the robot circle represenation in
    """
    acml_x = acmlX
    acml_y = acmlY
    acml_rot = acmlRot
    write_to_cvs = False
    # we need to convert the location to pixels
    if color_FoV == (255, 0, 255):
        x, y = get_pixel_location_from_acml(x, y, *base_info, rot)
        acml_x = x
        acml_y = y
        acml_rot = rot
    else:
        x, y = get_pixel_location_from_acml(x, y, *base_info)
        # we only want to save the diff for the location recorded by the detection
        if write_to_cvs and color_FoV == (0, 0, 255):
            x_shift = float(x - acml_x)
            y_shift = float(y - acml_y)
            # The roation diff needs to account for wrapping around back to 0
            rot_shift = float(
                min(
                    abs(rot + np.pi - (acml_rot + np.pi)),
                    abs(rot + 3 * np.pi - (acml_rot + np.pi)),
                    abs(rot + np.pi - (acml_rot + 3 * np.pi)),
                )
            )
            dist_shift = float(np.sqrt(np.power(x_shift, 2) + np.power(y_shift, 2)))
            with open(PATH_ERRORDIST_LOC_X1, "a") as f1:
                write = csv.writer(f1)
                write.writerow([x_shift])
            with open(PATH_ERRORDIST_LOC_Y1, "a") as f1:
                write = csv.writer(f1)
                write.writerow([y_shift])
            with open(PATH_ERRORDIST_LOC_ROT, "a") as f1:
                write = csv.writer(f1)
                write.writerow([rot_shift])
            with open(PATH_ERRORDIST_LOC_DIST, "a") as f1:
                write = csv.writer(f1)
                write.writerow([dist_shift])
            with open(PATH_ERRORDIST_LOC, "a") as f1:
                write = csv.writer(f1)
                write.writerow([x_shift, y_shift, rot_shift, dist_shift])
            if rot_shift < 0.20:
                with open(PATH_ERRORDIST_LOC_FILTERED, "a") as f1:
                    write = csv.writer(f1)
                    write.writerow([dist_shift])

        if write_to_cvs and color_FoV == (255, 0, 0):
            x_shift = float(x - acml_x)
            y_shift = float(y - acml_y)
            # The roation diff needs to account for wrapping around back to 0
            rot_shift = float(
                min(
                    abs(rot + np.pi - (acml_rot + np.pi)),
                    abs(rot + 3 * np.pi - (acml_rot + np.pi)),
                    abs(rot + np.pi - (acml_rot + 3 * np.pi)),
                )
            )
            dist_shift = float(np.sqrt(np.power(x_shift, 2) + np.power(y_shift, 2)))
            with open(PATH_ERRORDIST_PRM_LOC_X1, "a") as f1:
                write = csv.writer(f1)
                write.writerow([x_shift])
            with open(PATH_ERRORDIST_PRM_LOC_Y1, "a") as f1:
                write = csv.writer(f1)
                write.writerow([y_shift])
            with open(PATH_ERRORDIST_PRM_LOC_ROT, "a") as f1:
                write = csv.writer(f1)
                write.writerow([rot_shift])
            with open(PATH_ERRORDIST_PRM_LOC_DIST, "a") as f1:
                write = csv.writer(f1)
                write.writerow([dist_shift])
            with open(PATH_ERRORDIST_PRM_LOC, "a") as f1:
                write = csv.writer(f1)
                write.writerow([x_shift, y_shift, rot_shift, dist_shift])
            if rot_shift < 0.20:
                with open(PATH_ERRORDIST_PRM_LOC_FILTERED, "a") as f1:
                    write = csv.writer(f1)
                    write.writerow([dist_shift])
    # converting to degrees
    rot = rot / (2 * np.pi) * 360
    # the robot has a radius of about 20~25 so 5 pixels would also be fine
    robo_r = 4
    # drawing a circle with a cutout in a different color to represent the FoV
    draw_frame.pieslice(
        (x - robo_r, y - robo_r, x + robo_r, y + robo_r),
        start=0,
        end=360,
        fill=color_FoV,
        outline=color_FoV,
    )
    draw_frame.pieslice(
        (x - robo_r, y - robo_r, x + robo_r, y + robo_r),
        start=60 - rot,
        end=300 - rot,
        fill=color_outer,
        outline=color_outer,
    )

    return acml_x, acml_y, acml_rot


def initCV(pathWeights, map_path):
    """
    Initializes the networks and maps used by the camera based localization and object detection

    Args:
        pathWeights (str): Path to the weights of the YOLOv7 network

    Returns:
        A dictionary with following entries:
            - conf_network: the loaded weights from the network
            - base_info:
            - map_config: map configuration
            - obstacles_ws: detected workstations as obstacles
            - obstacles_movable: detected obstacles
            - names_movables: names/classes of the detected obstacles
            - map_ref: map reference
    """
    # Loads the models
    conf_network = get_conf_and_model(pathWeights)
    # gets the set of calibration data that needs to measured for each new png map
    base_info, map_config = get_base_info()
    # the gridmap is the locations of the workstations alligned with the 'grid' of the floortiles
    obstacles_ws, _ = get_obstacles_in_pixel_map(base_info)
    # # the movable obstacles that need to be placed in the robots way
    obstacles_movable, names_movables = get_obstacles_in_pixel_map(base_info, "movable")
    # # loading the png of the map and running the old detection system on it also configures the map to the right type
    map_ref, obstacles = initialize_map(map_path)
    # # removing the obstacles from the map, adding the workstations, removing old should not be needed if we have a new map, TODO new map add new obstacles such as klappbox, chair and box
    map_ref = modify_map(map_ref, obstacles, obstacles_ws, color=(255, 0, 255), convert_back_to_grey=True)
    # # adding the movable objects
    map_ref = modify_map(map_ref, [], obstacles_movable, color=(255, 0, 0), convert_back_to_grey=True)

    return {
        "conf_network": conf_network,
        "base_info": base_info,
        "map_config": map_config,
        "obstacles_ws": obstacles_ws,
        "obstacles_movable": obstacles_movable,
        "names_movables": names_movables,
        "map_ref": map_ref,
    }
