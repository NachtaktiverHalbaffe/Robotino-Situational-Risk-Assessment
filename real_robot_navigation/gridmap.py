import numpy as np
from object_detection_modified import Obstacle
from move_utils import *
from move_utils_cords import *


def get_box_and_klapp():
    """
    This is a list of the true postions of the movable objects on the map\
    having it like this allows us to move them quickly. This is only used\
    when the localisation is running without the detection or we want to\
    measure the detections performance
    """
    x_shift_klapp = 0.0
    y_shift_klapp = 0.0

    x_shift_box = 0.0
    y_shift_box = 0

    objects = []
    klapp = {
        "name": "klapp",
        "size": np.array([0.465, 0.48, 0.35]),
        "location": np.array(
            [-1.20 + 0.35 / 2 + x_shift_klapp, 0.0, 1.20 - 0.48 / 2 + y_shift_klapp]
        ),
        "angle": 0,
    }
    box = {
        "name": "box",
        "size": np.array([0.482, 0.353, 0.238]),
        "location": np.array(
            [-1.20 + 0.238 / 2 + x_shift_box, 0.0, 2.40 - 0.353 / 2 + y_shift_box]
        ),
        "angle": 0,
    }
    hocker = {
        "name": "hocker",
        "size": np.array([0.51, 0.32, 0.32]),
        "location": np.array([0 + 0.16, 0.0, 1.80 - 0.16]),
        "angle": 0,
    }
    # chair = {'name': 'chair',
    #         'size': np.array([1.04, 0.70, 0.70]),
    #         'location': np.array([0.6, 0.0, 1.80]),
    #         'angle': 0}
    objects.append(klapp)
    objects.append(box)
    objects.append(hocker)

    names = [object["name"] for object in objects]
    return objects, names


def get_objects():
    """
    This is a list of the postions of the workstations alligned\
    to the real labs floor grid. This gets better results than using\
    the pre existing image recognition to place them on the map. Seeing\
    as we use this informaion to backcalculate the postion of the robot\
    this needed to pre accurate
    """
    objects = []

    wm_1 = {
        "name": "wm1",
        "size": np.array([0.96, 1.15, 0.80]),
        "location": np.array([-2.90, 0.0, 1.27]),
        "angle": 0,
    }
    wm_2 = {
        "name": "wm2",
        "size": np.array([0.96, 1.15, 0.80]),
        "location": np.array([-2.90, 0.0, 2.50]),
        "angle": 0,
    }
    wm_3 = {
        "name": "wm3",
        "size": np.array([0.96, 1.15, 0.80]),
        "location": np.array([-2.335, 0.0, 3.75]),
        "angle": 0.80,
    }
    wm_4 = {
        "name": "wm4",
        "size": np.array([0.96, 1.15, 0.80]),
        "location": np.array([-0.42 - 0.08, 0.0, 4.20]),
        "angle": 1.55,
    }
    wm_5 = {
        "name": "wm5",
        "size": np.array([0.96, 1.15, 0.80]),
        "location": np.array([1.03, 0.0, 4.00]),
        "angle": 2.22,
    }
    wm_6 = {
        "name": "wm5",
        "size": np.array([0.96 * 2, 1.15, 0.80]),
        "location": np.array([1.98, 0.0, 3.30]),
        "angle": 2.22,
    }

    objects.append(wm_1)
    objects.append(wm_2)
    objects.append(wm_3)
    objects.append(wm_4)
    objects.append(wm_5)
    objects.append(wm_6)
    names = [object["name"] for object in objects]
    return objects, names


def corners_from_center(x, y, rotation, sizes):
    # x = depth, y = horizontal shift
    # compute rotational matrix around yaw axis
    R = np.array(
        [
            [+np.cos(rotation), 0, +np.sin(rotation)],
            [0, 1, 0],
            [-np.sin(rotation), 0, +np.cos(rotation)],
        ]
    )

    # 3D bounding box dimensions
    l = sizes[0]
    w = sizes[1]
    h = sizes[2]

    # shift for each corner
    z_corners = [l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2]
    y_corners = [0, 0, 0, 0, -h, -h, -h, -h]
    x_corners = [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2]

    # rotate and translate 3D bounding box
    corners_3D = np.dot(R, np.array([x_corners, y_corners, z_corners]))
    corners_3D[0, :] = corners_3D[0, :] + y  # object.t(0);
    corners_3D[1, :] = corners_3D[1, :] + 0.43  # .42#object.t(1);
    corners_3D[2, :] = corners_3D[2, :] + x  # object.t(2);
    return corners_3D


def object_grid_to_pixel(pixel_map_info, object):
    """Here we are moving the postions from the locaion in the grid map into
    the postion on the pixel image"""
    left_close_corner = object["location"]
    # here we correct the coordinate system
    d_real = object["size"][2]
    w_real = object["size"][1]
    h_real = object["size"][0]
    size = (d_real, w_real, h_real)
    rotation = object["angle"]
    R = np.array(
        [
            [+np.cos(rotation), 0, +np.sin(rotation)],
            [0, 1, 0],
            [-np.sin(rotation), 0, +np.cos(rotation)],
        ]
    )
    dist_to_center = np.dot(R, np.array(size))
    depth = left_close_corner[0] + 2.35  # should be more
    horizontal_shift = left_close_corner[2] - 0.6
    if pixel_map_info[-2] == 48:
        depth = left_close_corner[0]  # +0.10# should be more
        horizontal_shift = left_close_corner[2] - 4.5
    corner_points = corners_from_center(depth, horizontal_shift, -rotation, size)

    corners = []
    corners.append(corner_points[:, 7])
    corners.append(corner_points[:, 6])
    corners.append(corner_points[:, 5])
    corners.append(corner_points[:, 4])

    pixel_corners = []
    for corner in corners:
        pixel_corners.append(
            get_pixel_location_from_acml(
                *convert_grid_to_robo(corner[2], corner[0]), *pixel_map_info
            )
        )

    return pixel_corners


def get_obstacles_in_pixel_map(pixel_map_info, objects="ws"):
    """
    Detects the obstacles in a given pixel map

    Args:
        pixel_map_info:
        object (str, optional): If workstations ("ws") or obstacles ("movable") should be returned

    Returns:
        obstacles (list of Obstacle): Either obstacles or workstations
        names (list of str): Names of the obstacles
    """
    if objects == "ws":
        grid_objects, names = get_objects()
    if objects == "movable":
        grid_objects, names = get_box_and_klapp()
    pixel_corners_all = []
    for object in grid_objects:
        # translate and rotate the objects to match the pixel map given by base info
        pixel_corners_all.append(object_grid_to_pixel(pixel_map_info, object))
    # have the objects be obstacles
    obstacles = []
    for corners in pixel_corners_all:
        obstacles.append(Obstacle(corners))

    return obstacles, names
