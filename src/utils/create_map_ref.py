import os
from copy import deepcopy
from real_robot_navigation.gridmap import get_obstacles_in_pixel_map
from real_robot_navigation.move_utils import initialize_map, modify_map
from real_robot_navigation.move_utils_cords import get_base_info

PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../..", ""))


def createMapRef(mapPath: str):
    """
    Creates a map reference based on a image which can be used in PRM etc.

    Args:
        mapPath(str): Path to the image from the map from which the map reference should be created

    Returns:
        map_ref: The map reference. Is a grayscale PIL image
        all_obst (list(Obstacle)): All detected obstacles and workstations on the map
    """
    base_info, _ = get_base_info()
    obstacles_ws, _ = get_obstacles_in_pixel_map(base_info)
    map_ref, obstacles = initialize_map(mapPath)
    map_ref = modify_map(map_ref, obstacles, obstacles_ws, color=(255, 255, 255))
    all_obst = deepcopy(obstacles_ws) + deepcopy(obstacles)  # + deepcopy(obstacles_movable)
    map_ref.save(f"{PATH}/maps/map_ref.png")

    return map_ref, all_obst
