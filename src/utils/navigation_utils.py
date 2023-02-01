import os
from copy import deepcopy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

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
    map_ref = modify_map(map_ref, obstacles, obstacles_ws, color=(255, 255, 255), convert_back_to_grey=True)
    all_obst = deepcopy(obstacles_ws) + deepcopy(obstacles)  # + deepcopy(obstacles_movable)
    map_ref.save(f"{PATH}/maps/map_ref.png")

    return map_ref, all_obst


def pathToTraj(path: Path):
    """
    Converts a ROS Path message to a trajectory.

    Args:
        path (nav_msgs.Path): The Path message to convert

    Returns:
        trajectory (list of x,y-coordinates): The converted trajectory
    """
    trajectory = []

    for node in path.poses:
        x = node.pose.position.x
        y = node.pose.position.y
        trajectory.append((x, y))

    return trajectory


def trajToPath(trajectory: list):
    """
    Converts a trajectory to a ROS path message

    Args:
        trajectory (list of x,y-coordinates): The trajectory to convert

    Returns:
        path (nav_msgs.Path): The converted path message
    """
    path = Path()
    path.header.frame_id = "map"

    for node in trajectory:
        # Create a pose => Poses are the Nodes-equivalent in ROS's path
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = node[0]
        pose.pose.position.y = node[1]
        pose.pose.position.z = 0
        # Append pose to path
        path.poses.append(pose)

    return path
