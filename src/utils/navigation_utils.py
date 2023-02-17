import os
import rospy
from copy import deepcopy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from real_robot_navigation.gridmap import get_obstacles_in_pixel_map
from real_robot_navigation.move_utils import initialize_map, modify_map
from real_robot_navigation.move_utils_cords import (
    get_amcl_from_pixel_location,
    get_pixel_location_from_acml,
    get_base_info,
)
from autonomous_operation.PRM import Node, add_neighbours

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
    map_ref = modify_map(
        map_ref,
        obstacles,
        obstacles_ws,
        color=(255, 255, 255),
        convert_back_to_grey=True,
    )
    all_obst = deepcopy(obstacles_ws) + deepcopy(
        obstacles
    )  # + deepcopy(obstacles_movable)
    map_ref.save(f"{PATH}/maps/map_ref.png")

    return map_ref, all_obst


def pathToTraj(path: Path):
    """
    Converts a ROS Path message to a trajectory.

    Args:
        path (nav_msgs.Path): The Path message to convert

    Returns:
        trajectory (list of Nodes): The converted trajectory
    """
    trajectory = []
    base_info, _ = get_base_info()
    for node in path.poses:
        x = node.pose.position.x
        y = node.pose.position.y
        pixelCor = get_pixel_location_from_acml(x, y, *base_info)
        node = Node(int(pixelCor[0]), int(pixelCor[1]))
        trajectory.append(node)

    map_ref, _ = createMapRef(
        rospy.get_param("map_path", default=f"{PATH}/maps/FinalGridMapv2cleaned.png")
    )
    _, trajectory, _ = add_neighbours(N=20, nodes=trajectory, map_ref=map_ref)

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
    base_info, _ = get_base_info()

    for node in trajectory:
        if isinstance(node, Node):
            acmlNode = get_amcl_from_pixel_location(node.x, node.y, *base_info)
        else:
            acmlNode = get_amcl_from_pixel_location(node[0], node[1], *base_info)
        # Create a pose => Poses are the Nodes-equivalent in ROS's path
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = acmlNode[0]
        pose.pose.position.y = acmlNode[1]
        pose.pose.position.z = 0
        # Append pose to path
        path.poses.append(pose)

    return path
