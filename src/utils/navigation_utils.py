import json
import os
import rospy
import numpy as np
from copy import deepcopy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist, Point

from real_robot_navigation.gridmap import get_obstacles_in_pixel_map
from real_robot_navigation.move_utils import initialize_map, modify_map
from real_robot_navigation.move_utils_cords import (
    get_amcl_from_pixel_location,
    get_pixel_location_from_acml,
    get_base_info,
)
from autonomous_operation.PRM import Node, add_neighbours

PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../..", ""))
velocity_publisher_robot = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


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


def calc_command(target: Point, positionAssumption: Point, angleAssumption: float):
    """
    Calculates the distance to move and the angle to rotate to reach the specified target

    Args:
        target (geometry_msgs.Point): The coordinate to which the Robotino should move

    Returns:
        comm_angle (float): The angle the Robotino has to rotate
        distance (float): The distance the Robotino has to move
    """
    # from [-1, 1] to [-pi, pi]

    if angleAssumption < 0:  # from [-pi, pi] to [0, 2pi]
        angleAssumption = angleAssumption + (2 * np.pi)

    position = np.array([positionAssumption.x, positionAssumption.y])
    target = np.array([target.x, target.y])
    distance = np.linalg.norm(target - position)

    # comm_angle is the rotation angle -> positive angles represent counter clockwise rotation
    beta = np.arccos(np.abs(target[0] - position[0]) / distance)
    # important: in comparison to the "coordinates" of the image, the y-Axis is inverted, so it's different from my test-script
    if angleAssumption > np.pi:
        # print('current_angle in rad:', current_angle)
        angleAssumption = angleAssumption - 2 * np.pi
    # 4. Quadrant
    if target[0] - position[0] >= 0 and target[1] - position[1] < 0:
        comm_angle = 2 * np.pi - (beta + angleAssumption)
    # 1. Quadrant
    elif target[0] - position[0] >= 0 and target[1] - position[1] >= 0:
        comm_angle = beta - angleAssumption
    # 3. Quadrant
    elif target[0] - position[0] < 0 and target[1] - position[1] < 0:
        comm_angle = np.pi + beta - angleAssumption
    # 2. Quadrant
    else:
        comm_angle = np.pi - (beta + angleAssumption)
    if comm_angle > np.pi:
        comm_angle = comm_angle - 2 * np.pi

    return comm_angle, distance


def move(dist):
    """
    Moves a specified distance forward

    Args:
        dist: The distance the Robotino should move

    Returns:
        bool: If it moved successfully to the target (True) or failed (False)
    """
    speed = 0.10
    rospy.logdebug(f"[Control] Start moving {dist} meters")

    msg_test_forward = Twist()
    msg_test_forward.linear.x = speed
    msg_test_forward.linear.y = 0
    msg_test_forward.linear.z = 0
    msg_test_forward.angular.x = 0
    msg_test_forward.angular.y = 0
    msg_test_forward.angular.z = 0

    msg_test_stop = Twist()
    msg_test_stop.linear.x = 0
    msg_test_stop.linear.y = 0
    msg_test_stop.linear.z = 0
    msg_test_stop.angular.x = 0
    msg_test_stop.angular.y = 0
    msg_test_stop.angular.z = 0

    t0 = rospy.Time.now().to_sec()
    current_dist = 0
    dist = np.abs(dist)

    while current_dist < dist:
        velocity_publisher_robot.publish(msg_test_forward)
        t1 = rospy.Time.now().to_sec()
        current_dist = speed * (t1 - t0)

    velocity_publisher_robot.publish(msg_test_stop)


def rotate(angle):
    """
    Rotates the Robotino until it has reached the specified angle

    Args:
        angle (float): The angle to which the Robotino should rotate. Is in range e[0,2pi]

    Returns:
        bool: If it rotated successfully towards the target (True) or failed (False)
    """
    rospy.logdebug(f"[Control] Start rotating {angle} radians")
    rot_speed = 10 / (360) * (2 * np.pi)

    msg_test_stop = Twist()
    msg_test_stop.linear.x = 0
    msg_test_stop.linear.y = 0
    msg_test_stop.linear.z = 0
    msg_test_stop.angular.x = 0
    msg_test_stop.angular.y = 0
    msg_test_stop.angular.z = 0

    msg_test_rotate = Twist()
    msg_test_rotate.linear.x = 0
    msg_test_rotate.linear.y = 0
    msg_test_rotate.linear.z = 0
    msg_test_rotate.angular.x = 0
    msg_test_rotate.angular.y = 0
    if angle < 0:
        msg_test_rotate.angular.z = -rot_speed
    else:
        msg_test_rotate.angular.z = rot_speed

    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    angle = np.abs(angle)

    while current_angle < angle:
        velocity_publisher_robot.publish(msg_test_rotate)
        t1 = rospy.Time.now().to_sec()
        current_angle = rot_speed * (t1 - t0)

    velocity_publisher_robot.publish(msg_test_stop)


def navigateToPoint(
    target: Point,
    positionAssumption: Point,
    angleAssumption: float,
):
    """
    Navigates the real Robotino to an specified target by first rotating\
    towards the target and then driving forward. 

    Args:
        target (Point): The coordinate to which the Robotino should drive
        positionAssumption (Point): Assumption of the current/starting position
        angleAssumption (float): Assumption of the current angle of the robotino
    
    Returns:
        Publishes a response (Bool) to the topic  "/navigation_response"
    """
    comm_angle, comm_dist = calc_command(target, positionAssumption, angleAssumption)
    rospy.loginfo(f"[Control] Started moving to target {target}.")

    rotate(comm_angle)
    move(comm_dist)

    return comm_angle, comm_dist
