import numpy as np
import pandas as pd
import rospy
import rostopic
from pathlib import Path
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from utils.constants import Topics


def getCommonTraj():
    """
    Just a hardcoded list of the trajectories which other robotinos (controlled by Festos proprietary software)
    commonly drives. Derived from the work of Han Hu
    """
    # Location of workstations. Location of workstation and NOT their navigation markers are used
    # because it is assumed that they start driving from a docked position
    WS2 = (-1.03, -0.51)  # A
    WS3 = (-0.64, 3.23)  # D
    WS4 = (-1.86, 3.80)  # E
    WS5 = (-0.27, 0.70)  # B
    Robot = (-2.498, 0.219)  # Robotino parking point
    # Common navigation points
    C1 = (-1.619, 0.196)  # Near WS2 and Robot
    C2 = (-0.859, 1.015)  # Near WS2 and WS5
    C3 = (-0.941, 2.159)  # Near WS3
    C4 = (-2.492, 1.950)  # Near WS4 and WS5
    edges = [
        [C1, Robot],
        [WS2, C1],
        [C1, WS2],
        [C2, WS2],
        [C1, C2],
        [C2, WS3],
        [C2, C3],
        [C3, WS3],
        [C2, C4],
        [C3, C4],
        [C4, C1],
        [C4, WS5],
        [C4, WS4],
    ]

    return edges


def visualizeCommonTraj():
    """
    Send a markerarray to rviz where the common trajectories of other robotinos are visualized. Just for debugging
    """
    markerArray = MarkerArray()
    edges = getCommonTraj()
    i = 0
    for edge in edges:
        marker = Marker()
        startPoint = Point()
        endPoint = Point()
        startPoint.x = edge[0][0]
        startPoint.y = edge[0][1]
        endPoint.x = edge[1][0]
        endPoint.y = edge[1][1]

        marker.header.frame_id = "map"
        marker.type = marker.LINE_STRIP
        marker.ns = f"CommonTrajEdge{i}"
        marker.action = marker.ADD
        # marker scale
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        # marker position
        # marker.pose.position.x = 0.0
        # marker.pose.position.y = 0.0
        # marker.pose.position.z = 0.0

        marker.points.append(startPoint)
        marker.points.append(endPoint)
        markerArray.markers.append(marker)
        i += 1

    rospy.Publisher(
        Topics.MARKERS_COMMON_TRAJ.value, MarkerArray, queue_size=10
    ).publish(markerArray)


def getIntersection(a1, a2, b1, b2):
    """
    Returns the point of intersection of the lines passing through a2,a1 and b2,b1.

    Args:
        a1 (x,y-tuple): Start point on the first line. FIrst line is the finite sector
        a2 (x,y-tuple): End point on the first line. First line is the finite sector
        b1 (x,y-tuple): Start point on the second line
        b2 (x,y-tuple): End point on the second line

    Returns:
         x,y-tuple: Intersection point of two edges or infinite if lines are parallel
         bool: If line is intersecting
    """
    THRES = 0.2

    a = np.array([a1, a2])
    b = np.array([b1, b2])

    # t is scalar for a, s for b
    t, s = np.linalg.solve(np.array([a[1] - a[0], b[0] - b[1]]).T, b[0] - a[0])

    # print(f"t: {t} s:{s}")
    # s and t must be between 0 and 1 in general, otherwise the intersection is outside the edges
    # s is the sclaing factor for the vector of common edge onto which the proprietary robotinos drive
    # t is from the trajectory driven by the ego-robotino. So it must be between 0<= t <= THRES and
    # 1-THRES (=THRES_UPPER) <= t <=1 so only intersections in a certain range to the start and goal nodes,
    # where the robotino would collide or stop and lead to a production stop, are considered
    if (s <= 1 and s >= 0) and (t <= 1 and t >= (1 - THRES)):
        return (1 - t) * a[0] + t * a[1], True
    else:
        return np.inf, False


def closestNode(node, nodes):
    """
    Returns the nearest node to a given node

    Args:
        node (x,y-coordinate): Coordinate for which the nearest node should be calculated
        nodes ( list of x,y-coordinates): List of coordinates to search in

    Returns:
        The point with the nearest distance
    """
    node = node.array([node[0], node[1]])
    nodes = np.asarray(nodes)
    dist_2 = np.sum((nodes - node) ** 2, axis=1)
    return np.argmin(dist_2)


def loadErrorDistribution(path: str, bins: int = 5, precision: int = 8):
    """
    Loads error values from a CSV file and creates a error distribution from them

    Args:
        path (str): The path to the CSV file
        bins (int, optional): Number of segments into which the error distribution should be divided
        precision (int, optional): The number of decimal places to which the probabilities are rounded

    Returns:
        segments (list of float): The segments from the error distribution
        probabilities (list of float): The corresponding probability of each segment
    """
    file = Path(str(path).strip().replace("'", ""))
    if file.exists():
        data = pd.read_csv(path, header=None)
        probabilities, segments = np.histogram(data, bins=bins, density=False)
        # segments = np.round(segments, 2)
        probabilities = np.divide(probabilities, len(data))
        probabilities = np.round(probabilities, precision)
        try:
            rostopic.get_topic_class("/rosout")
            rospy.logdebug(
                f"[Crash and Remove] Loaded error distribution from {path}.\nSegments: {segments}\nProbabilities of segments: {probabilities}"
            )
        except:
            print(
                f"[Crash and Remove] Loaded error distribution from {path}.\nSegments: {segments}\nProbabilities of segments: {probabilities}"
            )

        return segments, probabilities
    else:
        raise FileExistsError("Error distribution doesn't exist")


def loadErrorDistributionLIDAR():
    segmentsAngles = [-0.17, -0.08, 0, 0.08, 0.17]
    probabilitiesAngles = [0.0153, 0.175, 0.7, 0.10, 0.008]
    segmentsDist = [-0.063, -0.037, -0.012, 0.012, 0.03]
    probabilitiesDist = [0.095, 0.2575, 0.29, 0.22, 0.12]

    try:
        rostopic.get_topic_class("/rosout")
        rospy.logdebug(
            f"[Crash and Remove] Loaded error distribution with LIDAR values."
        )
    except:
        print(f"[Crash and Remove] Loaded error distribution with LIDAR values.")
    return segmentsAngles, probabilitiesAngles, segmentsDist, probabilitiesDist


if __name__ == "__main__":
    loadErrorDistribution(
        "/home/ros/catkin_ws/src/robotino/logs/error_dist_csvs/lidar.csv"
    )
