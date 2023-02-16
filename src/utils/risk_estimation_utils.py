import numpy as np
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from utils.constants import Topics


def getCommonTraj():
    """
    Just a hardcoded list of the trajectories which other robotinos (controlled by Festos proprietary software) commonly drives
    """
    edges = [
        [(0.0034, 0.002), (3.4, 0.49)],  # 0 => C1
        [(3.13, 0.57), (3.4, 0.49)],  # C1 => A
        [(3.4, 0.49), (1.09, 3.19)],  # C1 => C2
        [(2.5, 1.13), (2.22, 1.36)],  # C2 => D
        [(3.4, 0.49), (1.09, 3.19)],  # C2 => C3
        [(0.82, 2.94), (1.074, 3.07)],  # C3 => D
        [(0.71, 2.74), (0.67, 2.51)],  # C2 => C4
        [(0.71, 2.74), (0.69, 2.49)],  # C4 => B
        [(3.16, 0.913), (0.73, 2.74)],  # C4 => C5
        [(0.48, 0.71), (2.49, 2.51)],  # C5 => E
    ]

    return edges


def visualizeCommonTraj():
    """
    Send a markerarray to rviz where the common trajectories of other robotinos are visualized
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
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.points.append(startPoint)
        marker.points.append(endPoint)
        markerArray.markers.append(marker)

    rospy.Publisher(Topics.MARKERS_COMMON_TRAJ.value, MarkerArray, queue_size=10).publish(markerArray)


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
    a = np.array(a1, a2)
    b = np.array(b1, b2)

    t, _ = np.linalg.solve(np.array([a[1] - a[0], b[0] - b[1]]).T, b[0] - a[0])

    if t < 1 and t >= 0:
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
