import numpy as np
import rospy

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
    THRES = 0.3

    a = np.array([a1, a2])
    b = np.array([b1, b2])

    # t is scalar for a, s for b
    t, s = np.linalg.solve(np.array([a[1] - a[0], b[0] - b[1]]).T, b[0] - a[0])
    pointIntersect = (1 - t) * a[0] + t * a[1]
    dist = np.linalg.norm(a2 - pointIntersect)

    # Robotino has a diameter of approx 45 cm. For infering when a intersection would suggest that a collision
    # would also lead to a production stop, the radius and a little margin is used as the threshold. A intersection
    # above this threshold means that other robotinos can drive around that robotino which collided. A intersection under
    # this threshold means that the other robotinos also have to stop because they cant drive around the collison => production stop
    if dist < THRES:
        return pointIntersect, True
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
