import numpy as np


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
    a = np.array([a1[0], a1[1]], [a2[0], a2[1]])
    b = np.array([b1[0], b1[1]], [b2[0], b2[1]])

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
