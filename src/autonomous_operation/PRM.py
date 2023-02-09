import json
import math
import time
import numpy as np
from PIL import Image, ImageDraw
from real_robot_navigation.move_utils_cords import (
    get_amcl_from_pixel_location,
    get_base_info,
    get_pixel_location_from_acml,
)
import rospy
import copy
import time
import sys, os

import pickle

PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../..", ""))

sys.setrecursionlimit(2000)

# TODO: these parameters should also be part of the config file...
# amount of nodes sampled in the map
N_NODES = 300
# amount of neighbours for each node -> the "k" of "k-nearest neighbour"
N_NEIGHBOURS = 20
# 5/6 seems to represent the robotino quite good
RADIUS_ROBOT = 8

# !! Don't change these COLOR values (they are used in Environment.py hardcoded..) !!
NODE_COLOR = 160
EDGE_COLOR = 50
START_COLOR = 205
GOAL_COLOR = 120
TRAJ_COLOR = 100
TRAJ_NODE_COLOR = 210
ADV_TRAJ_COLOR = 35
ADV_TRAJ_NODE_COLOR = 110
COLOR_UNKNOWN_AREA = 40

COST_BLACK_PIXEL = 1
COST_WHITE_PIXEL = 99999


class Node:
    """
    Nodes for PRM - Every node has an arbitrary number of neighbours (other nodes that are conneted via edges)

    Attributes:
        x (int): x-coordinate of the node
        y (int): y-coordinate of the node
    """

    def __init__(self, x, y):
        self.x = x
        """x-coordinate of the node"""
        self.y = y
        """y-coordinate of the node"""
        self.coordinates = np.array([x, y])
        """Coordinate of the node"""
        self.neighbours = []
        """Connected Nodes"""
        self.edges = []
        """Edges to which the node is connected"""
        # for deijkstra
        self.tentative_dist = math.inf
        """Deijkstra distance"""
        self.visited = False
        """If node was already visited by dijsktra"""
        self.predecessor = None
        """Predecessor Node in deijkstra"""

    def __str__(self):
        return str(self.coordinates)

    __repr__ = __str__  # X kind of a bad practice


class Edge:
    """
    Edges for PRM - Every edge has exactly two nodes (which it connects to)

    Attributes:
        node1 (Node): First/start node of the edge
        node2 (Node): Second/end node of the edge
        length: length of the edge -> doesn't regard the "color" of the traversed pixels
        cost (int): Cost value of the edge -> depends on length of the edge and the color of traversed pixels. Defaults to 0
    """

    def __init__(self, node1, node2, length, cost=0):
        self.node1 = node1
        """First/start node of the edge"""
        self.node2 = node2
        """Second/end node of the edge"""
        self.length = length
        """Length of the edge -> doesn't regard the "color" of the traversed pixels"""
        self.cost = cost
        """Cost value of the edge -> depends on length of the edge and the color of traversed pixels. Defaults to 0"""
        self.edge_points = []

    def set_cost(self, cost):
        self.cost = cost

    def __str__(self):
        node1_str = str(self.node1)
        node2_str = str(self.node2)
        return str(
            node1_str + "<--->" + node2_str + "  length: " + "%.2f" % self.length + " cost: " + "%.2f" % self.cost
        )

    __repr__ = __str__  # X kind of a bad practice


def add_nodes(map_ref, N, obstacles, start=None, goal=None):
    """
    Adds the nodes of the graph into the reference map for PRM -> since the robot has a width, we don't \
    place the nodes close to the obstacles (->RADIUS_ROBOT). This is done by randomly generating nodes and\
    adding them if no obstacles are within RADIUS_ROBOT

    Args:
        map_ref: reference map
        N (int): amount of nodes to sample
        obstacles(list(Obstacle)): Obstacle-objects (results from object detection)
        start (optional): if we want a specific position to be sampled as start we have to pass it here (as tuple)
        goal (optional): if we want a specific position to be sampled as goal we have to pass it here (as tuple)

    Returns:
        im_with_nodes: PIL Image of the reference map mit the nodes added as grey dots - just for visualization
        nodes (list(Node)): Nodes which are added
    """

    im_width = map_ref.size[0]
    im_height = map_ref.size[1]

    pixels = np.array(map_ref.getdata()).reshape((im_height, im_width))
    N_nodes = 0

    nodes = []
    # Add start and goal to list
    if start and goal:
        nodes.append(Node(start[0], start[1]))
        nodes.append(Node(goal[0], goal[1]))
    for node in _loadMarkersFromJson():
        if get_node_with_coordinates(nodes, node) == None:
            nodes.append(Node(node[0], node[1]))

    N_nodes = len(nodes)
    while N_nodes < N:
        # Generate random node
        random_x = np.random.randint(im_width)
        random_y = np.random.randint(im_height)

        # Calculate distances of all obstacles to random pnode
        distances = []
        for obstacle in obstacles:
            distances.append(obstacle.distance_to_point((random_x, random_y)))
        dist = np.min(np.array(distances))

        if (
            (dist > RADIUS_ROBOT + 4)
            and pixels[random_y][random_x] != 255
            and pixels[random_y][random_x] != NODE_COLOR
        ):
            # Only add node if no obstacle is too near
            pixels[random_y][random_x] = NODE_COLOR
            N_nodes += 1
            nodes.append(Node(random_x, random_y))

    im_with_nodes = Image.fromarray(pixels.astype("uint8"), mode="L")
    rospy.logdebug(f"[PRM] Number of Nodes: {len(nodes)}")
    return im_with_nodes, nodes


def calculate_edge_costs(map_ref, edges, obstacles=None, prot=False):
    """ Recalculates the cost of all passed edges based on the passed reference map and obstacles.

    PROBLEM of calculate_edge_cost: Can only detect collisions with known obstacles\
    therefore there is another function calc_cost, see comments there for more information

    Args:
        map_ref: reference map
        edges (list(Edge)): Edge-objects that should be recalculated
        obstacles (list(Obstacle), optional): Known obstacles in map_ref. Defaults to None
        prot (bool, optional): If the function is called to calculate the weights for the protagonist, we have to return (only) the\
            edges, which have been influenced by the protagonist. Defaults to False

    Returns: 
        edges_prot (dict): Edges that has been changed by the protagonist (if prot=True)
    """
    # t0 = time.perf_counter()
    obstacles_to_remove = []
    map_size = map_ref.size
    map_matrix = np.reshape(np.array(map_ref.getdata()), (map_size[0], map_size[1]))
    edges_prot = {}

    for edge in edges:
        if edge.cost >= COST_WHITE_PIXEL:
            # if edge cost is causing a collision already (edge cost >= COST_COLLISION) we don't have to recalculate
            # its weight (after prot it can only be even higher)
            continue
        else:
            edge_cost = 0
            distances = []
            close_object = False
            i = 0
            rounded_edge_points = np.round(np.array(edge.edge_points)).astype(int)
            rounded_edge_points_x = rounded_edge_points[:, 0]
            rounded_edge_points_y = rounded_edge_points[:, 1]
            grayscale_vals = map_matrix[rounded_edge_points_y, rounded_edge_points_x]

            for edge_point in edge.edge_points:
                distances = []
                grayscale_val = grayscale_vals[i]
                pixel_cost = grayscale_to_cost(grayscale_val)
                edge_cost += pixel_cost
                # check only for every second point on edge to save some time
                i += 1

                # Check if an obstacle is near the edge so the robotino would collide if it drives along the edge
                if obstacles:
                    if (i % 2 == 0) and (not close_object) and (edge_cost < COST_WHITE_PIXEL):
                        for obstacle in obstacles:
                            distances.append(obstacle.distance_to_point(edge_point))
                        if np.min(np.array(distances)) < RADIUS_ROBOT:
                            close_object = True
                            obstacles_to_remove.append(
                                (
                                    obstacles[np.argmin(np.array(distances))],
                                    np.argmin(np.array(distances)),
                                )
                            )

            if close_object:
                # if the next object border is closer than the robot radius, driving on the current edge causes a collision -> adjust edge cost
                edge_cost += COST_WHITE_PIXEL

            if prot:
                if edge_cost >= COST_WHITE_PIXEL:
                    edges_prot[edge] = edge.cost

            edge.cost = edge_cost
    return edges_prot, obstacles_to_remove


def interpolate_segment(segment):
    """
    Interpolate the passed segment (every second pixel only for performance reasons) -> is used when checking for distance to obstacles

    Args:
        segment (Edge): segment of a trajectory (from one waypoint (Node) to another - this is also just one Edge)

    Returns:
        segment_interpolated(list(coordinates)): Coordinates that interpolates the segment
    """
    p1 = segment[0]
    p2 = segment[1]

    interpol_stepsize = 2
    length = np.linalg.norm(np.array(p2) - np.array(p1))

    n_interpol_steps = int(length / interpol_stepsize)
    segment_interpolated = []

    if n_interpol_steps != 0:
        # calculate step size
        step_x = (p2[0] - p1[0]) / n_interpol_steps
        step_y = (p2[1] - p1[1]) / n_interpol_steps

        for i in range(0, n_interpol_steps + 1):
            segment_interpolated.append((np.round(p1[0] + i * step_x), np.round(p1[1] + i * step_y)))
    else:
        segment_interpolated.append((p1[0], p1[1]))
        segment_interpolated.append((p2[0], p2[1]))

    return segment_interpolated


def calc_nearest_dist(traj, obstacles):
    """
    Calculates the shortest distances for every segment of the passed trajectory to all of the known obstacles\
    analytically (using the borders of the obstacles)

    Args:
        traj: trajectory of which the shortest distance is supposed to be calculated
        obstacles (list(Obstacle)): Obstacles

    Returns:
        closest_distances_all(list(float)): Distance values (one value for every segment)
        closest_points_all(list(Point)): Every tuple represents location of the point on the corresponding\ 
                                                            segment that has the smallest distance to an object
    
    Small note 
    ---
    In the end the algorithm will just make use of the minimum value of the "closest_distances_all"\
    i.e. considering only the nearest (point) to an obstacle
    """
    closest_distances_all = []
    closest_points_all = []
    for i in range(0, len(traj) - 1):
        segment = [traj[i].coordinates, traj[i + 1].coordinates]
        segment_interpolated = interpolate_segment(segment)

        closest_distances = []
        closest_points = []
        for obstacle in obstacles:
            for interp_point in segment_interpolated:
                dist = obstacle.distance_to_point(interp_point)
                closest_distances.append(dist)
                closest_points.append(np.array(interp_point))

        indexes_closest_dist = np.argsort(np.array(closest_distances))
        closest_distances = np.array(closest_distances)[indexes_closest_dist]
        closest_points = np.array(closest_points)[indexes_closest_dist]
        closest_distances_all.append(closest_distances.tolist())
        closest_points_all.append(closest_points.tolist())

    return closest_distances_all, closest_points_all


def add_neighbours(map_ref, nodes, N):
    """
    Applies the k-nearest neighbours algorithm to construct a graph

    Args:
        map_ref: reference map
        nodes (list(Node)): nodes to build the graph of
        N (int): "k" of k-nearest neighbour -> amount of other nodes to connect for each node

    Returns:
        map_ref_copy1: reference map (unchanged)
        nodes (list(Node)): list of nodes, that now have neighbours and edges
        edges_all (list(Edge)): All edges in the graph with their cost values
    """
    deepcopy_total_time = 0
    calc_cost_total_time = 0
    draw_line_total_time = 0
    calc_all_distances_total_time = 0

    t0 = time.perf_counter()
    map_ref_copy_1 = copy.deepcopy(map_ref)
    deepcopy_total_time += time.perf_counter() - t0

    map_ref_draw = ImageDraw.Draw(map_ref_copy_1)
    edges_all = []

    t0 = time.perf_counter()
    map_ref_copy_2 = copy.deepcopy(map_ref)
    deepcopy_total_time += time.perf_counter() - t0

    for node_i in range(0, len(nodes)):
        available_nodes = copy.copy(nodes)
        available_nodes.remove(nodes[node_i])
        for neighbour in nodes[node_i].neighbours:
            available_nodes.remove(neighbour)
        for k in range(0, N):
            t0 = time.perf_counter()
            map_visu = copy.deepcopy(map_ref_copy_2)
            deepcopy_total_time += time.perf_counter() - t0

            map_visu_draw = ImageDraw.Draw(map_visu)

            if len(available_nodes) != 0:
                shortest_dist = math.inf
                if len(nodes[node_i].neighbours) >= N:
                    edge_distances = []
                    for edge in nodes[node_i].edges:
                        edge_distances.append(edge.length)
                    shortest_dist = np.max(edge_distances)

                # most time consuming part: --------------------------v
                t3 = time.perf_counter()
                closest_neighbour = None
                for available_node_j in range(0, len(available_nodes)):
                    dist = np.linalg.norm(nodes[node_i].coordinates - available_nodes[available_node_j].coordinates)
                    if dist < shortest_dist:
                        shortest_dist = dist
                        closest_neighbour = available_nodes[available_node_j]
                calc_all_distances_total_time += time.perf_counter() - t3
                # ----------------------------------------------------^

                if closest_neighbour is not None:
                    # calculate length and cost of edge
                    t2 = time.perf_counter()
                    map_ref_draw.line(
                        [
                            (
                                nodes[node_i].coordinates[0],
                                nodes[node_i].coordinates[1],
                            ),
                            (
                                closest_neighbour.coordinates[0],
                                closest_neighbour.coordinates[1],
                            ),
                        ],
                        fill=EDGE_COLOR,
                    )
                    map_visu_draw.line(
                        [
                            (
                                nodes[node_i].coordinates[0],
                                nodes[node_i].coordinates[1],
                            ),
                            (
                                closest_neighbour.coordinates[0],
                                closest_neighbour.coordinates[1],
                            ),
                        ],
                        fill=EDGE_COLOR,
                    )
                    draw_line_total_time += time.perf_counter() - t2
                    t1 = time.perf_counter()
                    cost = calc_cost(map_ref_copy_2, map_visu, nodes[node_i].coordinates)
                    calc_cost_total_time += time.perf_counter() - t1
                    # create new edge
                    edge = Edge(
                        nodes[node_i],
                        closest_neighbour,
                        length=shortest_dist,
                        cost=cost,
                    )
                    if edge not in edges_all:
                        edges_all.append(edge)
                    # append neighbour and edge for node_i
                    nodes[node_i].neighbours.append(closest_neighbour)
                    nodes[node_i].edges.append(edge)
                    # append neighbour and edge for closest_neighbour
                    closest_neighbour.neighbours.append(nodes[node_i])
                    closest_neighbour.edges.append(edge)

                    map_ref_draw.point(
                        [(nodes[node_i].coordinates[0], nodes[node_i].coordinates[1])],
                        fill=NODE_COLOR,
                    )
                    map_ref_draw.point(
                        [
                            (
                                closest_neighbour.coordinates[0],
                                closest_neighbour.coordinates[1],
                            )
                        ],
                        fill=NODE_COLOR,
                    )

                    available_nodes.remove(closest_neighbour)

    for edge in edges_all:
        p1 = edge.node1.coordinates
        p2 = edge.node2.coordinates

        length = np.linalg.norm(p2 - p1)

        step_x = (p2[0] - p1[0]) / length
        step_y = (p2[1] - p1[1]) / length

        edge_points = [p1]
        for i in range(0, int(length) + 1):
            point = np.array([np.round(p1[0] + i * step_x), np.round(p1[1] + i * step_y)])
            if not (point == edge_points[-1]).all():
                edge.edge_points.append((np.round(p1[0] + i * step_x), np.round(p1[1] + i * step_y)))

    return map_ref_copy_1, nodes, edges_all


def deijkstra(nodes, start, goal):
    """
    This function applies the dijkstra algorithm and returns a list of nodes that represent the optimal trajectory

    Args:
        nodes: list of Node-Objects that need the have neighbours added (->graph) at this point
        start (Node): Start node of the calculated trajectory
        goal (Node): Goal node of the calculated trajectory

    Returns: 
        "shortest" trajectory based on the passed graph. "shortest" means optimal in terms of the edge costs, \
        not necessarily in euclidean length.

    Small note
    ---
    I didn't account for "impossible" trajectories where deijkstra can't find the goal -> islands in graph
    """
    # init
    nodes_copy = copy.copy(nodes)
    nodes_unvisited = nodes
    current_node = start
    start.tentative_dist = 0
    done = False
    trajectory = []
    # loop
    while not done:
        for neighbour in current_node.neighbours:
            if not neighbour.visited:
                pos_new_tentative = find_common_edge(current_node, neighbour).cost + current_node.tentative_dist
                if pos_new_tentative < neighbour.tentative_dist:
                    neighbour.tentative_dist = pos_new_tentative
                    neighbour.predecessor = current_node
        current_node.visited = True
        nodes_unvisited.remove(current_node)

        # check if we reached the goal already and build up optimal trajectory
        if current_node == goal or len(nodes_unvisited) == 0:
            # found the shortest path or no path # XX todo: what if we found no path? -> len(nodes_unvisited) == 0
            done = True
            traj_node = goal
            trajectory.append(goal)
            while traj_node.predecessor is not None:
                trajectory.insert(0, traj_node.predecessor)
                traj_node = traj_node.predecessor

        # not finished yet... find the next node we want to visit
        if not len(nodes_unvisited) == 0:
            smallest_tent = math.inf
            smallest_tent_node = None
            for unv_node in nodes_unvisited:
                if unv_node.tentative_dist < smallest_tent:
                    smallest_tent = unv_node.tentative_dist
                    smallest_tent_node = unv_node
            current_node = smallest_tent_node

    for node in nodes_copy:
        node.tentative_dist = math.inf
        node.visited = False
        node.predecessor = None
    return trajectory


# recursive function to "follow" a line and sum up its costs
def calc_cost(ref_map, colored_map, coordinates):
    """
    This function "graphically" calculates the costs of an edge with the floodfill algorithm\
    -> this function is still used to calculate the cost of edges. ALTHOUGH there is also the function\
    "calculate_edge_costs" which also is used to calculate the costs. However these function are both needed for now
    because:
        - calc_cost is still needed to detect collisions with white pixels that are not recognized as part of an known
            obstacle.
            PROBLEM of calc_cost: The width of the robot is not considered in this calculation
        - whereas "calculate_edge_costs" considers the width for calculation but can only detect collisions with known
            obstacles
    
    PROBLEM of calculate_edge_cost: Can only detect collisions with known obstacles
    
    Args:
        ref_map: reference map
        colored_map: reference map with additionally one single edge marked as greyscale (for floodfill algorithm)
        coordinates: start point of the marked edge
    
    Returns: 
        edge cost
    """
    im_width = colored_map.size[0]
    im_height = colored_map.size[1]

    if im_width - 1 < coordinates[0] or im_height - 1 < coordinates[1] or (coordinates < 0).any():
        return 0

    if colored_map.getpixel((int(coordinates[0]), int(coordinates[1]))) == EDGE_COLOR:
        colored_map.putpixel((coordinates[0], coordinates[1]), 0)
        # this color can be anything but EDGE_COLOR
        grayscale_val = ref_map.getpixel((int(coordinates[0]), int(coordinates[1])))
        # we can just pick the first value of the RGB value, since it is a grayscale anyway
        edge_cost = grayscale_to_cost(grayscale_val)

        # below
        edge_cost += calc_cost(ref_map, colored_map, np.array([coordinates[0], coordinates[1] + 1]))
        # above
        edge_cost += calc_cost(ref_map, colored_map, np.array([coordinates[0], coordinates[1] - 1]))
        # right
        edge_cost += calc_cost(ref_map, colored_map, np.array([coordinates[0] + 1, coordinates[1]]))
        # left
        edge_cost += calc_cost(ref_map, colored_map, np.array([coordinates[0] - 1, coordinates[1]]))
        # right below
        edge_cost += calc_cost(ref_map, colored_map, np.array([coordinates[0] + 1, coordinates[1] + 1]))
        # right above
        edge_cost += calc_cost(ref_map, colored_map, np.array([coordinates[0] + 1, coordinates[1] - 1]))
        # left below
        edge_cost += calc_cost(ref_map, colored_map, np.array([coordinates[0] - 1, coordinates[1] + 1]))
        # left above
        edge_cost += calc_cost(ref_map, colored_map, np.array([coordinates[0] - 1, coordinates[1] - 1]))

        return edge_cost

    return 0


def grayscale_to_cost(grayscale):
    """
    Maps all relevant grayscale values to a specific cost value. For now only binary costs is used (white or black)
    -> can be adjusted to consider grayscale values

    Args:
        Grayscale: color value of the specific pixel -> e[0, 255]

    Returns:
        cost(int): Calculated cost value
    """
    cost = 0

    if grayscale == 255:
        cost = COST_WHITE_PIXEL
    elif grayscale == COLOR_UNKNOWN_AREA:
        # unknown territory
        cost = 10
    elif grayscale == 0:
        cost = COST_BLACK_PIXEL
    # --------------------- will be reworked
    elif grayscale == 234:
        cost = 1
    elif grayscale == 214:
        cost = 10
    elif grayscale == 194:
        cost = 16
    elif grayscale == 174:
        cost = 20
    elif grayscale == 154:
        cost = 5000
    else:
        rospy.logwarn("[PRM] Graph leads over a pixel color that should not exist in the reference-map !!!!!!")

    return cost


def find_common_edge(node1, node2):
    """
    Finds a common edge between two nodes if one exists

    Args:
        node1 (Node): Node-object
        node2 (Node): Node-object

    Returns:
        common_edge (Edge): Common Edge or None
    """
    common_edge = None
    for edge in node1.edges:
        if node2 in [edge.node1, edge.node2]:
            common_edge = edge

    return common_edge


def get_traj_edges(traj):
    """
    Returns all edges in the passed trajectory (the trajectory is just a list of nodes)

    Args:
        traj (list(Node)): Trajectory from which the edges should be returned

    Returns:
        edges (list(Edge)): Edges of trajectory
    """
    edges = []
    total_costs = 0

    for i in range(0, len(traj) - 1):
        common_edge = find_common_edge(traj[i], traj[i + 1])
        if common_edge not in edges:
            if common_edge:
                edges.append(common_edge)
                total_costs += common_edge.cost
                if common_edge.cost >= COST_WHITE_PIXEL:
                    pass

    return edges


def optimize_trajectory(map, traj):
    """
    !! Note !!
    ---
    This function is not used since it causes huge segments in the trajectory which is undesired.

    ---
    Checks if the nodes of the trajectory allows for a more "direct" path with less costs if they are all\
    connected. 

    Args:
        map: reference map
        traj: list of Node-objects

    Returns:
        traj_opt (list(Nodes)): optimized trajectory 
        map: Reference map
    """
    traj_replace = []
    for traj_node in traj:
        traj_replace.append(Node(traj_node.coordinates[0], traj_node.coordinates[1]))

    add_neighbours(map, traj_replace, len(traj) - 1)
    traj_opt = deijkstra(traj_replace, traj_replace[0], traj_replace[-1])
    draw_traj(map, traj_opt, False)

    return map, traj_opt


def draw_traj(map_visu, traj, forAgent, color=None):
    """
    This function draws the trajectory into the map and is a necessary step to "build" a state for the agents as well as
    for visualization purposes

    Args:
        map_visu: Copy of reference map
        traj (list(Node)): Trajectory which should be drawn
        forAgent (bool): Changes how the trajectory is depicted depending on if it is needed for the adversary (True) or just for visualization purpose (False)
        color (optional): If we pass a color here, the image will be saved as an RBG image and the trajectory has the color. Defaults to None

    Returns:
        map_visu: map with the trajectory drawn as greyscale or if passed in color
    """
    traj_color = copy.copy(TRAJ_COLOR)

    if color:
        # Convert image to RGB-img
        map_visu = map_visu.convert("RGB")
        traj_color = color
    im_draw = ImageDraw.Draw(map_visu)

    # Draw lines of trajectory
    for i in range(0, len(traj) - 1):
        im_draw.line(
            [
                (traj[i].coordinates[0], traj[i].coordinates[1]),
                (traj[i + 1].coordinates[0], traj[i + 1].coordinates[1]),
            ],
            fill=traj_color,
        )

    for traj_node in traj:
        if not forAgent:
            im_draw.point(
                [(traj_node.coordinates[0], traj_node.coordinates[1])],
                fill=TRAJ_NODE_COLOR,
            )

    # Draw points of nodes
    if not forAgent:
        points_off_x = [-1, 1, -1, 1, -1, 1, 0, 0, 0]
        points_off_y = [-1, -1, 1, 1, 0, 0, 1, -1, 0]
        for i in range(0, 9):
            im_draw.point(
                [
                    (
                        traj[0].coordinates[0] + points_off_x[i],
                        traj[0].coordinates[1] + points_off_y[i],
                    )
                ],
                fill=START_COLOR,
            )
            im_draw.point(
                [
                    (
                        traj[-1].coordinates[0] + points_off_x[i],
                        traj[-1].coordinates[1] + points_off_y[i],
                    )
                ],
                fill=GOAL_COLOR,
            )
    else:
        im_draw.point([(traj[0].coordinates[0], traj[0].coordinates[1])], fill=START_COLOR)
        points_off_x = [-1, 1, -1, 1, -1, 1, 0, 0, 0]
        points_off_y = [-1, -1, 1, 1, 0, 0, 1, -1, 0]
        for i in range(0, 9):
            im_draw.point(
                [
                    (
                        traj[0].coordinates[0] + points_off_x[i],
                        traj[0].coordinates[1] + points_off_y[i],
                    )
                ],
                fill=START_COLOR,
            )
        # intermediate notes in the trajectory will also be considered in the state with a different color
        if len(traj) > 2:
            for traj_node in traj[1 : (len(traj) - 1)]:
                im_draw.point(
                    [(traj_node.coordinates[0], traj_node.coordinates[1])],
                    fill=TRAJ_NODE_COLOR,
                )
    if color:
        return map_visu


# def visualize_adv_traj(map_visu, traj):
#
#     im_draw = ImageDraw.Draw(map_visu)
#     for i in range(0, len(traj)-1):
#         im_draw.line([(traj[i].coordinates[0], traj[i].coordinates[1]), (traj[i+1].coordinates[0], traj[i+1].coordinates[1])], fill=ADV_TRAJ_COLOR)
#     for traj_node in traj:
#         im_draw.point([(traj_node.coordinates[0], traj_node.coordinates[1])], fill=ADV_TRAJ_NODE_COLOR)


def _loadMarkersFromJson():
    """
    Loads the workstation markers from the corresponding JSON file so it can be appended to the Nodes in the PRM
    """
    wsNodes = []
    file = f"{PATH}/maps/markers.json"
    base_info, _ = get_base_info()

    if os.path.isfile(file):
        with open(file) as jsonfile:
            target_identified = json.load(jsonfile)

        for key in target_identified.keys():
            x = target_identified[key]["NavPosed"]["posedstamped"]["pose"]["position"]["x"]
            y = target_identified[key]["NavPosed"]["posedstamped"]["pose"]["position"]["y"]
            node = get_pixel_location_from_acml(x, y, *base_info)
            wsNodes.append((int(node[0]), int(node[1])))
    return wsNodes


def get_node_with_coordinates(nodes, xycoords):
    """
    Searches for a node with the passed coordinates in the passed list

    Args:
        nodes (list(Node)): Nodes on which the coordinates should be searched
        coordinates (int, int): tuple of x and y value (pixels in image)

    Returns:
        ret_node (Node): Found Node or None
    """
    ret_node = None

    for node in nodes:
        if (node.coordinates == xycoords).all():
            ret_node = node
    return ret_node


def calc_adv_traj(map_ref, adv_traj_coordinates, obstacles):
    """
    This function is needed when we generate a trajectory outside the PRM with the adversary

    Args:
        map_ref: reference map
        adv_traj_coordinates (list(Node)): Manipulated trajectory by adversary
        obstacles (list(Node)): Obstacles which are known to the map

    Returns:
        nodes (list(Node)):  Nodes from the calculated trajectory
        edges (list(Edge)): Edges from the calculated trajectory
    """
    x_max = map_ref.size[0] - 1
    y_max = map_ref.size[1] - 1

    nodes = []

    for coordinate in adv_traj_coordinates:
        nodes.append(Node(coordinate[0], coordinate[1]))

    for i in range(0, len(nodes) - 1):
        nodes[i].neighbours.append(nodes[i + 1])
        nodes[i + 1].neighbours.append(nodes[i])
        length = np.linalg.norm(nodes[i].coordinates - nodes[i + 1].coordinates)
        # map_visu = copy.deepcopy(map_ref)
        # map_visu_draw = ImageDraw.Draw(map_visu)
        # map_visu_draw.line([(nodes[i].coordinates[0], nodes[i].coordinates[1]), (nodes[i+1].coordinates[0], nodes[i+1].coordinates[1])], fill=EDGE_COLOR)
        # cost = calc_cost(map_ref, map_visu, nodes[i].coordinates)
        new_edge = Edge(nodes[i], nodes[i + 1], length, 0)
        nodes[i].edges.append(new_edge)
        nodes[i + 1].edges.append(new_edge)

    edges = get_traj_edges(nodes)

    for edge in edges:
        p1 = edge.node1.coordinates
        p2 = edge.node2.coordinates

        length = np.linalg.norm(p2 - p1)

        step_x = (p2[0] - p1[0]) / length
        step_y = (p2[1] - p1[1]) / length

        edge_points = [p1]
        for i in range(0, int(length) + 1):
            x = np.round(p1[0] + i * step_x)
            y = np.round(p1[1] + i * step_y)

            if x > x_max:
                x = x_max
            if y > y_max:
                y = y_max
            if x < 0:
                x = 0
            if y < 0:
                y = 0

            point = np.array([x, y])
            if not (point == edge_points[-1]).all():
                edge.edge_points.append((point[0], point[1]))

    edges_prot, obstacles_to_remove = calculate_edge_costs(map_ref, edges, obstacles)

    return nodes, edges, obstacles_to_remove


def findNearestNode(nodes, xCor, yCor):
    t0 = time.perf_counter()
    minDist = math.inf
    nearestNode = None
    cor = np.array([xCor, yCor])
    for node in nodes:
        # Calculate distance to node
        vec = np.array([node.x, node.y])
        dist = np.linalg.norm(cor - vec)
        # Save new nearest distance and corresponding node
        if dist < minDist:
            minDist = dist
            nearestNode = node

    rospy.logdebug(f"[PRM] Time to find nearest node with distance {minDist}: {time.perf_counter()-t0}")
    return nearestNode


def apply_PRM_init(map_ref, obstacles, start_node=None, goal_node=None, start=None, goal=None):
    """
    Applies the whole PRM process which includes all steps like sampling nodes, building a graph and\
    calculating the trajectory

    Args:
        map_ref: reference map
        obstacles (list(Obstacle)): Known obstacles on the map
        start_node (Node): If a specific start node is demanded it has to be passed here - otherwise it is random
        goal_node (Node): If a specific goal node is demanded it has to be passed here - otherwise it is random
    
    Returns: 
        traj (list(Node)): Calculated trajectory 
        traj_optimized (list(Node)): optimized trajectory (this is not used) -> see "optimize_trajectory" function
        nodes_copy (list (Node)): Nodes in the graph  -> this is a shallow copy and actually contains the real Node objects!
        edges_all (list(Edge)): All Edge in the graph
    """
    map_ref_copy = copy.deepcopy(map_ref)

    # add nodes to the cropped map
    load_nodes = False
    if load_nodes:
        # pickle load ~~~
        open_file = open(f"{PATH}/nodes_presentation", "rb")
        nodes = pickle.load(open_file)
        open_file.close()
        for i in range(0, len(nodes)):
            nodes[i] = Node(nodes[i].coordinates[0], nodes[i].coordinates[1])
    else:
        if None in [start, goal]:
            map_visu, nodes = add_nodes(map_ref_copy, N_NODES, obstacles)  # , starts, ends
        else:
            map_visu, nodes = add_nodes(map_ref_copy, N_NODES, obstacles, start, goal)  # , starts, ends
        map_visu.save(f"{PATH}/maps/map_nodes.png")

    t0 = time.perf_counter()
    # add neighbours / build up graph with edges and costs
    map_visu, nodes, edges_all = add_neighbours(map_ref_copy, nodes, N_NEIGHBOURS)
    # TODO: XXX this might cause trouble in for the adv!!! -> visualization will not exactly represent the truth (pixel perfect)
    try:
        calculate_edge_costs(map_ref_copy, edges_all, obstacles)
    except:
        map_visu, nodes = add_nodes(map_ref_copy, N_NODES, obstacles, start, goal)
        map_visu, nodes, edges_all = add_neighbours(map_ref_copy, nodes, N_NEIGHBOURS)
        calculate_edge_costs(map_ref_copy, edges_all, obstacles)
    nodes_copy = copy.copy(nodes)
    map_visu.save(f"{PATH}/maps/map_graph.png")
    rospy.logdebug(f"[PRM] Time add_neighbours: {time.perf_counter() - t0}")

    """" Uncomment this code to give your own path"""
    # start_node = get_node_with_coordinates(nodes, np.array(starts))
    # goal_node = get_node_with_coordinates(nodes, np.array(ends))

    if not (start_node and goal_node):
        start_node = nodes[0]
        goal_node = nodes[1]
        while (start_node.coordinates == goal_node.coordinates).all():
            goal_node = nodes[np.random.randint(0, len(nodes))]
    t2 = time.perf_counter()

    # calculate and draw trajectory with deijkstra's algorithm
    traj = deijkstra(nodes, start_node, goal_node)
    rospy.logdebug(f"[PRM] Time deijkstra: {time.perf_counter() - t2}")
    rospy.logdebug(f"[PRM] Trajectory: {get_traj_edges(traj)}")
    # visualize_traj(map_visu, traj)     # use for a visualization of the traj. with the whole graph also
    map_visu = copy.deepcopy(map_ref)
    draw_traj(map_visu, traj, False)

    map_visu.save(f"{PATH}/image/map_traj.png")

    map_visu, traj_opt = optimize_trajectory(map_ref_copy, traj)

    map_visu.save(f"{PATH}/image/map_opt_traj.png")

    return traj, traj_opt, nodes_copy, edges_all


def apply_PRM(
    map_ref,
    nodes,
    obstacles=None,
    visualize=False,
    start_node=None,
    goal_node=None,
    start=None,
    goal=None,
    edges_all=None,
    prot=False,
):
    """
    Applies the PRM process without sampling new nodes and building a graph (basically just does the dijkstra)

    Args:
        map_ref: reference map
        nodes: list of all Node-objects (the graph to do PRM on)
        visualize: determines wether the map with trajectory should be saved as png file (mostly for debugging)
        start_node (Node): If a specific start node is demanded it has to be passed here (Node-object) - otherwise it is random
        goal_node (Node): If a specific goal node is demanded it has to be passed here (Node-object) - otherwise it is random
        edges_all (list(Edge)): list of all Edge-objects
        prot: necessary parameter for the function "calculate_edge_costs which behaves differently if called for the protagonist

    Returns:
        traj (list(Node)): Calculated trajectory
        traj_optimized (list(Node)): Optimized trajectory (this is not used) -> see "optimize_trajectory" function
        nodes (list(Node)): Nodes in the graph
        edges_change_back (list(Edge)): Edges in the graph
    """
    t0_total = time.perf_counter()

    t0 = time.perf_counter()
    # map_ref_copy = copy.deepcopy(map_ref)     # only need that for opt_trajectory
    # nodes_copy = copy.deepcopy(nodes)
    deepcopy_time = time.perf_counter() - t0
    nodes_copy = copy.copy(nodes)  # shallow copy is enough here

    edges_change_back = None
    # if edges_all are passed, we recalculate their costs (for the protagonist)
    if edges_all:
        edges_change_back = calculate_edge_costs(map_ref, edges_all, prot=prot)

    # todo: a bit ugly for now --- [0] and [1] are the indices for the "start" and "goal" given by the apply_PRM(..) params
    # Generate random start and goal
    if start_node != None and goal_node != None:
        start_node = nodes_copy[np.random.randint(0, len(nodes_copy))]
        goal_node = nodes_copy[np.random.randint(0, len(nodes_copy))]
        while (start_node.coordinates == goal_node.coordinates).all():
            goal_node = nodes_copy[np.random.randint(0, len(nodes_copy))]
    # Use nearest node to start and goal so they are
    elif start != None and goal != None:
        start_node = findNearestNode(nodes_copy, start[0], start[1])
        goal_node = get_node_with_coordinates(nodes_copy, goal)
        if goal_node == None:
            goal_node = findNearestNode(nodes_copy, goal[0], goal[1])
    # calculate and draw trajectory with deijkstra's algorithm
    t1 = time.perf_counter()
    traj = deijkstra(nodes_copy, start_node, goal_node)
    deijkstra_time = time.perf_counter() - t1

    # visualize_traj(map_visu, traj)     # use for a visualization of the traj. with the whole graph also
    if visualize:
        map_visu = copy.deepcopy(map_ref)
        draw_traj(map_visu, traj, False)
        map_visu.save(f"{PATH}/image/map_traj.png")
    traj_opt = None

    return traj, traj_opt, nodes, edges_change_back
