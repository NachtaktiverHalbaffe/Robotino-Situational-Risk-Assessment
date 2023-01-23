import os
import pickle
import sys
import time

import cv2
import numpy as np
from PIL import Image, ImageDraw
from autonomous_operation.PRM import (
    apply_PRM,
    apply_PRM_init,
    draw_traj,
    calc_adv_traj,
    Node,
    get_traj_edges,
    get_node_with_coordinates,
    calc_nearest_dist,
)
from autonomous_operation.object_detection import apply_object_detection
import copy
from copy import deepcopy

from .config import configs
from real_robot_navigation.move_utils_cords import get_base_info
from real_robot_navigation.gridmap import get_obstacles_in_pixel_map

sys.setrecursionlimit(2000)

ACTION_SPACE_STEP_ADVERSARY = 5
N_ACTIONS_ADVERSARY = 5


class StateAdv1:
    """
    State of the sim-gap-adv

    Small notes to the states
    ---
    Basically both agents see the same state (which is "obs") but the other attributes differ (some stuff to\
    make things work implementation wise...).
    Also: In general I wanted to distinguish between "observation" and "state" but in the end didn't do so, 
    so the notation will be kinda random throughout this code
    
    ---
    Attributes:
        obs: obs is basically what the state is (the normalized reference map with trajectory) in general I wanted to \
            distinguish between observation and state but in the end didn't so the notation will be kinda random throughout this code
        traj_index: Just an attribut to keep track at which waypoint of the trajectory we are in our episode\
                -> You could say traj_index (or rather trajectory[traj_index]) gives us the step, while trajectory is the episode\
                but this should not be important anymore since it probably doesn't have to be changed anymore
        position: current position
        angle: current angle
    """

    def __init__(self, obs, pos, angle=0):
        self.obs = obs
        """Obs is basically what the state is (the normalized reference map with trajectory) in general I wanted to \
            distinguish between observation and state but in the end didn't so the notation will be kinda random throughout this code"""
        self.traj_index = 1
        """Just an attribut to keep track at which waypoint of the trajectory we are in our episode\
            -> You could say traj_index (or rather trajectory[traj_index]) gives us the step, while trajectory is the episode\
            but this should not be important anymore since it probably doesn't have to be changed anymore"""
        self.position = pos
        """Current position"""
        self.angle = angle
        """Current angle"""

    def __str__(self):
        print(
            "current #traj_node:",
            self.traj_index,
            "\n",
            "current position:",
            self.position,
        )


class StateProt:
    """State of the protagonist

    Small notes to the states
    ---
    Basically both agents see the same state (which is "obs") but the other attributes differ (some stuff to make things work implementation wise...).
    Also: in general I wanted to distinguish between "observation" and "state" but in the end didn't do so, so the notation will be kinda random throughout this code

    ---
    Attributes:
        obs: obs is basically what the state is (the normalized reference map with trajectory)
        position: current position
        vanilla_cost: total cost of the vanilla trajectory before applying the adversary
    """

    def __init__(self, obs, pos, vanilla_cost):
        self.obs = obs
        """Obs is basically what the state is (the normalized reference map with trajectory)"""
        self.position = pos
        """Current position"""
        self.vanilla_cost = vanilla_cost
        """Total cost of the vanilla trajectory before applying the adversary"""


class Command:
    """
    A command contains the necessary angle, speed and time to move from one waypoint to another.\
    This is only necessary because we need to model the 'real' process and the influence of the adversary
    """

    def __init__(self, angle, v, t):
        self.angle = angle
        """Angle which the Robotino should rotate befor driving forward """
        self.v = v
        """Velocity with which the Robotino should move forward"""
        self.t = t
        """Duration how long the Robotino should drive"""

    def __str__(self):
        return f"rotate: {str(self.angle)} speed: {str(self.v)} duration: {str(self.t)}"

    __repr__ = __str__  # kind of a bad practice


def calc_command(position, current_angle, target):
    """
    Calculate the necessary command to (theretically) move from "position" to "target" considering the current angle

    Args:
        position: current position coordinates (pixel location in image)
        current_angle: current angle
        target: desired position coordinates (pixel location in image)

    Returns:
        command: object with the necessary rotation angle, speed and time to reach the desired position
    """
    if (position == target).all():
        print("pos", position)  # 101, 103
        print("target", target)
    distance = np.linalg.norm(target - position)
    # the factor f = (distance/cost) can be used to scale the intended speed
    v = 1

    # comm_angle is the rotation angle -> positive angles represent counter clockwise rotation
    beta = np.arccos(np.abs(target[0] - position[0]) / distance)
    if current_angle > np.pi:
        current_angle = current_angle - 2 * np.pi
    if target[0] - position[0] >= 0 and target[1] - position[1] >= 0:
        # 4. Quadrant
        comm_angle = 2 * np.pi - (beta + current_angle)
    elif target[0] - position[0] >= 0 and target[1] - position[1] < 0:
        # 1. Quadrant
        comm_angle = beta - current_angle
    elif target[0] - position[0] < 0 and target[1] - position[1] >= 0:
        # 3. Quadrant
        comm_angle = np.pi + beta - current_angle
    else:
        # 2. Quadrant
        comm_angle = np.pi - (beta + current_angle)

    if comm_angle > np.pi:
        comm_angle = comm_angle - 2 * np.pi

    t = distance / v
    command = Command(comm_angle, v, t)

    return command


def calc_new_position(position, current_angle, command):
    """
    Function to calculate the new position (basically this function "applies" the command in the simulation)

    Args:
        position: current position
        current_angle: current angle
        command (Command): command which should be applied

    Returns:
        New position as coordinates in the image as well as the new angle
    """
    x_offset = command.v * command.t * np.cos(command.angle + current_angle)
    y_offset = command.v * command.t * np.sin(command.angle + current_angle)
    new_position_x = np.round(position[0] + x_offset)
    new_position_y = np.round(position[1] - y_offset)
    new_angle = command.angle + current_angle

    return (
        np.array([new_position_x.item(), new_position_y.item()], dtype=int),
        new_angle,
    )


def generate_obs_from_rgb_img(img_grey):
    """
    Generates an observation (state) for the agent by normalizing the passed image

    Args:
        img_grey: reference map with original trajectory (might be vanilla, might be the one after prot...)

    Returns:
        normalized version of passed image (observation/state of agent)
    """
    x_max = img_grey.size[0]
    y_max = img_grey.size[1]
    pixels = np.array(img_grey.getdata(), dtype=np.float32).reshape((y_max, x_max))

    v1 = 255
    v2 = 40
    v3 = 100
    v4 = 205
    v5 = 210
    # uncomment to see what the agent before normalisation
    # new_map = Image.fromarray(pixels.astype('uint8'), mode='L')
    # new_map.show()

    pixels[pixels == v1] = 0.6
    pixels[pixels == v2] = 0.2
    pixels[pixels == v3] = 0.4
    pixels[pixels == v4] = 1.0
    pixels[pixels == v5] = 0.8

    # uncomment to see what the agent sees (which should be basically a black image)
    # new_map = Image.fromarray(pixels.astype('uint8'), mode='L')
    # new_map.show()

    return pixels


def clear_image_files():
    """
    Removes the image files of the trajectory adversaries
    """
    if os.path.isfile("./image/adv_trajectory.png"):
        try:
            os.remove("./image/adv_trajectory.png")
        except PermissionError:
            print("permissionerror while removing ./image/adv_trajectory.png")


def initialize_map(map_path):
    """
    Initializes the reference map by applying the object detection

    Args:
        map_path: path for the scanner map

    Returns:
        reference map and the detected obstacles (with its borders)
    """
    map_ref, obstacles = apply_object_detection(map_path)

    # --- Only for test we use the already processed map ---
    # obstacles = None
    # ref_map_path = 'map_ref.png'
    # map_ref = Image.open(map_path)

    return map_ref, obstacles


def initialize_traj(
    map_ref,
    obstacles=None,
    nodes=None,
    visualize=False,
    edges_all=None,
    env=None,
    start=[62, 74],
    goal=[109, 125],
):
    """Initialize a new usually random trajectory for training
    
    If we pass nodes here then those nodes are used to calculate a trajectory. If no nodes are passed then new nodes are generated (takes time)

    Args:
        map_ref: map that we want to use PRM on - usually the reference map
        obstacles (list(Obstacle)): known obstacles (list of Obstacles-objects)
        nodes: current nodes (for the graph for PRM)
        visualize: determines if the new trajectory should be saved as an image (usually for debugging..)
        edges_all: edges_all are the current edges of our graph with their respective costs and nodes. They will\
                           be calculated new if we calculate new nodes
        env: Environment object. Only needed when dynamically generating random maps

    Returns: 
        trajectory: new trajectory as list of nodes
        nodes: new or old nodes, depending on param nodes
        edges_all: edges with their new costs (or unchanged costs, depending on whether we passed nodes)
    """
    # print(start, goal)
    if not nodes:
        if env:
            # env.map_ref, env.obstacles = create_random_map()
            # print("in not nodes and if env")
            # print(start,goal)
            traj, traj_opt, nodes, edges_all = apply_PRM_init(
                env.map_ref, env.obstacles
            )  # , start=start, goal=goal
        else:
            # print("in not nodes and else env")
            traj, traj_opt, nodes, edges_all = apply_PRM_init(
                map_ref, obstacles
            )  # , start=start, goal=goal

        # pickle dump ~~~
        # print('dumping nodes...')
        # open_file = open('nodes_presentation', "wb")
        # pickle.dump(nodes, open_file)
        # open_file.close()
    else:
        # for specific start / goal location: ------------------
        # print("in else nodes")
        start_node = get_node_with_coordinates(nodes, (62, 74))
        goal_node = get_node_with_coordinates(nodes, (109, 125))
        traj, _, nodes, _ = apply_PRM(map_ref, nodes, visualize=visualize)

        """ Uncomment the below code for own path"""
        # traj, _, nodes, _ = apply_PRM(map_ref, nodes, visualize=visualize, start_node=start_node, goal_node=goal_node)
        # ------------------------------------------------------

        # traj, _, nodes, _ = apply_PRM(map_ref, nodes, visualize=visualize)
    # print('fresh trajectory:', traj)

    return traj, nodes, edges_all


def initialize_state_adv1(map_ref, traj, relevant_segments, visualize=False):
    """
    Initialize the first state/observation for the sim-gap-adv

    Args:
        map_ref: reference map
        traj: current original trajectory for the new state
        relevant_segments: how many segments should be considered in the state for the sim-gap-adv
        visualize: wether the image files for observations should be generated

    Returns:
        new state for simulation-gap-adversary
    """
    if relevant_segments != 0:
        relevant_max = min(relevant_segments + 1, len(traj))
        traj = traj[:relevant_max]

    map_visu = copy.deepcopy(map_ref)
    draw_traj(map_visu, traj, forAgent=True)
    # if visualize:
    #     map_visu.save('./image/new_obs0.png')

    init_state = StateAdv1(generate_obs_from_rgb_img(map_visu), traj[0].coordinates)

    return init_state


def initialize_state_prot(environment, visualize=False):
    """
    Initialize the first state/observation for the protagonist.

    Args:
        environment: Environment object
        visualize: wether we want to visualize the actions of prot in image files

    Returns:
        new state/obs of protagonist
    """
    t0 = time.perf_counter()
    map_ref = environment.map_ref
    traj = copy.copy(environment.trajectory_vanilla)
    map_visu = copy.deepcopy(map_ref)
    # adversary = environment.adversary

    draw_traj(map_visu, traj, forAgent=True)

    # if adversary:
    #     init_state = StateAdv1(generate_obs_from_rgb_img(map_visu), traj[0].coordinates)
    #     observation = init_state.obs
    #
    #     traj = [traj[0]]
    #     done = False
    #
    #     while not done:
    #         action_index, _, _, _ = adversary.choose_action(observation, test_mode=True)
    #         observation, _, done, _, node_adv = environment.step_adv1(np.deg2rad(5 * action_index - 10))      # 5 and 10 should be somehow config parameter here (depending on action space of adv1)
    #         traj.append(node_adv)
    #
    #     map_visu = copy.deepcopy(map_ref)   # deepcopy doesn't take much time here
    #     visualize_traj(map_visu, traj, forAgent=True)

    # calculate the nearest distance / point to all objects
    nearest_distances, most_critical_points = calc_nearest_dist(
        traj, environment.obstacles
    )
    for i in range(
        0, len(nearest_distances)
    ):  # gives us the nearest distance / point for each segment in a list
        nearest_distances[i] = np.round(nearest_distances[i][0])
        most_critical_points[i] = most_critical_points[i][0]
        if nearest_distances[i] <= 0:
            nearest_distances[
                i
            ] = 1  # nearest dist can not be less than 1. the resolution when calculating the distance is not pixel perfect anyway
    nearest_distances = np.array(nearest_distances)
    vanilla_cost = 1 / np.min(nearest_distances)

    # print('actually before prot and before adversary:', environment.trajectory_vanilla)
    # environment.trajectory_vanilla = traj
    init_state = StateProt(
        generate_obs_from_rgb_img(map_visu), traj[0].coordinates, vanilla_cost
    )

    # print('nearest distances:')
    # print(nearest_distances)
    # print('most critical points:')
    # print(most_critical_points)
    # print('vanilla_distance_metric:', vanilla_cost)

    t_init_state = time.perf_counter() - t0
    # print('time init_state (protagonist):', t_init_state)

    if visualize:
        map_visu.save("./image/prot_obs.png")

    return init_state


def copy_nodes(edges):
    """
    Copies the passed edges and extracts their nodes and also copies and returns those the copies are deepcopies

    Args:
        edges: list of current Edge-objects

    Returns:
        copy of all nodes in edges and copy of edges themselves
    """
    t0 = time.perf_counter()
    edges_copy = copy.deepcopy(edges)
    nodes_copy = []
    for edge in edges_copy:
        if not (edge.node1 in nodes_copy):
            nodes_copy.append(edge.node1)
        if not (edge.node2 in nodes_copy):
            nodes_copy.append(edge.node2)
    print("time_copy_nodes:", time.perf_counter() - t0)
    return nodes_copy, edges_copy


class Environment:
    """
    The environment is the same for both agents, but they use a different step-function

    !! Some important note !!
    ---
    We have to be very careful whenever we change any nodes or edges (or attributes of these) because they\
    are referring to each other and changing one also changes the other (each edge has nodes and each nodes also has edges).

    AND: nodes_prot and nodes_vanilla as well as edges_all_vanilla and edges_all_prot are not supposed to be "mixed" 
    -> Node-objects in nodes_prot will be referenced in edges_all_prot but are not supposed to be referenced in edges_all_vanilla\
        same goes for nodes_vanilla which is not allowed contain/reference node objects appearing in edges_all_prot
    ---

    Attributes:
        adversary: trained adversary (consists of 2 models -> actor and critic) - necessary for step_prot()
        map_ref (PIL Image): reference map
        obstacles (list(Obstacle)): list of Obstacles-objects with their detected borders
        observation_space: size of the reference map represents the observation space. -> this attribute is not used...
        trajectory_vanilla (list(Node)): this is the "original" trajectory just after applying PRM without any agents interfering
        nodes_vanilla (list(Node)): the "original" nodes (or basically the graph), used to generate trajectory_vanilla via PRM
        edges_all_vanilla: all "original" edges connecting the nodes_vanilla. -> list of Edge-objects
                            all_edges_vanilla hahve their costs calculated based on nodes_vanilla and map_ref
        nodes_prot (list(Node)): the nodes (or graph..) after changing the map with protagonist -> used to calculate the new trajectory
        edges_all_prot  (list(Node)): all edges with their respective costs after changing the map with protagonist
        relevant_segments: config parameter that determines how many segments of a trajectory are "seen"\
                            "0" means the agents will see the whole trajectory (which there is not so much reason to change)
        state_adv1: initial state of the sim-gap-adv
        edges_vanilla(list(Edge)): edges of the vanilla trajectory
        done_after_collision: config parameter that determines if the episode (for sim-gap-adv) should terminate after\
                                collision was detected or not -> there is not really a reason to not choose it as "True"
        visualize (bool): if this is "True" some png files will be created in the process of training/evaluating the agents.\
                    Mainly for debugging (the result is not influenced in any way by this, except it might be a bit slower)
        state_prot: initial state of the protagonist
    """

    def __init__(
        self,
        map_path,
        relevant_segments=0,
        done_after_collision=True,
        visualize=False,
        adversary=None,
        start=[62, 74],
        goal=[109, 125],
    ):
        clear_image_files()
        self.debug_prints = False
        # For now only have angle-offset as action
        # self.action_space = gym.spaces.Discrete(5)          # {-10, -5, 0, 5, 10}
        # self.observation_space = gym.spaces.Box(low=np.full((160, 160), 0), high=np.full((160, 160), 1), shape=(160, 160), dtype=int) # 0: empty, 1: object, 2: trajectory segment (planned), 3: current position
        self.adversary = adversary
        self.map_ref, self.obstacles = initialize_map(map_path)
        "here we are obtaining all the obstacles as obstacles so that we can work with them for crash and remove"
        corners_table = [(105, 110), (103, 96), (90, 90), (80, 98)]
        obst_table = Obstacle(corners_table)
        self.obstacles = [obst_table]
        base_info, map_config = get_base_info()
        obstacles_ws, names_ws = get_obstacles_in_pixel_map(base_info)
        obstacles_movable, names_movables = get_obstacles_in_pixel_map(
            base_info, "movable"
        )
        self.obstacles.extend(obstacles_ws)
        self.obstacles.extend(obstacles_movable)
        self.map_ref = self.modify_map_2(
            self.map_ref,
            [],
            self.obstacles,
            color=(255, 255, 255),
            convert_back_to_grey=True,
        )
        "finished changing map with obstacles"
        self.persistent = False
        self.observation_space = self.map_ref.size
        self.map_ref_adv = copy.deepcopy(self.map_ref)
        self.need_new = True
        (
            self.trajectory_vanilla,
            self.nodes_vanilla,
            self.edges_all_vanilla,
        ) = initialize_traj(
            self.map_ref, self.obstacles, nodes=None, start=start, goal=goal
        )
        self.nodes_prot, self.edges_all_prot = copy_nodes(self.edges_all_vanilla)
        self.relevant_segments = relevant_segments
        """config parameter that determines how many segments of a trajectory are "seen"\
         "0" means the agents will see the whole trajectory (which there is not so much reason to change)"""
        self.state_adv1 = initialize_state_adv1(
            self.map_ref,
            self.trajectory_vanilla,
            relevant_segments=relevant_segments,
            visualize=True,
        )
        """Initial state of the sim-gap-adv"""
        self.edges_vanilla = get_traj_edges(self.trajectory_vanilla)
        """Edges of the vanilla trajectory"""
        self.done_after_collision = done_after_collision
        """Config parameter that determines if the episode (for sim-gap-adv) should terminate after\
         collision was detected or not -> there is not really a reason to not choose it as "True"""
        self.visualize = visualize
        """If this is "True" some png files will be created in the process of training/evaluating the agents.\
            Mainly for debugging (the result is not influenced in any way by this, except it might be a bit slower)"""
        self.state_prot = initialize_state_prot(self, visualize=True)
        """Initial state of the protagonist"""

        """ Swapnil code"""
        self.traj_adversary = []
        """Trajectory after the adversary has applied his actions"""

    def modify_map_2(
        self,
        map_ref,
        obstacles_org,
        obstacles,
        color=(255, 255, 255),
        convert_back_to_grey=True,
    ):
        """This is takes a set of obstacles to remove from the image and a set to place in the image, often used for shifting obstacles in
        the map by passing the same obstacles at the old and new different locations
        @param takes: the image we want to modify
        @param obstacles_org: set of obstacles to remove from the image
        @param obstacles: set of obstacles to place in the image
        @param color: the color of the object being placed
        @param convert_back_to_grey: if the image should be converted back into grey, needed if used in the PRM
        @return: map_ref_adv: the modified image
        """
        map_ref_adv = deepcopy(map_ref)
        map_ref_adv = map_ref_adv.convert("RGB")
        map_ref_adv_draw = ImageDraw.Draw(map_ref_adv)
        add = [(2, 2), (2, -2), (-2, -2), (-2, 2)]
        add = [(0, 0), (0, -0), (-0, -0), (-0, 0)]
        for obstacle in obstacles_org:
            # cv2.fillConvexPoly(self.map_ref_adv,obstacle.corners, color='black')
            # increase the size of the obstacle by one pixel
            # corners = [tuple(map(lambda i,j:i+j,obstacle.corners[i],add[i])) for i in range(4)]
            map_ref_adv_draw.polygon(
                obstacle.corners, fill=(0, 0, 0), outline=(0, 0, 0)
            )
        # add = [(1,1),(1,-1),(-1,-1),(-1,1)]
        for obstacle in obstacles:
            # cv2.fillConvexPoly(self.map_ref_adv,obstacle.corners, color='white')
            # corners = [tuple(map(lambda i,j:i+j,obstacle.corners[i],add[i])) for i in range(4)]
            map_ref_adv_draw.polygon(obstacle.corners, fill=color, outline=color)
        if convert_back_to_grey:
            map_ref_adv = map_ref_adv.convert("L")
        return map_ref_adv

    def reset_traj(self, node_num=1, pos=[-1, -1]):
        self.state_adv1.traj_index = node_num
        if len(pos) > 0:
            if pos[0] == -1:
                self.state_adv1.position = self.trajectory_vanilla[
                    node_num - 1
                ].coordinates
                # print("in -1 pos")
            else:
                self.state_adv1.position = pos
        self.state_adv1.angle = 0

    # todo: for now action is just simply the angle-offset but later this should actually be a combination of angle_offset and v_offset
    def step_adv1(self, action, action_prob, actions_sb3={}, probs_action_sb3={}):
        """
        Applies the action of the sim-gap-adv in the environment

        Args:
            action: angle offset (action of adversary)

        Returns:
            obs: the next state/observation
            reward (float): reward of this step
            done (bool): Wether this was the last step -> if done==1 the episode terminates here
            info (int): tells if we had a collision (1) or not (0)
            adv1_node2 : only used for applying the adversary after protagonist in step_prot(..)\
            -> this is the new "measured" position, resulting of the adversaries action
        """
        ############################################################
        # ------------------- Manipulations by adversary ------------------------
        ############################################################
        done = False

        command_vanilla = calc_command(
            self.state_adv1.position,
            self.state_adv1.angle,
            self.trajectory_vanilla[self.state_adv1.traj_index].coordinates,
        )
        """ Stephan"""
        angle_comm_new = command_vanilla.angle + action
        """ Swapnil"""
        # angle_comm_new = command_vanilla.angle
        v_comm_new = command_vanilla.v
        if "angle_loc" in actions_sb3:
            angle_comm_new = angle_comm_new + actions_sb3["angle_loc"]
        if "dist_loc" in actions_sb3:
            v_comm_new = v_comm_new + actions_sb3["dist_loc"] / command_vanilla.t
        command_disturbed = Command(angle_comm_new, v_comm_new, command_vanilla.t)

        # t0 = time.perf_counter()
        pos_new, angle_new = calc_new_position(
            self.state_adv1.position, self.state_adv1.angle, command_disturbed
        )

        if not "dist_loc" in actions_sb3:
            pos_new[0] = pos_new[0] + action

        # This block deals with the situation when the adversary coincidentally steers the robot on the position
        # of the next node in the trajectory (which would end up in a segment with distance 0)
        if not (self.state_adv1.traj_index + 1 >= len(self.trajectory_vanilla)):
            if (
                pos_new
                == self.trajectory_vanilla[self.state_adv1.traj_index + 1].coordinates
            ).all():
                self.state_adv1.traj_index += 1
                if self.state_adv1.traj_index >= len(self.trajectory_vanilla):
                    # traj finished (skipping last node because we are already there
                    done = True
        # calc_new_pos_time = time.perf_counter()-t0

        # Update trajectory => remove current node for which the action was taken
        # TODO Simplification
        traj_vanilla_coordinates = []
        relevant_max = len(self.trajectory_vanilla)
        if self.relevant_segments != 0:
            relevant_max = min(
                (
                    (self.state_adv1.traj_index + 1) + self.relevant_segments - 1
                ),  # TODO check if +1 and -1 can be safely removed
                len(self.trajectory_vanilla),
            )
            for node in self.trajectory_vanilla[
                self.state_adv1.traj_index + 1 : relevant_max
            ]:
                traj_vanilla_coordinates.append(node.coordinates)

        segment_adv_coordinates = [self.state_adv1.position, pos_new]

        """ Swapnil: Change the position by adding pixels, once it has reached the new position"""
        if (
            self.state_adv1.position[0] == pos_new[0]
            and self.state_adv1.position[1] == pos_new[1]
        ) and not "dist_loc" in actions_sb3:
            pos_new[1] = pos_new[1] + action + 1
            pos_new[0] = pos_new[0] + 1

        """ Swapnil code"""
        # self.traj_adversary.append(pos_new)

        """ Stephan code"""
        segment_adv_coordinates.extend(traj_vanilla_coordinates)

        # t3 = time.perf_counter()
        segment_adv_nodes, segments_adv, obstables_to_remove = calc_adv_traj(
            self.map_ref_adv, segment_adv_coordinates, self.obstacles
        )
        # calc_adv_traj_time = time.perf_counter()-t3
        cost_adv_segments = 0
        for segment in segments_adv:
            cost_adv_segments += segment.cost

        adv1_node1 = segments_adv[0].node1
        adv1_node2 = segments_adv[0].node2

        #############################################################
        # ---------------------------- Visualization ---------------------------------
        #############################################################
        if self.visualize:
            if os.path.isfile("./image/adv_trajectory.png"):
                visu_adv_traj_map = Image.open("./image/adv_trajectory.png")
            else:
                visu_adv_traj_map = copy.deepcopy(Image.open("./image/map_traj.png"))
            risk_predicting = True
            if risk_predicting and self.need_new:
                self.visu_adv_traj_map = copy.deepcopy(self.map_ref_adv)
                self.need_new = False
                self.visu_adv_traj_map = self.visu_adv_traj_map.convert("RGB")
            # visu_adv_traj_map = visu_adv_traj_map.convert('RGB')
            visu_adv_traj_map_draw = ImageDraw.Draw(self.visu_adv_traj_map)
            visu_adv_traj_map_draw.line(
                [
                    (adv1_node1.coordinates[0], adv1_node1.coordinates[1]),
                    (adv1_node2.coordinates[0], adv1_node2.coordinates[1]),
                ],
                fill=(255, 0, 0),
            )
            visu_adv_traj_map_draw.point(
                [(adv1_node1.coordinates[0], adv1_node1.coordinates[1])],
                fill=(100, 0, 0),
            )
            visu_adv_traj_map_draw.point(
                [(adv1_node2.coordinates[0], adv1_node2.coordinates[1])],
                fill=(100, 0, 0),
            )
            try:
                # self.visu_adv_traj_map.save('./image/adv_trajectory.png')
                self.visu_adv_traj_map.save("./image/adv_trajectory_DEBUG.png")
            except PermissionError:
                print("permissionError when saving file")

        #########################################################
        # ---------------------- Cost calculation ------------------------------
        #########################################################
        # Calculation of difference in costs (according to reference map)
        cost_segments_vanilla = 0
        for i in range(
            0, len(self.edges_vanilla[self.state_adv1.traj_index - 1 : relevant_max])
        ):
            cost_segments_vanilla += self.edges_vanilla[
                self.state_adv1.traj_index - 1 + i
            ].cost
        cost_difference = cost_adv_segments - cost_segments_vanilla

        ###########################################################################
        ## TODO DRY: Move this block into a function
        # calculation of difference in distance to closest object adversary
        nearest_distances_adv, most_critical_points_adv = calc_nearest_dist(
            [adv1_node1, adv1_node2], self.obstacles
        )
        for i in range(0, len(nearest_distances_adv)):
            # gives us the nearest distance / point for each segment in a list
            nearest_distances_adv[i] = np.round(nearest_distances_adv[i][0])
            most_critical_points_adv[i] = most_critical_points_adv[i][0]
            if nearest_distances_adv[i] <= 0:
                # nearest dist can not be less than 1. The resolution when calculating the distance is not pixel perfect anyway
                nearest_distances_adv[i] = 1
        nearest_distances_adv = np.array(nearest_distances_adv)
        adv1_dist_reward = 1 / np.min(nearest_distances_adv)

        # TODO Here the repetition starts....
        # vanilla...
        nearest_distances_vanilla, most_critical_points_vanilla = calc_nearest_dist(
            [
                self.trajectory_vanilla[self.state_adv1.traj_index - 1],
                self.trajectory_vanilla[self.state_adv1.traj_index],
            ],
            self.obstacles,
        )

        for i in range(0, len(nearest_distances_vanilla)):
            # gives us the nearest distance / point for each segment in a list
            nearest_distances_vanilla[i] = np.round(nearest_distances_vanilla[i][0])
            most_critical_points_vanilla[i] = most_critical_points_vanilla[i][0]
            if nearest_distances_vanilla[i] <= 0:
                # nearest dist can not be less than 1. the resolution when calculating the distance is not pixel perfect anyway
                nearest_distances_vanilla[i] = 1
        nearest_distances_vanilla = np.array(nearest_distances_vanilla)
        vanilla_dist_reward = 1 / np.min(nearest_distances_vanilla)
        ##
        #####################################################################################################

        distance_reward = 20 * (adv1_dist_reward - vanilla_dist_reward)

        ###############################################################
        # ------------------ Determine if collision occurred -------------------------
        ###############################################################
        collision = 0
        # finally the reward depending on (costs), distance and collision:
        reward_for_crash = 1000
        if cost_difference >= 9999:  # colission occured
            reward = reward_for_crash + distance_reward  # 1
            collision = 1
            if self.done_after_collision:
                done = True
                if self.debug_prints:
                    print("\U0001F6AB - collision")
        else:
            reward = distance_reward
        if "combined" in probs_action_sb3:
            reward = (
                reward + np.log(probs_action_sb3["combined"]) * 20
            )  # *config.configs[i]['prob_const']
            # print('reward is ',reward)
        else:
            """swapnil"""
            reward = reward * action_prob * config.configs[i]["prob_const"]
        """ Stephan """
        # reward = distance_reward
        old_position = self.state_adv1.position
        # print("Old Position: ", old_position)

        ##############################################################
        # ------------------------ Updating trajectory ------------------------------
        ##############################################################
        self.state_adv1.position = pos_new
        self.state_adv1.angle = angle_new
        """ Increment a trajectory index"""
        self.state_adv1.traj_index += 1

        relevant_max = min(relevant_max + 1, len(self.trajectory_vanilla))

        if self.state_adv1.traj_index >= len(self.trajectory_vanilla):
            done = True
        if not done:
            segments_vanilla = [Node(pos_new[0], pos_new[1])]
            segments_vanilla.extend(
                self.trajectory_vanilla[self.state_adv1.traj_index : relevant_max]
            )
            map_visu_new = copy.deepcopy(self.map_ref)
            # t1 = time.perf_counter()
            draw_traj(map_visu_new, segments_vanilla, forAgent=True)
            # visualize_traj_time = time.perf_counter()-t1
            # if self.visualize:
            #    map_visu_new.save('./image/new_obs' + str(self.state_adv1.traj_index - 1) + '.png')    # debug
            # t4 = time.perf_counter()
            self.state_adv1.obs = generate_obs_from_rgb_img(map_visu_new)
            # generate_obs_time = time.perf_counter()-t4

        info = collision

        if not collision and done and self.debug_prints:
            print("no collision")
        # scaling reward
        reward = reward / reward_for_crash
        return (
            self.state_adv1.obs,
            reward,
            done,
            info,
            adv1_node2,
            old_position,
            obstables_to_remove,
        )

    def step_prot(self, action):
        """
        Applies the action of the protagonist in the environment. Also applies the action of the adversary here so the protagonist\
        gets manipulated by the adversaries

        Args:
            action (int): kernel size of dilation filter

        Returns:
            reward (float): reward of this episode/step
            info (set(int, float)): info is a tuple of (collision, length_ratio) while collision tells if a collision occured and\
                                            length_ratio is the cost factor of how much longer the traj after prot is than vanilla
        """
        ##############################
        # ------------- Setup ---------------
        ##############################
        # Taking a matrix of size 'action' as the kernel
        if action == -1:
            kernel = np.ones((1, 1), dtype=np.uint8)
        else:
            kernel = np.ones((action, action), dtype=np.uint8)

        # Creating a map with which the protagonist can work
        map_ref_prot = np.array(self.map_ref.getdata(), dtype=np.uint8).reshape(
            self.map_ref.size[1], self.map_ref.size[0]
        )
        map_ref_prot = cv2.dilate(map_ref_prot, kernel, iterations=1)
        # map_ref_prot = cv2.medianBlur(map_ref_prot, action)
        # map_ref_prot = map_ref_prot.reshape(self.map_ref.size[1], self.map_ref.size[0])
        map_ref_prot = Image.fromarray(map_ref_prot.astype("uint8"), mode="L")

        # map_ref_prot.save('./image/map_prot.png')

        # print('trajectory before prot (###):', self.trajectory_vanilla)
        # print('edges_vanilla before prot:', get_traj_edges(self.trajectory_vanilla))
        start_node = get_node_with_coordinates(
            self.nodes_prot, self.trajectory_vanilla[0].coordinates
        )
        goal_node = get_node_with_coordinates(
            self.nodes_prot, self.trajectory_vanilla[-1].coordinates
        )

        ###########################################
        # ----------------- Apply action ---------------------
        ###########################################
        traj, _, _, edges_change_back = apply_PRM(
            map_ref_prot,
            self.nodes_prot,
            start_node=start_node,
            goal_node=goal_node,
            edges_all=self.edges_all_prot,
            prot=True,
        )

        # Calculate length of trajectory
        length_prot = 0
        for edge in get_traj_edges(traj):
            length_prot += edge.length

        traj_vanilla_copy = copy.copy(self.trajectory_vanilla)
        # print('vanilla vorher:')
        # print(self.trajectory_vanilla)
        self.trajectory_vanilla = traj
        # print('copy vorher:')
        # print(traj_vanilla_copy)
        # print('traj vorher (nach prot)')
        # print(traj)

        # print('len(self.nodes_prot)', len(self.nodes_prot))
        # print('edges after prot:', get_traj_edges(traj))
        # print('edges_vanilla after prot:', get_traj_edges(self.trajectory_vanilla))
        # print('trajectory after prot (###):', traj)
        # print('trajectory vanilla after prot (###):', self.trajectory_vanilla)

        #############################################################
        # ------------------  Apply action of adversary ----------------------------
        #############################################################
        collided = 0
        if self.adversary:
            # part of the step_prot(..) is the adversary applying his action (here)
            # print('applying adversary on prot_traj...')
            map_visu = copy.deepcopy(self.map_ref)

            draw_traj(map_visu, traj, forAgent=True)

            self.state_adv1 = StateAdv1(
                generate_obs_from_rgb_img(map_visu), traj[0].coordinates
            )
            observation = self.state_adv1.obs

            traj = [traj[0]]
            done = False

            while not done:
                action_index, _, _, _ = self.adversary.choose_action(
                    observation, test_mode=True
                )
                action_angle_offset = np.deg2rad(
                    ACTION_SPACE_STEP_ADVERSARY * action_index
                    - (int(N_ACTIONS_ADVERSARY / 2) * ACTION_SPACE_STEP_ADVERSARY)
                )
                observation, _, done, collision, node_adv = self.step_adv1(
                    action_angle_offset
                )
                if collision == 1:
                    collided = 1
                traj.append(node_adv)

        self.trajectory_vanilla = traj_vanilla_copy
        # print('copy')
        # print(traj_vanilla_copy)
        # print('vanilla nachher')
        # print(self.trajectory_vanilla)
        # print('trajectory')
        # print(traj)

        # Calculate length of original trajectory
        length_vanilla = 0
        for edge in get_traj_edges(self.trajectory_vanilla):
            length_vanilla += edge.length
        # print('length_vanilla', length_vanilla)

        # This (0.5) is an important hyperparameter that regulates the balance between
        # performance and safety (in combination with the other rewards and cost values...) -> might has to be adjusted!!!
        eps = 1e-12
        traj_length_penalty = 0.5 * (1 - length_vanilla / (length_prot + eps))
        # print('traj_length_penalty', traj_length_penalty)

        # print('trajectory after prot (and possibly adversary):', traj)

        # TODO Another DRY?
        # calculate the nearest distance / point to all objects
        nearest_distances, most_critical_points = calc_nearest_dist(
            traj, self.obstacles
        )
        for i in range(0, len(nearest_distances)):
            # gives us the nearest distance / point for each segment in a list
            nearest_distances[i] = np.round(nearest_distances[i][0])
            most_critical_points[i] = most_critical_points[i][0]
            if nearest_distances[i] == 0:
                # nearest dist can not be less than 1. the resolution when calculating the distance is not pixel perfect anyway
                nearest_distances[i] = 1
        nearest_distances = np.array(nearest_distances)

        prot_cost = 0.5 * (1 / np.min(nearest_distances))
        # print('dist cost after prot and after adv', prot_cost)

        if bool(collided):
            prot_cost = 1
        # print('prot_cost (distance) ', prot_cost)
        # distance_difference = self.state_prot.vanilla_cost - prot_cost
        distance_cost = prot_cost

        #########################################
        # ---------------- Visualization -------------------
        #########################################
        debug_visu = True
        if debug_visu:
            map_ref_prot = draw_traj(
                map_ref_prot, self.trajectory_vanilla, forAgent=False, color=(255, 0, 0)
            )
            map_ref_prot = draw_traj(
                map_ref_prot, traj, forAgent=False, color=(0, 0, 255)
            )
            map_ref_prot.save("./image/map_prot.png")
            # print('cost_difference:', cost_difference)
            # time.sleep(10)

        #########################################
        # --------- Calculate cost and reward -----------
        #########################################
        if edges_change_back:
            for edge, cost in edges_change_back.items():
                edge.cost = cost

        reward = -distance_cost - traj_length_penalty

        info = (collided, traj_length_penalty)

        # since we have an episodic environment for prot, there is no state/obs to return
        # TODO shrink return statements
        return (
            None,
            reward,
            True,
            info,
            traj,
        )

    def reset(
        self,
        relevant_agent,
        reset_traj=True,
        new_nodes=False,
        start=[62, 74],
        goal=[109, 125],
        forced_traj=None,
        persisten_map=False,
        new_traj=False,
    ):
        """
        Resets the environment - that means samples a new trajectory and if "new_nodes" is true then also samples new\
        nodes (and therefor graph) for PRM

        Args:
            relevant_agent (Agent): depending on which agent is trained the reset function has to behave a bit different
            reset_traj (bool): Telling the reset function if the trajectory should be reset. Defaults to True
            new_nodes (bool): Telling wether we want to sample a new graph (True) or keep the current one (False). Defaults to False
            start: Defaults to [62, 74]
            goal: Defaults to [109, 125]
        
        Returns: 
            obs_ret: which is the new state for the "relevant_agent"
        """
        clear_image_files()

        if persisten_map:
            self.map_ref_adv = copy.deepcopy(self.map_ref)
            self.persistent = True
        else:
            self.persistent = False
        self.need_new = True
        if new_traj:
            if not new_nodes:
                self.trajectory_vanilla, self.nodes_vanilla, _ = initialize_traj(
                    self.map_ref,
                    nodes=self.nodes_vanilla,
                    visualize=self.visualize,
                    start=start,
                    goal=goal,
                )
            else:
                (
                    self.trajectory_vanilla,
                    self.nodes_vanilla,
                    self.edges_all_vanilla,
                ) = initialize_traj(
                    self.map_ref,
                    obstacles=self.obstacles,
                    visualize=self.visualize,
                    env=self,
                    start=start,
                    goal=goal,
                )
                self.nodes_prot, self.edges_all_prot = copy_nodes(
                    self.edges_all_vanilla
                )
        if forced_traj is not None:
            self.trajectory_vanilla = forced_traj  # self.trajectory_vanilla[crash_point:min(crash_point+6,len(self.trajectory_vanilla))]
        self.edges_vanilla = get_traj_edges(self.trajectory_vanilla)
        obs_ret = None

        ############
        # Reset agent
        ############
        if relevant_agent == "adv1":
            self.state_adv1 = initialize_state_adv1(
                self.map_ref,
                self.trajectory_vanilla,
                relevant_segments=self.relevant_segments,
                visualize=self.visualize,
            )
            obs_ret = self.state_adv1.obs
        elif relevant_agent == "prot":
            self.state_adv1.traj_index = 1
            self.state_prot = initialize_state_prot(self)
            obs_ret = self.state_prot.obs
        if self.debug_prints:
            print("Fresh:", self.trajectory_vanilla)

        return obs_ret, self.trajectory_vanilla
