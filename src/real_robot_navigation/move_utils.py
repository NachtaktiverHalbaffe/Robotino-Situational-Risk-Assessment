import numpy as np

# from PIL import Image, ImageDraw
from PIL import ImageDraw
import sys
import os
from cv_bridge import CvBridge
import cv2
import rospy
from geometry_msgs.msg import Twist
from copy import deepcopy
from PIL import ImageOps
import matplotlib.pylab as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from autonomous_operation.PRM import (
    apply_PRM,
    apply_PRM_init,
)
from autonomous_operation.object_detection import apply_object_detection
from autonomous_operation.object_detection import Obstacle

# sys.path.insert(0, os.path.dirname(os.path.dirname(__file__))+'/yolov7')
# from detect_online import get_conf_and_model, loaded_detect


velocity_publisher_robot = rospy.Publisher("cmd_vel_real", Twist, queue_size=10)


class example:
    staticVariable = True  # Access through class


def initialize_traj(
    map_ref,
    obstacles=None,
    nodes=None,
    start_node=None,
    goal_node=None,
    visualize=False,
    edges_all=None,
    env=None,
    start=[150, 117],
    end=[76, 98],
):
    """
    initialize a new usually random trajectory for training
    if we pass nodes here then those nodes are used to calculate a trajectory.
    if no nodes are passed then new nodes are generated (takes time)
    @param map_ref: map that we want to use PRM on - usually the reference map
    @param obstacles: known obstacles (list of Obstacles-objects)
    @param nodes: current nodes (for the graph for PRM)
    @param visualize: determines if the new trajectory should be saved as an image (usually for debugging..)
    @param edges_all: edges_all are the current edges of our graph with their respective costs and nodes. They will
                        be calculated new if we calculate new nodes
    @param env: Environment object. Only needed when dynamically generating random maps
    @return: trajectory - new trajectory as list of nodes
             nodes - new or old nodes, depending on param nodes
             edges_all - edges with their new costs (or unchanged costs, depending on whether we passed nodes)
    """
    # import pdb; pdb.set_trace()
    if not nodes:
        if env:
            # env.map_ref, env.obstacles = create_random_map()
            traj, traj_opt, nodes, edges_all = apply_PRM_init(
                env.map_ref, env.obstacles
            )
        else:
            traj, traj_opt, nodes, edges_all = apply_PRM_init(
                map_ref, obstacles, start=start, goal=end
            )

        # pickle dump ~~~
        # print('dumping nodes...')
        # open_file = open('nodes_presentation', "wb")
        # pickle.dump(nodes, open_file)
        # open_file.close()
    elif start_node and goal_node:
        traj, _, nodes, _ = apply_PRM(
            map_ref, nodes, start_node=start_node, goal_node=goal_node
        )

    else:
        # for specific start / goal location: ------------------
        # start_node = get_node_with_coordinates(nodes, (62, 74))
        # goal_node = get_node_with_coordinates(nodes, (109, 125))
        # traj, _, nodes, _ = apply_PRM(map_ref, nodes, visualize=visualize, start_node=start_node, goal_node=goal_node)
        # ------------------------------------------------------

        traj, _, nodes, _ = apply_PRM(map_ref, nodes, visualize=visualize)
    # print('fresh trajectory:', traj)

    return traj, nodes, edges_all


def initialize_map(map_path):
    """
    initializes the reference map by applying the object detection
    @param map_path: path for the scanner map
    @return: reference map and the detected obstacles (with its borders)
    """
    map_ref, obstacles = apply_object_detection(map_path)
    # adding the box that the detection does not see
    order = [0, 1, 3, 2]
    # obstacles[0].corners = [obstacles[0].corners[i] for i in order]
    # add = [(1,1),(1,-1),(-1,-1),(-1,1)]
    corners_hand = [(102, 123), (103, 117), (94, 115), (92, 120)]
    # corners_hand = [tuple(map(lambda i,j:i+j,corners_hand[i],add[i])) for i in range(4)]
    obst_hand = Obstacle(corners_hand, label="table")
    obstacles.append(obst_hand)
    # --- only for test we use the already processed map ---
    # obstacles = None
    # ref_map_path = 'map_ref.png'
    # map_ref = Image.open(ref_map_path)
    # obstacle_adversary(obstacles,2)

    return map_ref, obstacles


def realCallback(data):
    global real_data
    real_data = [
        "real_data",
        data.pose.pose.position.x,
        data.pose.pose.position.y,
        data.pose.pose.orientation.z,  # this will be a value e[-1, 1] and can be converted [-pi, pi] with angle=arcsin(z)*2
        data.header.stamp,
    ]

    # print("--------------------------------------------------------------------")
    # print(real_data[1], real_data[2])


def calc_command(position, current_angle, target):
    current_angle = np.arcsin(current_angle) * 2  # from [-1, 1] to [-pi, pi]
    if current_angle < 0:  # from [-pi, pi] to [0, 2pi]
        current_angle = current_angle + (2 * np.pi)

    distance = np.linalg.norm(target - position)
    v = 0.1  # the factor f = (distance/cost) can be used to scale the intended speed
    # comm_angle is the rotation angle -> positive angles represent counter clockwise rotation
    # import pdb; pdb.set_trace()
    beta = np.arccos(np.abs(target[0] - position[0]) / distance)
    # print('beta:', beta)
    # important: in comparison to the "coordinates" of the image, the y-Axis is inverted, so it's different from my test-script
    if current_angle > np.pi:  # XXX NOT SURE ABOUT THIS YET!!!
        # print('current_angle in rad:', current_angle)
        current_angle = current_angle - 2 * np.pi
    if target[0] - position[0] >= 0 and target[1] - position[1] < 0:  # 4. Quadrant
        comm_angle = 2 * np.pi - (beta + current_angle)
        # print('4. Quadrant')
    elif target[0] - position[0] >= 0 and target[1] - position[1] >= 0:  # 1. Quadrant
        comm_angle = beta - current_angle
        # print('1. Quadrant')
    elif target[0] - position[0] < 0 and target[1] - position[1] < 0:  # 3. Quadrant
        comm_angle = np.pi + beta - current_angle
        # print('3. Quadrant')
    else:  # 2. Quadrant
        comm_angle = np.pi - (beta + current_angle)
        # print('2. Quadrant')
    if comm_angle > np.pi:
        # print('comm_angle was bigger than pi:', comm_angle)
        comm_angle = comm_angle - 2 * np.pi

    t = distance / v

    return comm_angle, distance


def move(dist):
    speed = 0.10

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


def rotate(angle):  # in this function I work with angle e[0, 2pi] !!!
    rot_speed = 10 / (360) * (2 * np.pi)  # 10 degrees/sec ???

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


def navigate_to_point(target, real_data_passed=None):
    if real_data_passed == None:
        global real_data
    else:
        real_data = real_data_passed
    # print(real_data)
    position = np.array([real_data[1], real_data[2]])
    current_angle = real_data[3]
    # print('position', position)
    # print('curr_angle', current_angle)
    comm_angle, comm_distance = calc_command(position, current_angle, target)
    # print('comm_angle', comm_angle)
    # print('comm_dist', comm_distance)
    print("----")
    print("now moving!!")
    rotate(comm_angle)
    move(comm_distance)
    print("arrived! --- target:", target)
    # print('actual position:', real_data)

    return comm_angle, comm_distance


def Image_Callback(img_data):
    # now = rospy.Time.now()
    if example.staticVariable:
        global Image_data
        global image_counter
        # global model
        # global deviation
        global center
        global conf
        global img_glob
        image_counter += 1

        bridge = CvBridge()

        cv_image = bridge.imgmsg_to_cv2(img_data, "rgb8")

        width = int(img_data.width * 0.80)
        height = int(img_data.height * 0.80)
        dim = (width, height)
        img_resized = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)
        img_glob = deepcopy(img_resized)

        Image_data = [
            "Image_Data",
            dim,  # dimensions of resized image
            img_resized,  # image data
            img_data.header.stamp,
        ]


def plot_boxes(results, frame):
    global center
    """
        Takes a frame and its results as input, and plots the bounding boxes and label on to the frame.
        :param results: contains labels and coordinates predicted by model on the given frame.
        :param frame: Frame which has been scored.
        :return: Frame with bounding boxes and labels ploted on it.
        """
    labels, cord = results.xyxyn[0][:, -1].numpy(), results.xyxyn[0][:, :-1].numpy()
    n = len(labels)  # no.of classes"
    x_shape, y_shape = frame.shape[1], frame.shape[0]
    for i in range(n):
        row = cord[i]

        if labels[i] == 3:
            # print(class_to_label(labels[i]))
            # print(labels[i])
            center = (row[0] + row[2]) / 2
        if row[4] >= 0.5:
            x1, y1, x2, y2 = (
                int(row[0] * x_shape),
                int(row[1] * y_shape),
                int(row[2] * x_shape),
                int(row[3] * y_shape),
            )
            bgr = (255, 0, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 2)
            cv2.putText(
                frame,
                class_to_label(labels[i]),
                (x1, y1),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                bgr,
                2,
            )

    return frame


def class_to_label(x):
    """
    This is for yolov5 version from thinesh
    For a given label value, return corresponding string label.
    :param x: numeric label
    :return: corresponding string label
    """
    if x == 0:
        y = "workstation"
    elif x == 1:
        y = "storage"
    elif x == 2:
        y = "table"
    elif x == 3:
        y = "box"
    elif x == 4:
        y = "robot"
    elif x == 5:
        y = "chair"
    return y


Pose_real_data = [0, 0, 0, 0]


def obstacle_adversary(obstacles, action_index):
    """
    initializes the reference map by applying the object detection
    @param obstacles: obstacles in the map
    @param action_index: action of the adversary
    @return: modified obstacles
    """
    obstacles_disturbed = deepcopy(obstacles)
    # print(obstacles_disturbed)
    for o in range(0, len(obstacles)):
        for i in range(0, len(obstacles[o].corners)):
            obstacles_disturbed[o].corners[i] = tuple(
                map(
                    lambda j, k: j + k,
                    obstacles_disturbed[o].corners[i],
                    action_index[o],
                )
            )
        for i in range(0, len(obstacles[o].borders)):
            obstacles_disturbed[o].borders[i][0] = tuple(
                map(
                    lambda j, k: j + k * np.cos(0),
                    obstacles_disturbed[o].borders[i][0],
                    action_index[o],
                )
            )
            obstacles_disturbed[o].borders[i][1] = tuple(
                map(
                    lambda j, k: j + k * np.cos(0),
                    obstacles_disturbed[o].borders[i][1],
                    action_index[o],
                )
            )
    # print(obstacles_disturbed)
    return obstacles_disturbed


def modify_map(
    map_ref,
    obstacles_org,
    obstacles,
    color=(255, 255, 255),
    convert_back_to_grey=True,
    savePath=None,
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
        corners = [
            tuple(map(lambda i, j: i + j, obstacle.corners[i], add[i]))
            for i in range(4)
        ]
        map_ref_adv_draw.polygon(corners, fill=(0, 0, 0), outline=(0, 0, 0))
    add = [(1, 1), (1, -1), (-1, -1), (-1, 1)]
    for obstacle in obstacles:
        # cv2.fillConvexPoly(self.map_ref_adv,obstacle.corners, color='white')
        # corners = [tuple(map(lambda i,j:i+j,obstacle.corners[i],add[i])) for i in range(4)]
        map_ref_adv_draw.polygon(obstacle.corners, fill=color, outline=color)
    if convert_back_to_grey:
        map_ref_adv = map_ref_adv.convert("L")

    if savePath != None:
        map_ref_adv = ImageOps.invert(map_ref_adv)
        map_ref_adv.save(savePath)
        rospy.logdebug(f"[Move Utils] Saved map_ref to {savePath}")
    return map_ref_adv


if __name__ == "__main__":
    print("This is a colletion of utils for robotino navigation")
