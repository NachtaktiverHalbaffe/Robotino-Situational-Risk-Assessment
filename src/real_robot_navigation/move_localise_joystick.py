import os, sys
from time import sleep
from gridmap import get_obstacles_in_pixel_map

sys.path.insert(0, "./yolov7")
from yolov7.detect_online import get_conf_and_model, loaded_detect
import matplotlib

from move_utils import *
from move_utils_cords import *


def realCallback(data):
    """This is used to define how to save the location arriving from ROS
    use this in a Subscriber"""
    global real_data
    real_data = [
        "real_data",
        data.pose.pose.position.x,
        data.pose.pose.position.y,
        data.pose.pose.orientation.z,  # this will be a value e[-1, 1] and can be converted [-pi, pi] with angle=arcsin(z)*2
        data.header.stamp,
    ]


def Image_Callback_v7(img_data):
    """This is used to define how to save the image arriving from ROS
    use this in a Subscriber"""
    # now = rospy.Time.now()
    if example.staticVariable:
        global Image_data
        global img_glob
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img_data, "rgb8")

        # Resize Image to 640 * 480 - YOLO was trained in this size
        width = int(img_data.width * 0.80)
        height = int(img_data.height * 0.80)
        dim = (width, height)
        img_resized = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)

        # The unmodified image
        img_glob = deepcopy(cv_image)

        Image_data = [
            "Image_Data",
            dim,  # dimensions of resized image
            img_resized,  # image data
            img_data.header.stamp,
        ]


def get_dists_workstation(corners_map_all, obstacle):
    """This gets the distances each workstation has to the ws placed by the detection
    @param corners_map_all: The corners of each workstation in a list
    @param obstacle: The detected ws as a distance from the camera
    @return: dists: array of the distances
    """
    dists = []
    arg_smallest_dist = 0
    smallest_dist = 10e10
    for corners, i in zip(corners_map_all, range(len(corners_map_all))):
        # distances between a workstations corners and the detected workstations corners
        dist = np.array(corners) - np.array(obstacle.corners)
        # pythagoras, root not needed because we just use this to sort
        dist = np.power(dist, 2)
        dist = np.sum(dist)
        dist = np.sum(dist)
        dist = np.sqrt(dist)
        dists.append(dist)
        if dist < smallest_dist:
            smallest_dist = dist
            arg_smallest_dist = i
    return np.array(dists)


def best_match_workstation_index(
    corners_map_all, obstacle, detection_corners, rots_ws, old_loc_assumption, rotation_detected, base_info
):
    """finds the workstation that best matches the detection based on the currently assumed postion
    @param corners_map_all: the corners of the workstations in the map
    @param obstacle: the detected workstation based on the old location
    @param detection_corners: the distance from detected workstation to the camera
    @param rots_ws: the rotation of the workstations on the map
    @param old_loc_assumption: where we currently assume to be based on acml
    @param rotation_detected: the rotation the object detection infered
    @param base_info: this is the set of calibration data needed to correctly transform from acml to pixel cords
    @return: detected_obst: the detected object as an Obstacle
    """
    dists_ws = get_dists_workstation(corners_map_all, obstacle)
    # arg_smallest_dist_rot, dists_rot = get_smalles_dist_rotations(corners_map_all, obstacle,old_loc_assumption)
    global map_config
    if old_loc_assumption[0] == "real_data" or True:
        old_rot_assumption = 2 * np.arcsin(old_loc_assumption[3])
    else:
        old_rot_assumption = local_acml_location[3]
    # calculating rotation from detected and cord transforms
    # detected_rotations = -(np.array(rots_ws)-rotation_detected.numpy()+np.pi-1.204)
    detected_rotations = -(np.array(rots_ws) - rotation_detected.numpy() + np.pi - map_config["rot"]) + 2 * np.pi
    dists_rot = deepcopy(detected_rotations)
    for rot, i in zip(detected_rotations, range(len(detected_rotations))):
        rot_shift = float(
            min(
                abs(rot + np.pi - (old_rot_assumption + np.pi)),
                abs(rot + 3 * np.pi - (old_rot_assumption + np.pi)),
                abs(rot + np.pi - (old_rot_assumption + 3 * np.pi)),
            )
        )
        dists_rot[i] = rot_shift
    # we get the detected localisation be subtracting the detected distances of a workstation from its location
    dists_loc = []
    for j in range(len(corners_map_all)):
        for i in range(
            len(detection_corners) - 3
        ):  # NOTE The -3 means we only go through once, all should be the same, useful for debug
            # change cords so we have the distance from the ws on the map
            detection_corner = convert_cam_to_robo(
                detection_corners[i][1], -detection_corners[i][0], detected_rotations[j]
            )
            # change cords
            ws_map = get_amcl_from_pixel_location(*corners_map_all[j][i], *base_info)
            # subtracting the detected distance from the workstation on the map
            loc_detec = (ws_map[0] - detection_corner[0], ws_map[1] - detection_corner[1])
            # loc_detec_acml = (ws_map[0]-corner[0],ws_map[1]-corner[1])
            old_loc_assumption_p = get_pixel_location_from_acml(
                old_loc_assumption[1], old_loc_assumption[2], *base_info
            )
            x_shift = loc_detec[0] - old_loc_assumption[1]
            y_shift = loc_detec[1] - old_loc_assumption[2]
            dist_shift = float(np.sqrt(np.power(x_shift, 2) + np.power(y_shift, 2)))
            dists_loc.append(dist_shift)
    dists_loc = np.array(dists_loc)
    # weighted sum to give each metric the same importance
    dists = np.sum(np.array([dists_ws, dists_rot * 100, dists_loc * 20]), axis=0)
    arg_smallest_dist = np.argmin(dists)
    return arg_smallest_dist


def get_obstacles_from_detection(detected_workstation_dist, local_acml_location, base_info):
    """turns the info provided by the object detection into an obstacle
    @param detected_workstation_dist: the distance from cam to each corner of the object
    @param local_acml_location: This is the current position in acml coordinate system
    @param base_info: this is the set of calibration data needed to correctly transform from acml to pixel cords
    @return: detected_obst: the detected object as an Obstacle
    """
    corners_detec = list(map(tuple, zip(*detected_workstation_dist)))
    rot = 2 * np.arcsin(local_acml_location[3])
    for i in range(len(corners_detec)):
        corner = convert_cam_to_robo(corners_detec[i][1], -corners_detec[i][0], rot)
        corner = (corner[0] + local_acml_location[1], corner[1] + local_acml_location[2])
        corner = get_pixel_location_from_acml(*corner, *base_info)
        corners_detec[i] = corner
    detected_obst = Obstacle(corners_detec)
    return detected_obst


def draw_map_location_acml(x, y, rot, draw_frame, base_info, color_FoV=(255, 0, 255), color_outer=(255, 0, 0)):
    """This function draws the robot onto the pixelmap
    @param x: the x cord in acml cord system
    @param y: the y cord in acml cord system
    @param ros: the the roation of the robot
    @param draw_frame: the image to draw on
    @param base_info: this is the set of calibration data needed to correctly transform from acml to pixel cords
    @param color_FoV: Color to draw the FoV window in
    @param color_outer: Color to draw the rest of the robot circle represenation in
    """
    global acml_x
    global acml_y
    global acml_rot
    write_to_cvs = False
    # we need to convert the location to pixels
    if color_FoV == (255, 0, 255):
        x, y = get_pixel_location_from_acml(x, y, *base_info, rot)
        acml_x = x
        acml_y = y
        acml_rot = rot
    else:
        x, y = get_pixel_location_from_acml(x, y, *base_info)
        # we only want to save the diff for the location recorded by the detection
        if write_to_cvs and color_FoV == (0, 0, 255):
            x_shift = float(x - acml_x)
            y_shift = float(y - acml_y)
            # The roation diff needs to account for wrapping around back to 0
            rot_shift = float(
                min(
                    abs(rot + np.pi - (acml_rot + np.pi)),
                    abs(rot + 3 * np.pi - (acml_rot + np.pi)),
                    abs(rot + np.pi - (acml_rot + 3 * np.pi)),
                )
            )
            dist_shift = float(np.sqrt(np.power(x_shift, 2) + np.power(y_shift, 2)))
            with open("./real_robot_navigation/error_dist_csvs/loc/error_dist_loc_x1.csv", "a") as f1:
                write = csv.writer(f1)
                write.writerow([x_shift])
            with open("./real_robot_navigation/error_dist_csvs/loc/error_dist_loc_y1.csv", "a") as f1:
                write = csv.writer(f1)
                write.writerow([y_shift])
            with open("./real_robot_navigation/error_dist_csvs/loc/error_dist_loc_rot_abs_chair1.csv", "a") as f1:
                write = csv.writer(f1)
                write.writerow([rot_shift])
            with open("./real_robot_navigation/error_dist_csvs/loc/error_dist_loc_dist1.csv", "a") as f1:
                write = csv.writer(f1)
                write.writerow([dist_shift])
            with open("./real_robot_navigation/error_dist_csvs/loc/error_dist_loc1.csv", "a") as f1:
                write = csv.writer(f1)
                write.writerow([x_shift, y_shift, rot_shift, dist_shift])
            if rot_shift < 0.20:
                with open("./real_robot_navigation/error_dist_csvs/loc/error_dist_loc_dist_filtered1.csv", "a") as f1:
                    write = csv.writer(f1)
                    write.writerow([dist_shift])
    # converting to degrees
    rot = rot / (2 * np.pi) * 360
    # the robot has a radius of about 20~25 so 5 pixels would also be fine
    robo_r = 4
    # drawing a circle with a cutout in a different color to represent the FoV
    draw_frame.pieslice(
        (x - robo_r, y - robo_r, x + robo_r, y + robo_r), start=0, end=360, fill=color_FoV, outline=color_FoV
    )
    draw_frame.pieslice(
        (x - robo_r, y - robo_r, x + robo_r, y + robo_r),
        start=60 - rot,
        end=300 - rot,
        fill=color_outer,
        outline=color_outer,
    )


def run_detec_and_localise_joystick(
    weights_detection, weights_localise, use_detection_cam=False, use_localise_cam=True, log_detection_error=False
):
    global acml_x
    global acml_y
    global acml_rot
    global img_glob
    global real_data
    global map_config
    # Loads the models
    if use_detection_cam:
        conf_detection = get_conf_and_model(weights_detection)
    if use_localise_cam:
        conf_localise = get_conf_and_model(weights_localise)
    # gets the set of calibration data that needs to measured for each new png map
    base_info, map_config = get_base_info()
    # the gridmap is the locations of the workstations alligned with the 'grid' of the floortiles
    obstacles_ws, names_ws = get_obstacles_in_pixel_map(base_info)
    # the movable obstacles that need to be placed in the robots way
    obstacles_movable, names_movables = get_obstacles_in_pixel_map(base_info, "movable")
    # rotation of the workstations in the gridmap TODO remove this line and get it into the obstables?
    rots_ws = [0, 0, 0.85, 1.57, 2.19, 2.19]
    # loading the png of the map and running the old detection system on it also configures the map to the right type
    map_path = map_config["path"]
    map_ref, obstacles = initialize_map(map_path)
    # removing the obstacles from the map, adding the workstations, removing old should not be needed if we have a new map, TODO new map add new obstacles such as klappbox, chair and box
    map_ref = modify_map(map_ref, obstacles, obstacles_ws, color=(255, 0, 255), convert_back_to_grey=True)
    # adding the movable objects
    map_ref = modify_map(map_ref, [], obstacles_movable, color=(255, 0, 0), convert_back_to_grey=True)
    # map_ref.save('./image/test_loc.png') if you want to see/save the map at this point

    # TODO this is for debugging the offset of inaccuracy postion of lidar vs center of robot
    matplotlib.use("QtAgg")

    # TODO This is to use the actuall ROS info
    rospy.init_node("test_loc", anonymous=True)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, realCallback, queue_size=10)
    rospy.sleep(1)
    sub = rospy.Subscriber("/image_raw", Image, Image_Callback_v7, queue_size=10)
    rospy.sleep(3)
    while not rospy.is_shutdown():
        # TODO This is for offline work, remove when finished with the calibration
        # real_data = ['fake',0.6,2.2,0.999] # 2.5 instead of 2 gets close
        # real_data = ['fake',-3.6,2.2,-0.0999] # 2.5 instead of 2 gets close
        # img_glob = cv2.imread('./yolov7/data/bb/img1.png')
        # while True:

        # copy newest image, the current postion, the workstations and the map TODO not sure if deepopy ws_map is needed
        img_local = deepcopy(img_glob)
        dc_obstacles_ws = deepcopy(obstacles_ws)
        local_acml_location = deepcopy(real_data)
        local_acml_location = offset_to_robo(local_acml_location)

        # copy map and create a drawing frame
        map_ref_loc = deepcopy(map_ref).convert("RGB")
        map_ref_loc_draw = ImageDraw.Draw(map_ref_loc)

        if use_detection_cam:
            detec_movables = loaded_detect(img_local, *conf_detection, True)
            detec_movables_obstacles = []
            rotations_detected = []
            index_names = []
            for detec_m in detec_movables:
                index_names.append(names_movables.index(detec_m["label"]))
                detec_movables_obstacle = get_obstacles_from_detection(
                    detec_m["birds_eye"], local_acml_location, base_info
                )
                detec_movables_obstacles.append(detec_movables_obstacle)
                rotations_detected.append(detec_m["rotation"])
            if detec_movables_obstacles:
                objects_to_move = [obstacles_movable[i] for i in index_names]
                # NOTE this is debug code,remove to correcly move object
                # objects_to_move = []
                map_ref_loc = modify_map(
                    map_ref_loc, [], detec_movables_obstacles, color=(0, 255, 255), convert_back_to_grey=False
                )
                # map_ref_loc = modify_map(map_ref_loc,objects_to_move,detec_movables_obstacles,color=(0,255,255),convert_back_to_grey=False)
                map_ref_loc_draw = ImageDraw.Draw(map_ref_loc)
            if log_detection_error and detec_movables_obstacles:
                for ground_truth, detection_assumption, rotation_detected in zip(
                    objects_to_move, detec_movables_obstacles, rotations_detected
                ):
                    # here we need to split stuff into x,y
                    g_truth = np.array(ground_truth.corners)
                    detec_assum = np.array(detection_assumption.corners)
                    # summing over the corners and then subtracting before deviding by number of corners gives the distances of centers
                    differences = np.sum(g_truth, axis=0) / 4 - np.sum(detec_assum, axis=0) / 4
                    differences_pyt = np.sqrt(np.sum(np.power(differences, 2)))
                    with open("./real_robot_navigation/error_dist_csvs/error_dist_detec_22_12.csv", "a") as f1:
                        write = csv.writer(f1)
                        write.writerow([differences[0], differences[1], differences_pyt])
                    with open(
                        "./real_robot_navigation/error_dist_csvs/error_dist_detec_22_12_rot_door_closed.csv", "a"
                    ) as f1:
                        write = csv.writer(f1)
                        old_rot_assumption = 2 * np.arcsin(local_acml_location[3])
                        rot = -(0 - rotation_detected.numpy() + np.pi - map_config["rot"]) + 2 * np.pi
                        rot_shift = float(
                            min(
                                abs(rot + np.pi - (old_rot_assumption + np.pi)),
                                abs(rot + 3 * np.pi - (old_rot_assumption + np.pi)),
                                abs(rot + np.pi - (old_rot_assumption + 3 * np.pi)),
                            )
                        )
                        error = min(abs(rot_shift), abs(rot_shift - np.pi / 2), abs(rot_shift - np.pi))
                        print(error)
                        write.writerow([error])

        # convert the amcl pose rotation to radians
        amcl_rot_radians = 2 * np.arcsin(local_acml_location[3])
        draw_map_location_acml(
            local_acml_location[1], local_acml_location[2], amcl_rot_radians, map_ref_loc_draw, base_info
        )

        # corners of the workstations on the map
        corners_map_all = [obstacle.corners for obstacle in dc_obstacles_ws]

        if use_localise_cam:
            # getting the detection based on the newest image
            localise_dict = loaded_detect(img_local, *conf_localise)
            # if there was a detection
            if localise_dict:
                detected_workstation_dist, rotation_detected = localise_dict["birds_eye"], localise_dict["rotation"]
                # print(detected_workstation_dist)
                # turning the detected corners into an obstacle
                detected_obst = get_obstacles_from_detection(detected_workstation_dist, local_acml_location, base_info)
                # comparing detected obstacles with workstations on map to find correct one
                detection_corners = list(map(tuple, zip(*detected_workstation_dist)))
                index_smallest_dist_ws = best_match_workstation_index(
                    corners_map_all,
                    detected_obst,
                    detection_corners,
                    rots_ws,
                    local_acml_location,
                    rotation_detected,
                    base_info,
                )

                # now we want to show the matched ws in blue, the acml location in green
                map_ref_loc = modify_map(
                    map_ref_loc,
                    [],
                    [obstacles_ws[index_smallest_dist_ws]],
                    color=(255, 0, 0),
                    convert_back_to_grey=False,
                )
                map_ref_loc = modify_map(
                    map_ref_loc, [], [detected_obst], color=(0, 255, 0), convert_back_to_grey=False
                )
                map_ref_loc_draw = ImageDraw.Draw(map_ref_loc)

                # calculating rotation from detected and cord transforms
                # detected_rotation = -(rots_ws[index_smallest_dist_ws]-rotation_detected+np.pi-1.204)
                detected_rotation = (
                    -(rots_ws[index_smallest_dist_ws] - rotation_detected + np.pi - map_config["rot"]) + 2 * np.pi
                )
                # basicly just transpose for the list
                corners_detec_2 = list(map(tuple, zip(*detected_workstation_dist)))
                # we get the detected localisation be subtracting the detected distances of a workstation from the actual one
                for i in range(
                    len(corners_detec_2) - 3
                ):  # NOTE The -3 means we only go through once, all should be the same, useful for debug
                    # change cords so we have the distance from the ws on the map
                    corner = convert_cam_to_robo(corners_detec_2[i][1], -corners_detec_2[i][0], detected_rotation)
                    # change cords
                    ws_map = get_amcl_from_pixel_location(*corners_map_all[index_smallest_dist_ws][i], *base_info)
                    # subtracting the detected distance from the workstation on the map
                    loc_detec = (ws_map[0] - corner[0], ws_map[1] - corner[1])
                # print(loc_detec)
                print("adding to map")
                draw_map_location_acml(
                    *loc_detec,
                    detected_rotation,
                    map_ref_loc_draw,
                    base_info,
                    color_FoV=(0, 0, 255),
                    color_outer=(0, 255, 0)
                )

                print(detected_rotation, np.arcsin(local_acml_location[3]) * 2)
        # map_ref_loc.save('./image/test_loc_circle.png')
        cv2.imshow("map", np.kron(np.asarray(map_ref_loc.convert("RGB")), np.ones((2, 2, 1))))
        cv2.waitKey(1)
        # sleep(0.5)# in seconds
        # sleep(100000)


if __name__ == "__main__":
    use_detection_cam = True
    use_localise_cam = True
    weights_detection = "/home/nachtaktiverhalbaffe/dev/catkin_ws/src/robotino/src/yolov7/weights/tiny10_hocker.pt"
    weights_localise = "/home/nachtaktiverhalbaffe/dev/catkin_ws/src/robotino/src/yolov7/weights/ws_tiny5.pt"

    run_detec_and_localise_joystick(
        weights_detection, weights_localise, use_detection_cam=use_detection_cam, use_localise_cam=use_localise_cam
    )
