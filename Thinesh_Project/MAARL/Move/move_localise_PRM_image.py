"""author Kai Binder
"""
import os, sys
from time import sleep
from gridmap import get_obstacles_in_pixel_map
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__))+'/yolov7')
from detect_online import get_conf_and_model, loaded_detect
import matplotlib

from move_utils import *
from move_utils_cords import *

def realCallback(data):
    """This is used to define how to save the location arriving from ROS
    use this in a Subscriber"""
    global real_data
    real_data = ['real_data',
                 data.pose.pose.position.x, 
                 data.pose.pose.position.y,
                 data.pose.pose.orientation.z,  # this will be a value e[-1, 1] and can be converted [-pi, pi] with angle=arcsin(z)*2
                 data.header.stamp]

def Image_Callback_v7(img_data):
    """This is used to define how to save the image arriving from ROS
    use this in a Subscriber"""
    # now = rospy.Time.now()
    if(example.staticVariable):
        global Image_data
        global img_glob
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img_data,"rgb8")


        # Resize Image to 640 * 480 - YOLO was trained in this size
        width = int(img_data.width * 0.80)
        height = int(img_data.height * 0.80)
        dim = (width, height)
        img_resized = cv2.resize(cv_image, dim, interpolation = cv2.INTER_AREA)

        # The unmodified image
        img_glob = deepcopy(cv_image)

        Image_data = ['Image_Data',
                    dim, # dimensions of resized image
                    img_resized,  # image data
                    img_data.header.stamp]

def get_dists_workstation(corners_map_all, obstacle):
    """This gets the distances each workstation has to the ws placed by the detection
    @param corners_map_all: The corners of each workstation in a list
    @param obstacle: The detected ws as a distance from the camera
    @return: dists: array of the distances
    """
    dists = []
    arg_smallest_dist = 0
    smallest_dist = 10e10
    for corners,i in zip(corners_map_all, range(len(corners_map_all))):
        # distances between a workstations corners and the detected workstations corners
        dist = np.array(corners)-np.array(obstacle.corners)
        # pythagoras, root not needed because we just use this to sort
        dist = np.power(dist,2)
        dist = np.sum(dist)
        dist = np.sum(dist)
        dist = np.sqrt(dist)
        dists.append(dist)
        if dist < smallest_dist:
            smallest_dist = dist
            arg_smallest_dist = i
    return np.array(dists)
    
def best_match_workstation_index(corners_map_all, obstacle, rots_ws, old_loc_assumption, rotation_detected):
    """
    This gets determins which workstation we are currently looking at by comparing the distance of the
    workstation(ws) location and roation as well as the robots location
    @param corners_map_all: The corners of each workstation in a list
    @param obstacle: The detected ws as a distance from the camera
    @param ros_ws: the the roation of the individual workstations
    @param old_loc_assumption: where we assume to be, used to determin which ws we are looking at,
    @param rotation_detected: the roation detected by the detection algo
    @return: arg_smallest_dist: the index of the ws with the most likely match
    """

    dists_ws = get_dists_workstation(corners_map_all, obstacle)
    # arg_smallest_dist_rot, dists_rot = get_smalles_dist_rotations(corners_map_all, obstacle,old_loc_assumption)
    # NOTE at the moment that is always right because i change the others before sending it
    if local_acml_location[0] =='real_data' or True:
        old_rot_assumption = 2*np.arcsin(local_acml_location[3])
    else:
        old_rot_assumption = local_acml_location[3]
    # calculating rotation from detected and cord transforms
    detected_rotations = -(np.array(rots_ws)-rotation_detected.numpy()+np.pi-1.204)+np.pi*2
    dists_rot = deepcopy(detected_rotations)
    for rot, i in zip(detected_rotations,range(len(detected_rotations))):
        rot_shift = float(min(abs(rot+np.pi-(old_rot_assumption+np.pi)),abs(rot+3*np.pi-(old_rot_assumption+np.pi)),abs(rot+np.pi-(old_rot_assumption+3*np.pi))))
        dists_rot[i] = rot_shift
    # we get the detected localisation be subtracting the detected distances of a workstation from its location
    corners_detec_2 = obstacle.corners
    dists_loc = []
    for j in range(len(corners_map_all)):
        for i in range(len(corners_detec_2)-3): # NOTE The -3 means we only go through once, all should be the same, useful for debug
            # change cords so we have the distance from the ws on the map
            corner = convert_cam_to_robo(corners_detec_2[i][1],-corners_detec_2[i][0], detected_rotations[j])
            # change cords
            ws_map = get_amcl_from_pixel_location(*corners_map_all[j][i],*base_info)
            # subtracting the detected distance from the workstation on the map                
            loc_detec_p = (corners_map_all[j][i][0]-corner[0],corners_map_all[j][i][1]-corner[1])
            loc_detec_acml = (ws_map[0]-corner[0],ws_map[1]-corner[1])
            x_shift = loc_detec_p[0]-old_loc_assumption[1]
            y_shift = loc_detec_p[1]-old_loc_assumption[2]
            dist_shift = float(np.sqrt(np.power(x_shift,2)+np.power(y_shift,2)))
            dists_loc.append(dist_shift)
    dists_loc = np.array(dists_loc)
    # weighted sum to give each metric the same importance #*2 to have rotation,loc be more important
    dists = np.sum(np.array([dists_ws,dists_rot*100*2,dists_loc]),axis=0)
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
    rot = 2*np.arcsin(local_acml_location[3])
    for i in range(len(corners_detec)):
        corner = convert_cam_to_robo(corners_detec[i][1],-corners_detec[i][0], rot)
        corner = (corner[0]+local_acml_location[1],corner[1]+local_acml_location[2])
        corner = get_pixel_location_from_acml(*corner,*base_info)
        corners_detec[i] = corner
    detected_obst = Obstacle(corners_detec)
    return detected_obst

def draw_map_location_acml(x,y,rot,draw_frame,base_info,color_FoV=(255, 0, 255),color_outer=(255, 0, 0)):
    """ This function draws the robot onto the pixelmap
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
    if color_FoV==(255, 0, 255):
        x,y = get_pixel_location_from_acml(x,y,*base_info,rot)
        acml_x = x
        acml_y = y
        acml_rot = rot
    else:
        x,y = get_pixel_location_from_acml(x,y,*base_info)
        # we only want to save the diff for the location recorded by the detection
        if write_to_cvs and color_FoV==(255, 0, 0):
            x_shift = float(x-acml_x)
            y_shift = float(y-acml_y)
            # The roation diff needs to account for wrapping around back to 0
            rot_shift = float(min(abs(rot+np.pi-(acml_rot+np.pi)),abs(rot+3*np.pi-(acml_rot+np.pi)),abs(rot+np.pi-(acml_rot+3*np.pi))))
            dist_shift = float(np.sqrt(np.power(x_shift,2)+np.power(y_shift,2)))
            with open('dist_loc/error_dist_loc_x5.csv','a') as f1:
                write = csv.writer(f1)
                write.writerow([x_shift])
            with open('dist_loc/error_dist_loc_y5.csv','a') as f1:
                write = csv.writer(f1)
                write.writerow([y_shift])
            with open('dist_loc/error_dist_loc_rot_abs_chair5.csv','a') as f1:
                write = csv.writer(f1)
                write.writerow([rot_shift])
            with open('dist_loc/error_dist_loc_dist5.csv','a') as f1:
                write = csv.writer(f1)
                write.writerow([dist_shift])
            with open('dist_loc/error_dist_loc5.csv','a') as f1:
                write = csv.writer(f1)
                write.writerow([x_shift,y_shift,rot_shift,dist_shift])
            if rot_shift<0.20:
                with open('dist_loc/error_dist_loc_dist_filtered5.csv','a') as f1:
                    write = csv.writer(f1)
                    write.writerow([dist_shift])
    # converting to degrees
    rot = rot/(2*np.pi)*360
    # the robot has a radius of about 20~25 so 5 pixels would also be fine
    robo_r = 4
    # drawing a circle with a cutout in a different color to represent the FoV
    draw_frame.pieslice((x-robo_r, y-robo_r, x+robo_r, y+robo_r), start=0, end=360, fill=color_FoV, outline=color_FoV)
    draw_frame.pieslice((x-robo_r, y-robo_r, x+robo_r, y+robo_r), start=60-rot, end=300-rot, fill=color_outer, outline=color_outer)



if __name__ == '__main__':
    """This code is used to have the robot navigate from a known starting position to any end position using the object detection"""
    global acml_x
    global acml_y
    global acml_rot
    global img_glob
    global real_data
    global odom_real_data
    # Loads the model
    conf = get_conf_and_model()
    # gets the set of calibration data that needs to measured for each new png map
    base_info = get_base_info()
    # the gridmap is the locations of the workstations alligned with the 'grid' of the floortiles
    obstacles_ws = get_obstacles_in_pixel_map(base_info)
    # the movable obstacles that need to be placed in the robots way
    obstacles_movable = get_obstacles_in_pixel_map(base_info, 'movable')
    # all obstacles for PRM
    all_obst = deepcopy(obstacles_ws)+deepcopy(obstacles_movable)
    #rotation of the workstations in the gridmap TODO remove this line and get it into the obstables?
    rots_ws = [0,0,0.85,1.57,2.19,2.19]
    # loading the png of the map and running the old detection system on it also configures the map to the right type
    map_path = "/home/kaib/catkin_ws/src/FinalScannerMap1.png"
    map_ref, obstacles = initialize_map(map_path)
    # removing the obstacles from the map, adding the workstations, removing old should not be needed if we have a new map, TODO new map add new obstacles such as klappbox, chair and box
    map_ref_PRM = modify_map(map_ref, obstacles, obstacles_ws, color = (255,255,255))
    map_ref = modify_map(map_ref, obstacles, obstacles_ws, color = (255,0,255))
    # adding the movable objects
    map_ref_PRM = modify_map(map_ref_PRM, [], obstacles_movable, color = (255,255,255))
    map_ref = modify_map(map_ref, [], obstacles_movable, color = (255,0,0))
    # map_ref.save('./image/test_loc.png') if you want to see/save the map at this point

    # this is for debugging the offset of inaccuracy: postion of lidar vs center of robot
    matplotlib.use('QtAgg')

    # TODO This is to use the actuall ROS info
    rospy.init_node('test_loc_odom', anonymous=True)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, realCallback, queue_size=10)
    rospy.sleep(1)
    sub = rospy.Subscriber('/image_raw', Image, Image_Callback_v7, queue_size=10)
    rospy.sleep(3)
    #we use the acml locaion as our start postion
    local_acml_location = deepcopy(real_data)# TODO is debug
    new_rotation = 2*np.arcsin(real_data[3])
    new_loc = (real_data[1],real_data[2])
    change_angle = 0
    change_dist = 0
    # if the location was updated by the image sice the last movement
    updated_location = False
    # while not rospy.is_shutdown():

    start = get_pixel_location_from_acml(*new_loc, *base_info)
    start = ([int(start[0]),int(start[1])])
    # init_nodes for the new map and get the trajectory from point one to point two TODO FIXME add the start and End NODE
    trajectory_vanilla, nodes_vanilla, edges_all_vanilla = initialize_traj(map_ref_PRM,all_obst, nodes=None,start = start, end = [88,127])
    traj_updt = []
    # get the updated trajetory in meters instead of pixels
    for i in range(len(trajectory_vanilla)):
        traj =  get_amcl_from_pixel_location(trajectory_vanilla[i].x,trajectory_vanilla[i].y,*base_info)
        traj_updt.append(traj)
    # move along the traj
    for trajectory_node, counter in zip(traj_updt[1:], range(0,len(traj_updt)-1)):
        
        # showing the traj in the map
        if counter <len(traj_updt)-1:
            map_ref_PRM = map_ref_PRM.convert('RGB')
            visu_adv_traj_map_draw = ImageDraw.Draw(map_ref_PRM)
            visu_adv_traj_map_draw.line([(trajectory_vanilla[counter].coordinates[0], trajectory_vanilla[counter].coordinates[1]), (trajectory_vanilla[counter+1].coordinates[0], trajectory_vanilla[counter+1].coordinates[1])], fill=(255, 255, 0))
            visu_adv_traj_map_draw.point([(trajectory_vanilla[counter].coordinates[0], trajectory_vanilla[counter].coordinates[1])], fill=(200, 255, 0))
            visu_adv_traj_map_draw.point([(trajectory_vanilla[counter+1].coordinates[0], trajectory_vanilla[counter+1].coordinates[1])], fill=(200, 255, 0))
        cv2.imshow('map_traj', np.asarray(map_ref_PRM))
        cv2.waitKey(1)

        # copy newest image, the current postion, the workstations and the map TODO not sure if deepopy ws_map is needed
        img_local = deepcopy(img_glob)
        dc_obstacles_ws = deepcopy(obstacles_ws)
        local_acml_location = deepcopy(real_data)
        # copy map and create a drawing frame
        

        
        # convert the amcl pose rotation to radians and then draw it on the map
        amcl_rot_radians = 2*np.arcsin(local_acml_location[3])
        map_ref_loc = deepcopy(map_ref).convert('RGB')
        map_ref_loc_draw = ImageDraw.Draw(map_ref_loc)
        draw_map_location_acml(local_acml_location[1],local_acml_location[2],amcl_rot_radians,map_ref_loc_draw,base_info)

        # update posion by combining the last detection with the last move command
        if updated_location:
            new_rotation = detected_rotation+change_angle
        else:
            new_rotation = new_rotation+change_angle
        # just making sure that the range is [-pi,pi]
        if new_rotation<(-np.pi):
            new_rotation = new_rotation+np.pi*2
        if new_rotation>(np.pi):
            new_rotation = new_rotation-np.pi*2
        draw_map_location_acml(*traj_updt[counter],new_rotation,map_ref_loc_draw,base_info,color_FoV=(255, 0, 0), color_outer=(0,255,0))

        # corners of the workstations on the map
        corners_map_all = [obstacle.corners for obstacle in dc_obstacles_ws]

        # getting the detection based on the newest image
        detected_workstation_dist, rotation_detected  = loaded_detect(img_local,*conf)
        # if there was a detection
        if not detected_workstation_dist is None:
            # turning the detected corners into an obstacle
            new_loc_as_real = ['new_loc',traj_updt[counter][0],traj_updt[counter][1],np.sin(new_rotation/2)]
            # detected_obst = get_obstacles_from_detection(detected_workstation_dist,new_loc_as_real, base_info)
            detected_obst = get_obstacles_from_detection(detected_workstation_dist,new_loc_as_real, base_info)
            # comparing detected obstacles with workstations on map to find correct one
            index_smallest_dist_ws = best_match_workstation_index(corners_map_all, detected_obst, rots_ws, new_loc_as_real, rotation_detected)

            # now we want to show the matched ws in blue, the acml location in green
            map_ref_loc = modify_map(map_ref_loc,[],[obstacles_ws[index_smallest_dist_ws]],color=(255,0,0),convert_back_to_grey=False)
            map_ref_loc = modify_map(map_ref_loc,[],[detected_obst],color=(0,255,0),convert_back_to_grey=False)
            map_ref_loc_draw = ImageDraw.Draw(map_ref_loc)

            # calculating rotation from detected and cord transforms
            detected_rotation = -(rots_ws[index_smallest_dist_ws]-rotation_detected+np.pi-1.204)
            # basicly just transpose for the list
            corners_detec_2 = list(map(tuple, zip(*detected_workstation_dist)))
            # we get the detected localisation be subtracting the detected distances of a workstation from the actual one
            for i in range(len(corners_detec_2)-3): # NOTE The -3 means we only go through once, all should be the same, useful for debug
                # change cords so we have the distance from the ws on the map
                corner = convert_cam_to_robo(corners_detec_2[i][1],-corners_detec_2[i][0], detected_rotation)
                # change cords
                ws_map = get_amcl_from_pixel_location(*corners_map_all[index_smallest_dist_ws][i],*base_info)
                # subtracting the detected distance from the workstation on the map                
                loc_detec = (ws_map[0]-corner[0],ws_map[1]-corner[1])
            draw_map_location_acml(*loc_detec,detected_rotation,map_ref_loc_draw,base_info,color_FoV=(0, 0, 255), color_outer=(0,255,0))
            updated_location = True
        else:
            updated_location = False
        # map_ref_loc.save('./image/test_loc_circle.png')
        cv2.imshow('map', np.kron(np.asarray(map_ref_loc.convert('RGB')),np.ones((2,2,1))))
        cv2.waitKey(100)

        # navigating to the new point, returns change that needs to be added to the location we detected to get the new one
        if updated_location:
            if detected_rotation<(-np.pi):
                detected_rotation = detected_rotation+np.pi*2
            if new_rotation>(np.pi):
                detected_rotation = detected_rotation-np.pi*2
            detec_as_real = ['dete',loc_detec[0],loc_detec[1],np.sin(detected_rotation/2)]
        else:
            if new_rotation<(-np.pi):
                new_rotation = new_rotation+np.pi*2
            if new_rotation>(np.pi):
                new_rotation = new_rotation-np.pi*2
            detec_as_real = ['dete',traj_updt[counter][0],traj_updt[counter][1],np.sin(new_rotation/2)]
        # now we drive to the next node in a straight line from our current position
        change_angle, change_dist = navigate_to_point(trajectory_node,detec_as_real)




