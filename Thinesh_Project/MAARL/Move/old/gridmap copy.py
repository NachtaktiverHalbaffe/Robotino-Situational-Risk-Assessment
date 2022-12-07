import numpy as np
from object_detection import Obstacle
from move_utils import *

def get_objects():
    objects = []
    ex_1 = {'name': 'wm4',
            'size': np.array([0.96, 1.15, 0.80]),
            'location': np.array([-0.42+2, 0.0, 4.32-2.5]),
            'angle': 1.57}
    ex_2 = {'name': 'wm4',
            'size': np.array([0.96, 1.15, 0.80]),
            'location': np.array([-0.42+2, 0.0, 4.32]),
            'angle': 1.57}
    ex_3 = {'name': 'ex3',
            'size': np.array([0.96, 1.15, 0.80]),
            'location': np.array([-1.92, 0.0, 4.32]),
            'angle': 1.57}
#     objects.append(ex_1)
#     objects.append(ex_2)
#     objects.append(ex_3)

    wm_1 = {'name': 'wm1',
            'size': np.array([0.96, 1.15, 0.80]),
            'location': np.array([-2.90, 0.0, 1.27]),
            'angle': 0}
    wm_2 = {'name': 'wm2',
            'size': np.array([0.96, 1.15, 0.80]),
            'location': np.array([-2.90, 0.0, 2.50]),
            'angle': 0}
    wm_3 = {'name': 'wm3',
            'size': np.array([0.96, 1.15, 0.80]),
            'location': np.array([-2.855+0.124, 0.0, 3.72+0.69]),
            'location': np.array([-2.855+0.49, 0.0, 3.72+0.12]),
            'angle': 0.785}
    wm_4 = {'name': 'wm4',
            'size': np.array([0.96, 1.15, 0.80]),
            'location': np.array([-0.42-0, 0.0, 4.32]),
            'angle': 1.57}
    wm_5 = {'name': 'wm5',
            'size': np.array([0.96, 1.15, 0.80]),
            'location': np.array([1.18-0, 0.0, 4.12]),
            'angle': 2.09}
    wm_6 = {'name': 'wm5',
            'size': np.array([0.96*2, 1.15, 0.80]),
            'location': np.array([2.23-0, 0.0, 3.52]),
            'angle': 2.09}
    checkerboard = {'name': 'checkerboard',
                    'size': np.array([0.001, 0.6, 0.6]),
                    'location': np.array([0.30, 0.0, 2.10]),
                    'angle': 0.0}

    objects.append(wm_1)
    objects.append(wm_2)
    objects.append(wm_3)
    objects.append(wm_4)
    objects.append(wm_5)
    objects.append(wm_6)
    # objects.append(checkerboard)

    return objects


def get_FoV_points():
    outer_points = []
    lf = {'name': 'left_front',
          'size': np.array([0.001, 0.2, 0.2]),
          'location': np.array([-0.59, 0.0, 1.45]),
          'angle': 0.0}
    rf = {'name': 'right_front',
          'size': np.array([0.001, 0.2, 0.2]),
          'location': np.array([0.80, 0.0, 1.380]),
          'angle': 0.0}
    rb = {'name': 'right_back',
          'size': np.array([0.001, 0.2, 0.2]),
          'location': np.array([3.35, 0.0, 5.8]),
          'angle': 0.0}
    lb = {'name': 'left_back',
          'size': np.array([0.001, 0.2, 0.2]),
          'location': np.array([-2.35, 0.0, 5.85]),
          'angle': 0.0}
    outer_points.append(lf)
    outer_points.append(rf)
    outer_points.append(rb)
    outer_points.append(lb)
    return outer_points

def corners_from_center(x,y,rotation,sizes):
    # x = depth, y = horizontal shift
    # compute rotational matrix around yaw axis
    R = np.array([[+np.cos(rotation), 0, +np.sin(rotation)],
                    [0, 1,               0],
        [-np.sin(rotation), 0, +np.cos(rotation)]])

    # 3D bounding box dimensions
    l = sizes[0];
    w = sizes[1];
    h = sizes[2];

    # shift for each corner
    z_corners = [l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2];
    y_corners = [0,0,0,0,-h,-h,-h,-h];
    x_corners = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2];

    # rotate and translate 3D bounding box
    corners_3D = np.dot(R,np.array([x_corners,y_corners,z_corners]))
    corners_3D[0,:] = corners_3D[0,:] + y#object.t(0);
    corners_3D[1,:] = corners_3D[1,:] + 0.43#.42#object.t(1);
    corners_3D[2,:] = corners_3D[2,:] + x#object.t(2);
    return corners_3D



def object_grid_to_pixel(pixel_map_info,object):

    left_close_corner =  object['location']
    # here we correct the coordinate system    
    d_real = object['size'][2]
    w_real = object['size'][1]
    h_real = object['size'][0]
    size = (d_real,w_real,h_real)
    rotation = object['angle']
    R = np.array([[+np.cos(rotation), 0, +np.sin(rotation)],
                    [0, 1,               0],
        [-np.sin(rotation), 0, +np.cos(rotation)]])
    dist_to_center = np.dot(R,np.array(size))
    depth = left_close_corner[0]+2.35# should be more
    horizontal_shift = left_close_corner[2]-0.6
    corner_points = corners_from_center(depth,horizontal_shift,-rotation,size)

    corners = []
    corners.append(corner_points[:,7])
    corners.append(corner_points[:,6])
    corners.append(corner_points[:,5])
    corners.append(corner_points[:,4])

    pixel_corners = []
    for corner in corners:
        pixel_corners.append(get_pixel_location_from_acml(*convert_grid_to_robo(corner[2], corner[0]),*pixel_map_info))

    return pixel_corners
    # we then want to return the corners beause that is what obstales needs as a init
    # sort so that we start with max max



def get_obstacles_in_pixel_map(pixel_map_info):
    grid_objects = get_objects()
    pixel_corners_all = []
    for object in grid_objects:
        # translate and rotate the objects to match the pixel map given by base info
        pixel_corners_all.append(object_grid_to_pixel(pixel_map_info,object))
    # for object in grid_objects[0:2]:
    #     pixel_corners_all.append(object_grid_to_pixel_2(pixel_map_info,object))
    # have the objects be obstacles
    obstacles = []
    for corners in pixel_corners_all:
        obstacles.append(Obstacle(corners))
    
    return obstacles


