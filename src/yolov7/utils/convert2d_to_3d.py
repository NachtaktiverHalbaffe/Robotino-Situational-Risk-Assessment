# converts the 2d boxes into 3d boxes
import math
import numpy as np 
import torch

def infer_depth(top,bot,h_real):
    """infer the distance of the object based on the total hight of the object"""
    hight = top-bot
    a = 651.6301600000004*h_real/0.8
    return a/np.abs(hight)

def infer_depth_from_top(top,h_real):
    """infer the distance of the object based on how high it is relative to the vanishing point"""
    offset = 245
    print(600-top)
    hight = 600-top-offset
    a = 420.17*(h_real-0.415)/(0.95-0.415)
    return a/np.abs(hight)

def infer_width(shift_p, depth):
    """infer the horizontal shift of a point based on the distance of the object to the camera at that pixel"""
    a_y = 0.0012277075719758462
    offset_y = 400
    return (shift_p-offset_y)*a_y*depth

def infer_rot(top,bot,d,w,dir):
    """calculate the extra distance the far corner must have for the object to fit in the bounding box"""
    # requires the outer pixel values as well as the depth and width of the object
    l = top[0]
    r = bot[0]
    offset_y = 400
    l = l-offset_y
    r = r-offset_y
    a= 0.0012277075719758462
    if dir =='c':
        y = (a**2*d*l*(r-l) + np.sqrt((a**2)*(l**2*(w**2)-d**2*(l-r)**2)+w**2))/(a**2*(l**2)+1)
    elif(dir =='a'):
        y = (a**2*d*r*(l-r) + np.sqrt((a**2)*(r**2*(w**2)-d**2*(l-r)**2)+w**2))/(a**2*(r**2)+1)
    else:
        print('except unknown roation, name either c or a')
    return y

def cube_rules(xyxy, label,h_real,w_real,d_real,use_pure_hight):
    top, bot = xyxy[0:2], xyxy[2:4]
    depth = infer_depth(top[1],bot[1],h_real)
    if use_pure_hight and bot[1]>595:
        depth = infer_depth_from_top(top[1],h_real)
    if label[-1]== 'c':
        shift = infer_width(bot[0],depth)
        y_diff = infer_rot(top,bot,depth,w_real,'c')
        rot = np.arcsin(y_diff/w_real)
        if math.isnan(rot):
            rot = torch.tensor(0,dtype=torch.float32)
        depth_c = depth+np.sin(rot)*w_real/2+np.cos(rot)*d_real/2
        shift_c = shift-np.cos(rot)*w_real/2+np.sin(rot)*d_real/2
        # if the object is likley not fully in frame change start point for possible states to a visible point
        # if bot[0]>795:
        #     shift = infer_width(top[0],depth)
        #     shift = shift+w_real/2
        # else:
        #     shift = shift-w_real/2
        # depth = depth+d_real/2

    # if the roation is in the other direction
    elif label[-1] == 'a':
        shift = infer_width(top[0],depth)
        y_diff = infer_rot(top,bot,depth,w_real,'a')
        rot = np.arcsin(y_diff/w_real)
        rot = -rot
        if math.isnan(rot): 
            rot = torch.tensor(0,dtype=torch.float32)
        depth_c = depth+np.sin(-rot)*w_real/2+np.cos(-rot)*d_real/2
        shift_c = shift+np.cos(-rot)*w_real/2-np.sin(-rot)*d_real/2
        # if top[0]<5:
        #     shift = infer_width(bot[0],depth)
        #     shift = shift-w_real/2
        # else:
        #     shift = shift+w_real/2        
        # depth = depth+d_real/2
    else:
        shift = 100000
    return depth, shift, depth_c, shift_c, rot, [d_real,w_real,h_real]

def corners_from_center(x,y,rotation,sizes):
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
    corners_3D[0,:] = corners_3D[0,:] + y.numpy()#object.t(0);
    corners_3D[1,:] = corners_3D[1,:] + 0.43#.42#object.t(1);
    corners_3D[2,:] = corners_3D[2,:] + x.numpy()#object.t(2);
    return corners_3D

def apply_for_cube(xyxy,label, corners_3D,boundry,h_real,w_real,d_real):
    use_pure_hight = True
    depth, shift, depth_c, shift_c, rot,sizes = cube_rules(xyxy,label,h_real,w_real,d_real,use_pure_hight)
    # depth_c = depth_c+0.11
    corners_3D.append(corners_from_center(depth_c, shift_c, rot,sizes))
    if label_reaches_border(xyxy,use_pure_hight):
        boundry = True
        corners_3D_rot_0 = corners_from_center(depth, shift, 0,sizes)
        corners_3D.append(corners_3D_rot_0)
    return depth, shift, depth_c, shift_c, rot,sizes, boundry, corners_3D

def label_reaches_border(xyxy,use_pure_hight):
    if use_pure_hight:
        min_y = -1
    else:
        min_y = 5
    # check if label far enough at the edge to make a partial detection likely
    if max(xyxy[0],xyxy[2])>795 or min(xyxy[0],xyxy[2])<5:
        return True
    if min(xyxy[1],xyxy[3])>595 or min(xyxy[1],xyxy[3])<min_y:
        return True
    return False

def convert_2d_3d(xyxy, im0, label):
    """Converting a 2d object detection to a 3d bounding box, this is done based on know information about the sizes of the
    objects and the intrisic and extrisic matrix of the camera. The size of each object is stored in this function and the correct
    one matched to the passed label before requesting the calculations to convert to 3d bb
    @param xyxy: the bounding box provided by yolo
    @param im0: the image the detection happened on
    @param label: the type of object that was detected as well as the confidence
    @return corners_3D: all 8 corners of the 3d bounding box
    @return boundry: if the object hits the corners of the image and is therefore uncertain
    @return (depth, shift, depth_c, shift_c, rot): detected information about distances, unused at the moment
    """
    """movable objects  are ['sklappbox_c','sklappbox_a','box_c','box_a','chair','klappbox_c','klappbox_a','sbox_c','sbox_a']
    workstations can be ['workstation_c', 'workstation_a]'"""
    movable_names = ['sklappbox_c','sklappbox_a','box_c','box_a','chair','klappbox_c','klappbox_a','sbox_c','sbox_a','hocker_c','hocker_a']
    boundry = False
    corners_3D = []
    label = label[0:-5]
    if label in ['workstation_c','workstation_a']:
        h_real = 0.95
        w_real = 1.15
        d_real = 0.80
        depth, shift, depth_c, shift_c, rot,sizes, boundry, corners_3D = apply_for_cube(xyxy,label,corners_3D,boundry,h_real,w_real,d_real)
    elif(label in movable_names):
        if label in ['sklappbox_c','sklappbox_a','klappbox_c','klappbox_a']:
            h_real = 0.465
            w_real = 0.48
            d_real = 0.35
            if label[0]=='s':
                w_real_ = w_real
                w_real = d_real
                d_real = w_real_
            depth, shift, depth_c, shift_c, rot,sizes, boundry, corners_3D = apply_for_cube(xyxy,label,corners_3D,boundry,h_real,w_real,d_real)

        if label in ['sbox_c','sbox_a','box_c','box_a']:
            h_real = 0.482
            w_real = 0.353
            d_real = 0.238    
            if label[0]=='s':
                w_real_ = w_real
                w_real = d_real
                d_real = w_real_
            depth, shift, depth_c, shift_c, rot,sizes, boundry, corners_3D = apply_for_cube(xyxy,label,corners_3D,boundry,h_real,w_real,d_real)
        
        if label in ['hocker_c','hocker_a']:
            h_real = 0.51
            w_real = 0.32
            d_real = 0.32    
            depth, shift, depth_c, shift_c, rot,sizes, boundry, corners_3D = apply_for_cube(xyxy,label,corners_3D,boundry,h_real,w_real,d_real)

        if label == 'chair':
                h_real = 1.04
                w_real = 0.7
                d_real = 0.7
                # TODO expand to cirle version
                depth, shift, depth_c, shift_c, rot,sizes, boundry, corners_3D = apply_for_cube(xyxy,label,corners_3D,boundry,h_real,w_real,d_real)
    else:
        raise ValueError('unknown label')

    return corners_3D, boundry, (depth, shift, depth_c, shift_c, rot)


# TODO the sides need to not take the outer for the startposition neeed to use the side that is in the picture instead
# TODO the sides need to be done in a way that actually sets the image side when using 0