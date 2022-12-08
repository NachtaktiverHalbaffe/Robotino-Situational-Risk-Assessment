
import numpy as np
import rospy
import matplotlib.pylab as plt
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__))+'/maps')
from map_config_pixel_to_acml import config_FinalGridMapv2, config_FinalScannerMap1

real_data =[]
image_counter = 0
Pose_real_data = [0,0,0,0]

def convert_robo_to_grid(x_r,y_r):
    "applies the roation of -69° to the robo cords to get the crods in the grind we measure on"
    global rot_glob
    x_g = x_r*np.cos(-rot_glob)-y_r*np.sin(-rot_glob)
    y_g = x_r*np.sin(-rot_glob)+y_r*np.cos(-rot_glob)
    return x_g,y_g

def convert_grid_to_robo(x_r,y_r):
    "applies the roation of -69° to the robo cords to get the crods in the grind we measure on"
    global rot_glob
    x_g = x_r*np.cos(rot_glob)-y_r*np.sin(rot_glob)
    y_g = x_r*np.sin(rot_glob)+y_r*np.cos(rot_glob)
    return x_g,y_g

global normal
global normal2
global changed
global changed2
normal = {}
normal2 = +100
changed = {}
changed2 = +100


def get_pixel_location_from_acml(x_a,y_a,x3,y3,x5_3,x5_3_p,y5_3,y5_3_p,x3_p,y3_p,rot_unused,rot=None):
    "get pixel location from acml pos"
    global normal
    global normal2
    global changed
    global changed2
    if not rot == None:
        rot = rot-0.55
        if rot>np.pi:
            rot = rot-np.pi*2
        normal.update({rot:y_a})
        lists = sorted(normal.items())
        x,y = zip(*lists)
        plt.clf()
        plt.plot(x,y)
        #plt.plot([1, 2, 3, 4,5,2,6,3])
        #plt.close()
        normal2 = min(x_a,normal2)
        # print(normal,normal2,'normal')
        x_a = x_a-np.cos(rot)*0.17
        y_a = y_a-np.sin(rot)*0.17
        changed.update({rot:y_a})
        lists = sorted(changed.items())
        x,y = zip(*lists)
        plt.plot(x,y)
        changed2 = min(x_a,changed2)
        # print(changed,changed2,'changed')
        
        plt.ylabel('some numbers')
        # plt.show(block=False)
        # plt.pause(0.2)
        # print(min(normal,key=normal.get))
        # print(min(changed,key=changed.get))

        # print(x_a,y_a,'changed_xy')
        # print(np.cos(rot)*0.235,np.sin(rot)*0.235,rot,'change')
    x_convert = x5_3_p/x5_3 #pixel per meter in width, pixels have flipped x axis (is also offset, applied below)
    y_convert = y5_3_p/y5_3 #pixel per meter in hight, pixels have flipped y axis (is also offset, applied below)
    offset_x = -x3*x_convert+x3_p # calculating x offset based on the pixel location of x3
    offset_y = y3*x_convert+y3_p # calculating y offset based on the pixel location of y3
    x_p = x_a*x_convert+offset_x # calulating pixel value of x
    y_p = y_a*y_convert+offset_y # calulating pixel value of y
    return x_p, y_p

def get_amcl_from_pixel_location(x_p,y_p,x3,y3,x5_3,x5_3_p,y5_3,y5_3_p,x3_p,y3_p,rot_unused,rot=None):
    "get pixel location from acml pos"
    x_convert = x5_3_p/x5_3 #pixel per meter in width, pixels have flipped x axis (is also offset, applied below)
    y_convert = y5_3_p/y5_3 #pixel per meter in hight, pixels have flipped y axis (is also offset, applied below)
    offset_x = -x3*x_convert+x3_p # calculating x offset based on the pixel location of x3
    offset_y = y3*x_convert+y3_p # calculating y offset based on the pixel location of y3
    #x_p = x_a*x_convert+offset_x # calulating pixel value of x
    #y_p = y_a*y_convert+offset_y # calulating pixel value of y
    x_a = (x_p - offset_x)/ x_convert
    y_a = (y_p- offset_y)/y_convert
    if not rot == None:
        rot = 2*np.arcsin(rot)
        x_a = x_a-np.cos(rot)*0.17
        y_a = y_a-np.sin(rot)*0.17
    return x_a, y_a

def convert_robo_to_cam(x_r,y_r, rot):
    "applies the roation of robo to the acml cords get the x,y in camera cords"
    rot = -rot
    x_g = x_r*np.cos(rot)-y_r*np.sin(rot)
    y_g = x_r*np.sin(rot)+y_r*np.cos(rot)
    return x_g,y_g

def convert_cam_to_robo(x_r,y_r, rot):
    "applies the roation of robo to the camera cords get the x,y in acml"
    x_g = x_r*np.cos(rot)-y_r*np.sin(rot)
    y_g = x_r*np.sin(rot)+y_r*np.cos(rot)
    return x_g,y_g

def get_base_info():

    config_Map = config_FinalGridMapv2
    # config_Map = config_FinalScannerMap1
    #Workstation_5
    #position: 
    x5= config_Map['x5_a']
    y5= config_Map['y5_a']
    #orientation: 
    z2= 0.9865593763339435
    w1= 0.16340317306460247
    x5_p = config_Map['x5_p']
    y5_p = config_Map['y5_p']
    #workstation_3
    #position: 
    x3= config_Map['x3_a']
    y3= config_Map['y3_a']
    #orientation: 
    z1= -0.8168027744144787
    w1= 0.5769170024438613
    x3_p = config_Map['x3_p']
    y3_p = config_Map['y3_p']

    # distance to check center point
    y5_3 = y5-y3
    x5_3 = x5-x3
    y5_3_p = y5_p-y3_p
    x5_3_p = x5_p-x3_p
    global rot_glob
    rot_glob = config_Map['rot']

    return (x3,y3,x5_3,x5_3_p,y5_3,y5_3_p,x3_p,y3_p,config_Map['rot']),config_Map


if __name__ == '__main__':
    print('This is a utils collections on the topic of changing coords between camera,acml and pixel locations.')
    print('The base info part needs to be redone if you want to use it with a new map')