#!/usr/bin/env python3

#https://stackoverflow.com/questions/57518178/how-to-detect-main-structure-contour-by-detecting-straight-lines

import os
import rospkg
import yaml
import cv2
import numpy as np
import math

rospack = rospkg.RosPack()

map_path = os.path.join(rospack.get_path("digital_twin"),'map','cropped15.pgm')
map_yaml_path = os.path.join(rospack.get_path("digital_twin"),'map','cropped15.yaml')
data_path = os.path.join(rospack.get_path("digital_twin"),'param','static_env.yaml')

with open(map_yaml_path, "r") as yamlfile:
    data = yaml.load(yamlfile, Loader=yaml.FullLoader)
    print("Read successful")
print(data)
print(data['resolution'])

img = cv2.imread(map_path)
gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
gray=255-gray
gray=cv2.threshold(gray,4,255,cv2.THRESH_BINARY)[1]
gray=cv2.blur(gray,(15,1))
contours,hierarchy = cv2.findContours(gray,cv2.RETR_LIST ,cv2.CHAIN_APPROX_NONE )

for cnt in contours:
    area = cv2.contourArea(cnt)
    
    if area>40000 and area< 300000:
        print(area)
        #cv2.drawContours(img,[cnt],0,(255,0,0),2)
        rect = cv2.minAreaRect(cnt)
        print(rect[0]) # ( center (x,y), (width, height), angle of rotation ).

        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(img,[box],0,(0,0,255))
        center= rect[0]
        angle1= rect[2]
        l = box #edges 
        ''' 
        four walls
        edges given , w and h , center(x,y)
        e1 to e2 the 
        dist = math.hypot(x2 - x1, y2 - y1)
        '''

        walls = {}
        for i in range(0, len(l)):
            e1 =l[i]
            e2 = l[(i+1) % len(l)]
            x1 =e1[0]
            x2 =e2[0]
            y1 =e1[1]
            y2 =e2[1]
            dist = round(math.hypot(x2 - x1, y2 - y1))
            center = (np.mean([x1, x2]),np.mean([y1, y2]))
            angle = round((math.atan2(x2 - x1, y2 - y1)),2)
            centerX = round(np.mean([x1, x2]))
            centerY = round(np.mean([y1, y2]))
            wall_info =  { 'wall_'+str(i): { 'center' : {'x':centerX, 'y': centerY}, 'length': dist, 'angle' : angle } }
            #print(wall_info, x1,x2,y1,y2)
            walls.update(wall_info)
        #print(walls)

        import os.path
        mode = 'w' if os.path.isfile(data_path) else 'a'
        # append mode creates file too!!!
        with open(data_path, mode) as yamlfile:
            data = yaml.dump(walls, yamlfile)
            print("Write successful")

#cv2.imshow('walls',img)    
#cv2.waitKey()


# done!!!
