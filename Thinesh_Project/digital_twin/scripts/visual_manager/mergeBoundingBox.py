#!/usr/bin/env python3
from __future__ import print_function
import cv2 as cv
import numpy as np
import argparse
import random as rng
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt
#!/usr/bin/env python3

import math
import numpy as np
import os
import rospkg
import numpy as np
import yaml

rospack = rospkg.RosPack()

map_path = os.path.join(rospack.get_path("digital_twin"),'map','cropped15.pgm')
yaml_path = os.path.join(rospack.get_path("digital_twin"),'param','obstacle.yaml')
rng.seed(12345)






def non_max_suppression_fast(boxes, overlapThresh =0):
   # if there are no boxes, return an empty list
   if len(boxes) == 0:
      return []

   # if the bounding boxes integers, convert them to floats --
   # this is important since we'll be doing a bunch of divisions
   if boxes.dtype.kind == "i":
      boxes = boxes.astype("float")
#  
   # initialize the list of picked indexes   
   pick = []

   # grab the coordinates of the bounding boxes
   x1 = boxes[:,0]
   y1 = boxes[:,1]
   x2 = boxes[:,2]
   y2 = boxes[:,3]

   # compute the area of the bounding boxes and sort the bounding
   # boxes by the bottom-right y-coordinate of the bounding box
   area = (x2 - x1 + 1) * (y2 - y1 + 1)
   idxs = np.argsort(y2)

   # keep looping while some indexes still remain in the indexes
   # list
   while len(idxs) > 0:
      # grab the last index in the indexes list and add the
      # index value to the list of picked indexes
      last = len(idxs) - 1
      i = idxs[last]
      pick.append(i)

      # find the largest (x, y) coordinates for the start of
      # the bounding box and the smallest (x, y) coordinates
      # for the end of the bounding box
      xx1 = np.maximum(x1[i], x1[idxs[:last]])
      yy1 = np.maximum(y1[i], y1[idxs[:last]])
      xx2 = np.minimum(x2[i], x2[idxs[:last]])
      yy2 = np.minimum(y2[i], y2[idxs[:last]])

      # compute the width and height of the bounding box
      w = np.maximum(0, xx2 - xx1 + 1)
      h = np.maximum(0, yy2 - yy1 + 1)

      # compute the ratio of overlap
      overlap = (w * h) / area[idxs[:last]]

      # delete all indexes from the index list that have
      idxs = np.delete(idxs, np.concatenate(([last],
         np.where(overlap > overlapThresh)[0])))

   # return only the bounding boxes that were picked using the
   # integer data type
   return boxes[pick].astype("int")













def thresh_callback(val):
    threshold = val
    
    canny_output = cv.Canny(src_gray, threshold, threshold * 2)

    contours, hierarchy = cv.findContours(canny_output, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    contours_poly = [None]*len(contours)
    boundRect = [None]*len(contours)
    centers = [None]*len(contours)
    radius = [None]*len(contours)
    listData = []

    concat = np.concatenate(contours)
    hulls = cv.convexHull(concat)

    boxes = []

    for i, c in enumerate(contours):
        print(cv.contourArea(c))
        contours_poly[i] = cv.approxPolyDP(c, 0, True)
        #boundRect[i] = cv.boundingRect(contours_poly[i])

        centers[i], radius[i] = cv.minEnclosingCircle(contours_poly[i])

        x,y,w,h = cv.boundingRect(c)

        if (w > 150 and h > 50) or (h >150 and w > 50) : 

            print('L shaped wall')
        else:
            
            rect = cv.minAreaRect(c)
            box = cv.boxPoints(rect)
            box = np.int0(box)
            listData.append(box)
            (x, y), (w, h), yaw = cv.minAreaRect(c)
            #print(x, y, w, h, yaw)
            cv.drawContours(src,[box],0,(0,0,255))

            center= rect[0]
            angle1= rect[2]
            #print(rect)
            l = box #edges 
            ''' 
            four walls
            edges given , w and h , center(x,y)
            e1 to e2 the 
            dist = math.hypot(x2 - x1, y2 - y1)
            '''

            wall_info =  { 'rect_'+str(i): { 'center' : {'x':x, 'y': y}, 'length': {'w':w,'h':h}, 'angle' : yaw } }
                #print(wall_info, x1,x2,y1,y2)
            #print(walls)

            # append mode creates file too!!!
            with open(yaml_path, 'a') as yamlfile:
                if (cv.contourArea(c) > 20):
                    data = yaml.dump(wall_info, yamlfile)
                    print("Write successful")

            boxes.append(box)

    bb =non_max_suppression_fast(boxes)
    cv.drawContours(src,[bb],0,(0,0,255))

    #concat = np.concatenate(contours)
    #hulls = cv.convexHull(concat)
    #cv.approxPolyDP(hulls)
    #cv.drawContours(src, hulls,-1,(255,255,0,1))

    #cv.imwrite('.pgm', drawing)
    cv.drawContours(src, contours,-1,(255,255,0,1))
    cv.imshow('countours', src)



#parser = argparse.ArgumentParser(description='Code for Creating Bounding boxes and circles for contours tutorial.')
#parser.add_argument('--input', help='Path to input image.', default='31may.pgm')
#args = parser.parse_args()

src = cv.imread(map_path)

if src is None:
    print('Could not open or find the image:',map_path)
    exit(0)
# Convert image to gray and blur it
src_gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)

#cv.imshow(source_window, src)

max_thresh = 255
thresh = 100 # initial threshold
thresh_callback(thresh)
cv.waitKey()


# https://stackoverflow.com/questions/19079619/efficient-way-to-combine-intersecting-bounding-rectangles
def combineBoundingBox(box1, box2):
    x = min(box1[0], box2[0])
    y = min(box1[1], box2[1])
    w = box2[0] + box2[2] - box1[0]
    h = max(box1[1] + box1[3], box2[1] + box2[3]) - y
    return (x, y, w, h)


'''

def filter_prediction(self, output, image):
        if len(output) < 2:
            return pd.DataFrame()
        else:
            df = pd.DataFrame(output)
            df = df.assign(
                    area=lambda x: df[0].apply(lambda x: cv2.contourArea(x)),
                    bounding=lambda x: df[0].apply(lambda x: cv2.boundingRect(x))
                    )
            df = df[df['area'] > MIN_AREA]
            df_filtered = pd.DataFrame(
                    df['bounding'].values.tolist(), columns=['x1', 'y1', 'w', 'h'])
            df_filtered = df_filtered.assign(
                    x1=lambda x: x['x1'].clip(0),
                    y1=lambda x: x['y1'].clip(0),
                    x2=lambda x: (x['x1'] + x['w']),
                    y2=lambda x: (x['y1'] + x['h']),
                    label=lambda x: x.index.astype(str),
                    class_name=lambda x: x.index.astype(str),
                    )
            return df_filtered 
            '''
