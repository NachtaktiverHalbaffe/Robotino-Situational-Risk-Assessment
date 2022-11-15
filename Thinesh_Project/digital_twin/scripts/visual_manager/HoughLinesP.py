#!/usr/bin/env python3

from __future__ import print_function

#https://stackoverflow.com/questions/57518178/how-to-detect-main-structure-contour-by-detecting-straight-lines

import os
import rospkg
import yaml
import cv2
import numpy as np
import math

rospack = rospkg.RosPack()

map_path = os.path.join(rospack.get_path("digital_twin"),'map','31may.pgm')
map_yaml_path = os.path.join(rospack.get_path("digital_twin"),'map','cropped15.yaml')
data_path = os.path.join(rospack.get_path("digital_twin"),'param','static_env.yaml')



import os


import cv2
import numpy as np

'''
img = cv2.imread(path)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray,50,150,apertureSize = 3)
minLineLength = 1
maxLineGap = 10
lines = cv2.HoughLinesP(edges,1,np.pi/180,100,minLineLength,maxLineGap)
for x1,y1,x2,y2 in lines[0]:
    cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)

cv2.imshow('houghlines5.jpg',img)
'''

''' image black to white 
image = cv2.imread(path)

original = image.copy()
image = cv2.bitwise_not(image) # inverting white and black
gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

# https://newbedev.com/remove-non-straight-lines-from-text-image
# slanted line kernel
kernel = np.array([[0, 0, 1],
                   [0, 1, 0],
                   [1, 0, 0]], dtype=np.uint8)

# morphological transformations to smooth the image
#kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

kernel = np.array([[0, 0, 1],
                   [0, 1, 0],
                   [1, 0, 0]], dtype=np.uint8)
# Find horizontal lines
horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (35,2))
detect_horizontal = cv2.morphologyEx(close, cv2.MORPH_OPEN, horizontal_kernel, iterations=2)
cnts = cv2.findContours(detect_horizontal, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = cnts[0] if len(cnts) == 2 else cnts[1]
for c in cnts:
    cv2.drawContours(original, [c], -1, (36,255,12), -1)

# Find vertical lines
vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2,35))
detect_vertical = cv2.morphologyEx(close, cv2.MORPH_OPEN, vertical_kernel, iterations=2)
cnts = cv2.findContours(detect_vertical, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = cnts[0] if len(cnts) == 2 else cnts[1]
for c in cnts:
    cv2.drawContours(original, [c], -1, (36,255,12), -1)
'''



import cv2
import numpy as np

image=cv2.imread(map_path)
cv2.imshow('input image',image)

orig_image = image.copy()

gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
ret, thresh=cv2.threshold(gray,127,255,cv2.THRESH_BINARY_INV)

contours, hierarchy=cv2.findContours(thresh.copy(),cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)

areas = []

for c in contours:
    area = cv2.contourArea(c)
    areas.append(area)

max_area = max(areas)
rects=[]
for c in contours:
    area = cv2.contourArea(c)
    x,y,w,h=cv2.boundingRect(c)
    cv2.rectangle(orig_image,(x,y),(x+w,y+h),(0,0,255),2)
    #calculate accuracy as a percent of contour perimeter
    accuracy=0.01*cv2.arcLength(c,True)
    approx=cv2.approxPolyDP(c,accuracy,True)
    if area == max_area:
        pol = c
        cv2.drawContours(image,[approx],0,(0,255,0),2)
    rects.append()

cv2.imshow('Approx polyDP', image)

def non_max_suppression_fast(boxes, overlapThresh):
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