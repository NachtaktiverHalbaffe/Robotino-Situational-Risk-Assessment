#!/usr/bin/env python3


# import the necessary packages 
# https://www.pyimagesearch.com/2014/11/17/non-maximum-suppression-object-detection-python/

from __future__ import print_function
import cv2 as cv
import numpy as np
import argparse
import random as rng
import numpy as np

from collections import namedtuple 
rng.seed(12345)
'''
tImageZone = namedtuple('tImageZone', 'x y w h')

def merge_zone(z1, z2):
    if (z1.x == z2.x and z1.y == z2.y and z1.w == z2.w and z1.h == z2.h):
        return z1
    x = min(z1.x, z2.x)
    y = min(z1.y, z2.y)
    w = max(z1.x + z1.w, z2.x + z2.w) - x
    h = max(z1.y + z1.h, z2.y + z2.h) - y
    return tImageZone(x, y, w, h)

def is_zone_overlap(z1, z2):
    # If one rectangle is on left side of other
    if (z1.x > z2.x + z2.w or z1.x + z1.w < z2.x):
        return False
    # If one rectangle is above other
    if (z1.y > z2.y + z2.h or z1.y + z1.h < z2.y):
        return False
    return True


def combine_zones(zones):
    index = 0
    if zones is None: return zones
    while index < len(zones):
        no_Over_Lap = False
        while no_Over_Lap == False and len(zones) > 1 and index < len(zones):
            zone1 = zones[index]
            tmpZones = np.delete(zones, index, 0)
            tmpZones = [tImageZone(*a) for a in tmpZones]
            for i in range(0, len(tmpZones)):
                zone2 = tmpZones[i]
                if (is_zone_overlap(zone1, zone2)):
                    tmpZones[i] = merge_zone(zone1, zone2)
                    zones = tmpZones
                    no_Over_Lap = False
                    break
                no_Over_Lap = True
        index += 1
    return zones



import itertools

# my Rectangle = (x1, y1, x2, y2), a bit different from OP's x, y, w, h
def intersection(rectA, rectB): # check if rect A & B intersect
    a, b = rectA, rectB
    startX = max( min(a[0], a[2]), min(b[0], b[2]) )
    startY = max( min(a[1], a[3]), min(b[1], b[3]) )
    endX = min( max(a[0], a[2]), max(b[0], b[2]) )
    endY = min( max(a[1], a[3]), max(b[1], b[3]) )
    if startX < endX and startY < endY:
        return True
    else:
        return False

def combineRect(rectA, rectB): # create bounding box for rect A & B
    a, b = rectA, rectB
    startX = min( a[0], b[0] )
    startY = min( a[1], b[1] )
    endX = max( a[2], b[2] )
    endY = max( a[3], b[3] )
    return (startX, startY, endX, endY)

#https://stackoverflow.com/questions/46260892/finding-the-union-of-multiple-overlapping-rectangles-opencv-python
def checkIntersectAndCombine(rects):
    if rects is None:
        return None
    mainRects = rects
    noIntersect = False
    while noIntersect == False and len(mainRects) > 1:
        mainRects = list(set(mainRects))
        # get the unique list of rect, or the noIntersect will be 
        # always true if there are same rect in mainRects
        newRectsArray = []
        for rectA, rectB in itertools.combinations(mainRects, 2):
            newRect = []
            if intersection(rectA, rectB):
                newRect = combineRect(rectA, rectB)
                newRectsArray.append(newRect)
                noIntersect = False
                # delete the used rect from mainRects
                if rectA in mainRects:
                    mainRects.remove(rectA)
                if rectB in mainRects:
                    mainRects.remove(rectB)
        if len(newRectsArray) == 0:
            # if no newRect is created = no rect in mainRect intersect
            noIntersect = True
        else:
            # loop again the combined rect and those remaining rect in mainRects
            mainRects = mainRects + newRectsArray
    return mainRects

#  Felzenszwalb et al.
def non_max_suppression_slow(boxes, overlapThresh):
	# if there are no boxes, return an empty list
	if len(boxes) == 0:
		return []
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
		# grab the last index in the indexes list, add the index
		# value to the list of picked indexes, then initialize
		# the suppression list (i.e. indexes that will be deleted)
		# using the last index
		last = len(idxs) - 1
		i = idxs[last]
		pick.append(i)
		suppress = [last]

		# loop over all indexes in the indexes list
		for pos in xrange(0, last):
			# grab the current index
			j = idxs[pos]
			# find the largest (x, y) coordinates for the start of
			# the bounding box and the smallest (x, y) coordinates
			# for the end of the bounding box
			xx1 = max(x1[i], x1[j])
			yy1 = max(y1[i], y1[j])
			xx2 = min(x2[i], x2[j])
			yy2 = min(y2[i], y2[j])
			# compute the width and height of the bounding box
			w = max(0, xx2 - xx1 + 1)
			h = max(0, yy2 - yy1 + 1)
			# compute the ratio of overlap between the computed
			# bounding box and the bounding box in the area list
			overlap = float(w * h) / area[j]
			# if there is sufficient overlap, suppress the
			# current bounding box
			if overlap > overlapThresh:
				suppress.append(pos)
		# delete all indexes from the index list that are in the
		# suppression list
		idxs = np.delete(idxs, suppress)
	# return only the bounding boxes that were picked
	return boxes[pick]




def thresh_callback(val):
    threshold = val
    
    canny_output = cv.Canny(src_gray, threshold, threshold * 2)
    
    
    contours, hierarchy = cv.findContours(canny_output, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    contours_poly = [None]*len(contours)
    boundRect = [None]*len(contours)
    centers = [None]*len(contours)
    radius = [None]*len(contours)
    listData = []
    for i, c in enumerate(contours):
        contours_poly[i] = cv.approxPolyDP(c, 3, True)
        boundRect[i] = cv.boundingRect(contours_poly[i])
        centers[i], radius[i] = cv.minEnclosingCircle(contours_poly[i])
        x,y,w,h = cv.boundingRect(c)
        rect = cv.minAreaRect(c)
        box = cv.boxPoints(rect)
        box = np.int0(box)

        listData.append(box)
    
        cv.drawContours(src,[box],0,(0,0,255))
    for box in listData:
        combine_zones(box)



	# perform non-maximum suppression on the bounding boxes
	#pick = non_max_suppression_slow(boundingBoxes, 0.3)
	#print "[x] after applying non-maximum, %d bounding boxes" % (len(pick))
    
    with open('your_file.txt', 'w') as f:
        f.writelines(["%s\n" % item  for item in listData])
    #cv.imwrite('.pgm', drawing)
    cv.drawContours(src, contours,-1,(255,255,0,1))
    cv.imshow('countours', src)

parser = argparse.ArgumentParser(description='Code for Creating Bounding boxes and circles for contours tutorial.')
parser.add_argument('--input', help='Path to input image.', default='31may.pgm')
args = parser.parse_args()
src = cv.imread(cv.samples.findFile(args.input))
if src is None:
    print('Could not open or find the image:', args.input)
    exit(0)
# Convert image to gray and blur it
src_gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
src_gray = cv.blur(src_gray, (3,3))
source_window = 'Source'
cv.namedWindow(source_window)
cv.imshow(source_window, src)

max_thresh = 255
thresh = 100 # initial threshold
cv.createTrackbar('Canny thresh:', source_window, thresh, max_thresh, thresh_callback)
thresh_callback(thresh)
cv.waitKey()

'''
import math
import numpy as np
import os
import rospkg
import numpy as np
import yaml

rospack = rospkg.RosPack()



import cv2;
import numpy as np;
map_path = os.path.join(rospack.get_path("digital_twin"),'map','cropped15.pgm')

# Read image
im_in = cv2.imread("map_path", cv2.IMREAD_GRAYSCALE)

# Threshold.
# Set values equal to or above 220 to 0.
# Set values below 220 to 255.

th, im_th = cv2.threshold(im_in, 0, 255, cv2.THRESH_BINARY_INV)

# Copy the thresholded image.
im_floodfill = im_th.copy()

# Mask used to flood filling.
# Notice the size needs to be 2 pixels than the image.
h, w = im_th.shape[:2]
mask = np.zeros((h+2, w+2), np.uint8)

# Floodfill from point (0, 0)
cv2.floodFill(im_floodfill, mask, (0,0), 255)

# Invert floodfilled image
im_floodfill_inv = cv2.bitwise_not(im_floodfill)

# Combine the two images to get the foreground.
im_out = im_th | im_floodfill_inv

# Display images.
cv2.imshow("Thresholded Image", im_th)
cv2.imshow("Floodfilled Image", im_floodfill)
cv2.imshow("Inverted Floodfilled Image", im_floodfill_inv)
cv2.imshow("Foreground", im_out)
cv2.waitKey(0)
