#!/usr/bin/env python3

import numpy as np
import cv2
from skimage.metrics import structural_similarity as ssim

from scipy.cluster.hierarchy import linkage, fcluster
from scipy.spatial import distance as ssd

#https://gist.github.com/anilsathyan7/cab95759d9093d7c34cf1a460610c5d2
# Read input image and convert to grayscale

import os
import rospkg
import yaml
import cv2
import numpy as np
import math

rospack = rospkg.RosPack()

map_path = os.path.join(rospack.get_path("digital_twin"),'map','cropped15.pgm')
img = cv2.imread(map_path)
print("Image shape: " + str(img.shape)) 
imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


# Apply adaptive threshold
cv2.imwrite('gray.png',imgray)
thresh = cv2.adaptiveThreshold(imgray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,11,2)
cv2.imwrite('edge.png',thresh)


# Find all contours of binary image
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Number of contours
num_contours=len(contours)
print("Total number of contours: " + str(num_contours))

# Fill up the shapes
filled=cv2.fillPoly(thresh, pts=contours, color=(255,255,255))

# Crop and save proper contours separately [filter by area]
colour_crop=[]
num_contours=0
min_area=50
for cnt in contours:
  if(cv2.contourArea(cnt)>min_area):
    x,y,w,h = cv2.boundingRect(cnt)
    colour_crop.append(img[y:y+h, x:x+w])
    cv2.imwrite("contour_binary"+str(num_contours)+".png",filled[y:y+h, x:x+w])
    num_contours=num_contours+1

# Number of proper contours
print("Number of proper contours: " + str(num_contours))

# Initialize similarity matrix
ssim_mat=np.zeros((num_contours, num_contours))

# Configure cluster settings
cluster=np.zeros(num_contours, dtype=int) 
num_classes=9
colours=[(255,0,0),(0,255,0),(0,0,255),(0,0,0),(255,255,0), (0,255,255),(255,0,255),(128,128,128),(165,42,42)]


# Compute similarity matrix using ssim 
print("Similarity Matrix - SSIM")
for i in range(num_contours):
   for j in range(num_contours):
      im1=np.array(cv2.cvtColor(colour_crop[i], cv2.COLOR_BGR2GRAY))
      im2=np.array(cv2.cvtColor(colour_crop[j], cv2.COLOR_BGR2GRAY))

      im1=cv2.resize(im1,((im1.shape[1]+im2.shape[1])//2, (im1.shape[0]+im2.shape[0])//2))
      im2=cv2.resize(im2, (im1.shape[1], im1.shape[0]) )

      im1=cv2.equalizeHist(im1)
      im2=cv2.equalizeHist(im2)

      ssim_mat[i][j],_ = ssim(im1, im2, full=True)
     
      print(str(round(ssim_mat[i][j],4))+",",end =" ")
   print("\n")


'''
 Cluster the contours using similarity matrix and max cluster no.
 Converts similarity matrix to  distance matrix [Range: 0 - 1]
 Returns cluster vector  [Range: 0 - (num_classes-1)]
'''
Zd = linkage(ssd.squareform(1-ssim_mat), method="complete") 
cluster = fcluster(Zd, num_classes, criterion='maxclust') - 1  
print("Cluster:-\n" + str(cluster))


# Draw the contour clusters
c=0
for cnt in contours:
  if(cv2.contourArea(cnt)>min_area):
    #x,y,w,h = cv2.boundingRect(cnt)
    (x, y), (w, h), angle = cv2.minAreaRect(cnt)
    rect = cv2.minAreaRect(c)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
           
            
    cv2.drawContours(img,[box],0,(0,0,255))

    print(x,y)
    #cv2.imwrite("contour_coloured"+str(c)+".png",img[y:y+h, x:x+w])
    #cv2.rectangle(img,(x,y),(x+w,y+h),colours[cluster[c]],2)
    c=c+1 

# Show output in window
cv2.imshow("Bounding Boxes", img)
cv2.waitKey(0)
cv2.destroyAllWindows()