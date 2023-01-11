import cv2
import config as config
import numpy as np
configs = config.configs[0]

trajectory_vanilla = np.array([[133,  62], [139,  73], [147,  84], [163, 110], [154, 137]], np.int32)
trajectory_adversary_position_upper = np.array([[133,  62], [139+4,  73], [147+4,  84], [163+4, 110], [154+4, 137]], np.int32)
trajectory_adversary_position_lower = np.array([[133,  62], [139-4,  73], [147-4,  84], [163-4, 110], [154-4, 137]], np.int32)
map = cv2.imread(configs["geo_path"])

pts = trajectory_vanilla.reshape((-1, 1, 2))
pts_upper = trajectory_adversary_position_upper.reshape((-1, 1, 2))
pts_lower = trajectory_adversary_position_lower.reshape((-1, 1, 2))
 
isClosed = False
 
# color in BGR
color = (0, 0, 255)
color_upper = (0, 255, 0)
color_lower = (255, 0, 0)
 
# Line thickness of 2 px
thickness = 2
 
# Using cv2.polylines() method
# Draw a Blue polygon with
# thickness of 1 px
image = cv2.polylines(map, [pts],
                      isClosed, color, thickness)
image = cv2.polylines(image, [pts_upper],
                      isClosed, color_upper, thickness)
image = cv2.polylines(image, [pts_lower],
                      isClosed, color_lower, thickness)

image = cv2.resize(image,[600,600])
cv2.imshow("Map", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
