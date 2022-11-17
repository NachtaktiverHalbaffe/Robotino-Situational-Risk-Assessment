#!/usr/bin/env python3
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from pyzbar.pyzbar import decode

def callback(data):
 
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
  #print("in callback")
 
  # Output debugging information to the terminal
  #rospy.loginfo("receiving video frame")
   
  # Convert ROS Image message to OpenCV image
  current_frame = br.imgmsg_to_cv2(data)
  for barcode in decode(current_frame):
    mydata = barcode.data.decode('utf-8')
    print(barcode.data)
   
  # Display image
  #cv2.imshow("camera", current_frame)
   
  #cv2.waitKey(1)
      
def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  print("in receive_message")
  rospy.init_node('video_sub_py', anonymous=True)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('/image_raw', Image, callback)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()
