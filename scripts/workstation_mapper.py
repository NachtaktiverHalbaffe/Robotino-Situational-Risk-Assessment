#!/usr/bin/env python3
import json
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int16
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
from tf_laserlink2map import laser2mapConv

from constants import Topics, Nodes

# from real_nav_control.getDistAngleLidar import getDistAngle
# from real_nav_control.conversions import polar2Cartesion

# Global variable to read camera data
Image_data = []
# Global counter to save image
image_counter = 0
# Global datcollection
target_identified = {}

# Constants
JSON_PATH = "maps/markers.json"


def fetchCoordinate(wsID: Int16):
    """
    Converts a ID from a workstation to a coordinate where the Robotino can safely navigate to

    Args:
        wsID (Int16): Id of the target. Is given by the subscriber who calls this function

    Returns:
        Coordinate is published to topic "/target"
    """
    targetId = int(wsID)
    # TODO Logic to find coordinate

    # Create ROS message
    targetCor = Point()

    # Publish coordinate
    publisher = rospy.Publisher(Point, Topics.TARGET.value)
    try:
        publisher.publish(targetCor)
    except:
        print(f"Error, cant publish target to topic {Topics.TARGET.value}")


def qrCodeScanner(rawImage: Image):
    """
    Scans a QR Code and determines the id of the workstation and it's target. Taken from Vaibav Tiwaris work

    Args:
        rawImage (Image): Image from with the QR Code is scanned. Image comes from camera of Robotinio.\
                                        Is given by the subscriber who calls this function
    
    Returns:
        Saves identified workstations to global variable  "target_identified"
    """
    global image_counter
    global target_identified
    image_counter += 1

    br = CvBridge()
    img = br.imgmsg_to_cv2(rawImage, "rgb8")

    # Calculating image dimensions
    width = int(rawImage.width * 0.80)
    height = int(rawImage.height * 0.80)
    dim = (width, height)

    # Resize image
    img_resized = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

    if decode(img_resized):
        # ------------------ Detecting QR Code --------------------------
        rospy.logdebug("Detecting QR Code")
        for barcode in decode(img_resized):
            center_x_value = 0

            for iter in range(1, 251):
                # Get data from barcode
                mydata = barcode.data.decode("utf-8")
                # Calculate center of QR-Code
                pts = np.array([barcode.polygon], np.int32)
                pts = pts.reshape(-1, 1, 2)
                cv2.polylines(img_resized, [pts], True, (255, 0, 255), 5)
                pts2 = barcode.rect

                # Calculate the center of the QR code
                center_x = pts2[0] + (pts2[2] / 2)
                center_x_value = center_x + center_x_value

        # Calculate the angle of the target wrt to camera
        camera_angle = 36 - (0.1125 * (center_x_value / 250))

        # Calculate the distance of the angle wrt the given camera angle
        # TODO import image
        objectDist, navObjectDist, navObjectAngle = getDistAngle(camera_angle)

        if objectDist < 2.0 and objectDist != 0.0:
            # Find the x and y axis of the target wrt to laser frame
            # TODO import from real_nav
            object_x, object_y = polar2Cartesion(objectDist, camera_angle)
            navObject_x, navObject_y = polar2Cartesion(navObjectDist, navObjectAngle)

            if mydata in target_identified.keys():
                rospy.logdebug(f"{mydata} is already identified")
            else:
                posedstamped = PoseStamped()
                navPosedstamped = PoseStamped()
                posedstamped = laser2mapConv(
                    object_x, object_y, 0.0, roll=camera_angle, pitch=0.0, yaw=0.0
                )
                navPosedstamped = laser2mapConv(
                    navObject_x, navObject_y, 0.0, roll=camera_angle, pitch=0.0, yaw=0.0
                )

                target_identified[mydata] = {
                    "MarkerPosed": {
                        "posedstamped": {
                            "header": {
                                "seq": posedstamped.header.seq,
                                "stamp": str(posedstamped.header.stamp),
                                "frame_id": posedstamped.header.frame_id,
                            },
                            "pose": {
                                "position": {
                                    "x": posedstamped.pose.position.x,
                                    "y": posedstamped.pose.position.y,
                                    "z": posedstamped.pose.position.z,
                                },
                                "orientation": {
                                    "x": posedstamped.pose.orientation.x,
                                    "y": posedstamped.pose.orientation.y,
                                    "z": posedstamped.pose.orientation.z,
                                    "w": posedstamped.pose.orientation.w,
                                },
                            },
                        }
                    },
                    "NavPosed": {
                        "posedstamped": {
                            "header": {
                                "seq": navPosedstamped.header.seq,
                                "stamp": str(navPosedstamped.header.stamp),
                                "frame_id": navPosedstamped.header.frame_id,
                            },
                            "pose": {
                                "position": {
                                    "x": navPosedstamped.pose.position.x,
                                    "y": navPosedstamped.pose.position.y,
                                    "z": navPosedstamped.pose.position.z,
                                },
                                "orientation": {
                                    "x": navPosedstamped.pose.orientation.x,
                                    "y": navPosedstamped.pose.orientation.y,
                                    "z": navPosedstamped.pose.orientation.z,
                                    "w": navPosedstamped.pose.orientation.w,
                                },
                            },
                        }
                    },
                }
        else:
            rospy.logdebug(f"Target {mydata} is not whithin range")

    #     cv2.putText(
    #         img_resized,
    #         mydata,
    #         (pts2[0], pts2[1]),
    #         cv2.FONT_HERSHEY_SIMPLEX,
    #         0.9,
    #         (255, 0, 255),
    #         2,
    #     )

    # cv2.imshow("Camera", img_resized)
    # cv2.waitKey(1)


def targetLocation():
    """
    Creates a marker which represents the target coordinate to which the Robotino\
    can safely navigate when a workstation as a target is given

    Args:
        Reads the identified workstation from the file  "maps/markers.json"
    
    Returns:
        Saves the coordinates of the identified workstation to XXXX
    """
    file = JSON_PATH
    jsonFile = None
    with open(file) as jsonfile:
        jsonFile = json.loads(jsonfile)


def workstationMapper():
    """ 
    Runs the node itself and subscribes to all necessary topics. This is basically a adapter for the work\
    from Vaibav Tiwari which gets reused here.
    """
    rospy.init_node(Nodes.WORKSTATION_MAPPER.value)
    # Converts the id of a workstation to a coordinate which the path planner can use
    rospy.Subscriber(Topics.TARGET_ID.value, Int16, fetchCoordinate)

    # -------------------------------- QR Code Scanning ------------------------------------
    # Load already identified workstations
    file = JSON_PATH
    jsonFile = None
    with open(file) as jsonfile:
        jsonFile = json.loads(jsonfile)
    # Only start QR code scanner if not all workstations are identified
    if len(jsonFile.keys()) != 4:
        # Scans QR Code and identifies workstations
        camera_sub = rospy.Subscriber(Topics.IMAGE_RAW.value, Image, qrCodeScanner)

        rate = rospy.Rate(500)
        while not rospy.is_shutdown():
            # Stop QR scanning if all workstations are identified
            if len(target_identified.keys()) == 4:
                # unregister subscriber which is responsible for QR code scanning
                camera_sub.unregister()
                shutdownProcess()
            rate.sleep()

    # Prevents python from exiting until this node is stopped
    rospy.spin()


def shutdownProcess():
    global target_identified
    file = JSON_PATH
    with open(file, "w") as jsonfile:
        json.dump(target_identified, jsonfile, sort_keys=True, indent=4)

    rospy.logdebug(f"Saved workstation mapping to {file}")


if __name__ == "__main__":
    workstationMapper()
