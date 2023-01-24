#!/usr/bin/env python3
import json
import rospy
import cv2
import os
import numpy as np
from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode

from utils.tf_laserlink2map import laser2mapConv
from utils.getDistAngleLidar import getDistAngle
from utils.constants import Topics, Nodes
from utils.conversions import polar2Cartesion
from utils.ros_logger import set_rospy_log_lvl

# Global datcollection
target_identified = {}

publisher_target = rospy.Publisher(Topics.TARGET.value, PoseStamped, queue_size=10)


def createMarkers(topic: str, markerType: str = "NavPosed"):
    global target_identified
    markerArray = MarkerArray()
    publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)

    for key in target_identified.keys():
        marker = Marker()
        marker.ns = key
        marker.header.frame_id = target_identified[key][markerType]["posedstamped"][
            "header"
        ]["frame_id"]
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        if "navposed" in markerType.lower():
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
        else:
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        marker.pose.position.x = target_identified[key][markerType]["posedstamped"][
            "pose"
        ]["position"]["x"]
        marker.pose.position.y = target_identified[key][markerType]["posedstamped"][
            "pose"
        ]["position"]["y"]
        marker.pose.position.z = target_identified[key][markerType]["posedstamped"][
            "pose"
        ]["position"]["z"]
        marker.pose.orientation.x = target_identified[key][markerType]["posedstamped"][
            "pose"
        ]["orientation"]["x"]
        marker.pose.orientation.y = target_identified[key][markerType]["posedstamped"][
            "pose"
        ]["orientation"]["y"]
        marker.pose.orientation.z = target_identified[key][markerType]["posedstamped"][
            "pose"
        ]["orientation"]["z"]
        marker.pose.orientation.w = target_identified[key][markerType]["posedstamped"][
            "pose"
        ]["orientation"]["w"]
        markerArray.markers.append(marker)

        target_id = 0
        for marker in markerArray.markers:
            marker.id = target_id
            target_id += 1

        try:
            publisher.publish(markerArray)
        except:
            return


def fetchCoordinate(wsID: Int16):
    """
    Converts a ID from a workstation to a coordinate where the Robotino can safely navigate to

    Args:
        wsID (Int16): Id of the target. Is given by the subscriber who calls this function

    Returns:
        Coordinate is published to topic "/target"
    """
    global target_identified
    targetId = wsID.data
    rospy.logdebug(f"[Workstation Mapper] Fetching coordinates")
    try:
        # Create ROS message
        targetCor = PoseStamped()
        targetCor.pose.position.x = target_identified[f"Workstation{targetId}"][
            "NavPosed"
        ]["posedstamped"]["pose"]["position"]["x"]
        targetCor.pose.position.y = target_identified[f"Workstation{targetId}"][
            "NavPosed"
        ]["posedstamped"]["pose"]["position"]["y"]
        targetCor.pose.position.z = target_identified[f"Workstation{targetId}"][
            "NavPosed"
        ]["posedstamped"]["pose"]["position"]["z"]
        targetCor.pose.orientation.x = target_identified[f"Workstation{targetId}"][
            "NavPosed"
        ]["posedstamped"]["pose"]["orientation"]["x"]
        targetCor.pose.orientation.y = target_identified[f"Workstation{targetId}"][
            "NavPosed"
        ]["posedstamped"]["pose"]["orientation"]["y"]
        targetCor.pose.orientation.z = target_identified[f"Workstation{targetId}"][
            "NavPosed"
        ]["posedstamped"]["pose"]["orientation"]["z"]
        targetCor.pose.orientation.w = target_identified[f"Workstation{targetId}"][
            "NavPosed"
        ]["posedstamped"]["pose"]["orientation"]["w"]
    except:
        rospy.logerr(f"Error, cant publish target to topic {Topics.TARGET.value}")
        return
    # Publish coordinate
    try:
        publisher_target.publish(targetCor)
        rospy.logdebug(
            f"Published target ({targetCor.pose.position.x},{targetCor.pose.position.y}) to topic {Topics.TARGET.value}"
        )
    except:
        rospy.logerr(f"Error, cant publish target to topic {Topics.TARGET.value}")


def qrCodeScanner(rawImage: Image):
    """
    Scans a QR Code and determines the id of the workstation and it's target. Taken from Vaibav Tiwaris work

    Args:
        rawImage (Image): Image from with the QR Code is scanned. Image comes from camera of Robotinio.\
                                        Is given by the subscriber who calls this function
    
    Returns:
        Saves identified workstations to global variable  "target_identified"
    """
    global target_identified
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
        rospy.logdebug("[Workstation Mapper] Detecting QR Code")
        for barcode in decode(img_resized):
            center_x_value = 0

            for iter in range(1, 251):
                # Get data from barcode
                myData = barcode.data.decode("utf-8")
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
            objectDist, navObjectDist, navObjectAngle = getDistAngle(camera_angle)

            if objectDist < 2.0 and objectDist != 0.0:
                # Find the x and y axis of the target wrt to laser frame
                object_x, object_y = polar2Cartesion(objectDist, camera_angle)
                navObject_x, navObject_y = polar2Cartesion(
                    navObjectDist, navObjectAngle
                )

                if myData in target_identified.keys():
                    rospy.logdebug(
                        f"[Workstation Mapper] {myData} is already identified"
                    )
                else:
                    posedstamped = PoseStamped()
                    navPosedstamped = PoseStamped()
                    posedstamped = laser2mapConv(
                        object_x, object_y, 0.0, roll=camera_angle, pitch=0.0, yaw=0.0
                    )
                    navPosedstamped = laser2mapConv(
                        navObject_x,
                        navObject_y,
                        0.0,
                        roll=camera_angle,
                        pitch=0.0,
                        yaw=0.0,
                    )

                    target_identified[myData] = {
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
                    rospy.loginfo(
                        f"[Workstation Mapper] Identified workstation {myData} and saved marker"
                    )
            else:
                rospy.logdebug(
                    f"[Workstation Mapper] Target {myData} is not within range"
                )


def workstationMapper():
    """ 
    Runs the node itself and subscribes to all necessary topics. This is basically a adapter for the work\
    from Vaibav Tiwari which gets reused here.
    """
    global target_identified
    rospy.init_node(Nodes.WORKSTATION_MAPPER.value)

    set_rospy_log_lvl(rospy.DEBUG)

    rospy.loginfo(f"Starting node {Nodes.WORKSTATION_MAPPER.value}")
    # Load already identified workstations
    _loadMarkersFromJson()
    # Converts the id of a workstation to a coordinate which the path planner can use
    rospy.Subscriber(Topics.TARGET_ID.value, Int16, fetchCoordinate, queue_size=10)
    # -------------------------------- QR Code Scanning ------------------------------------
    # Only start QR code scanner if not all workstations are identified
    camera_sub = rospy.Subscriber(
        Topics.IMAGE_RAW.value, Image, qrCodeScanner, queue_size=2
    )

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        createMarkers(Topics.MARKERS_NAV.value)
        createMarkers(Topics.MARKERS.value, markerType="MarkerPosed")
        if len(target_identified.keys()) == 4:
            # unregister subscriber which is responsible for QR code scanning
            camera_sub.unregister()
        rate.sleep()

    _saveMarkersToJson()


def _saveMarkersToJson():
    global target_identified
    file = rospy.get_param("~json_path")
    with open(file, "w") as jsonfile:
        json.dump(target_identified, jsonfile, sort_keys=True, indent=4)

    rospy.logdebug(f"[Workstation Mapper] Saved workstation mapping to {file}")


def _loadMarkersFromJson():
    global target_identified
    file = rospy.get_param("~json_path")
    if os.path.isfile(file):
        with open(file) as jsonfile:
            target_identified = json.load(jsonfile)

    rospy.logdebug(f"[Workstation Mapper] Loaded workstation mapping from {file}")


if __name__ == "__main__":
    try:
        workstationMapper()
    except:
        # ROS is shutdown ==> Save markers
        _saveMarkersToJson()
        rospy.loginfo(f"Shutdown node {Nodes.WORKSTATION_MAPPER.value}")
