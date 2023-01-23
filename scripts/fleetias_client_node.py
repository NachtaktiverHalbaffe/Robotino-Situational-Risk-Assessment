#!/usr/bin/env python3
import json
from threading import Thread
import rospy
import socket

from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

from utils.constants import Topics, Nodes

PORT = 13002


def activateFeature(feature: str, enabled: bool):
    """
    Enables or disables a feature. Done via publishing to \
    topics, implementation of enabling/disabling is done in Node itself

    Args:
        feature (str): Name of feature
        enabled (bool): If feature is enabled (True) or not (False)
    
    Returns:
        str: Response message
    """
    publisher = rospy.Publisher()
    if "lidar" in feature.lower():
        Thread(
            target=_publishFeature, args=[Topics.LIDAR_ENABLED.value, enabled]
        ).start()
    else:
        rospy.logwarn(
            f'Couldn\'t activate/deactivate feature "{feature}": Unkown feature'
        )
        return f'Error: Feature "{feature}" unknown.'

    return f'Success: Feature "{feature}" activate/deactivated'


def _publishFeature(topic: str, enabled: bool):
    publisher = rospy.Publisher(topic, Bool)
    rate = rospy.Rate(500)
    while not rospy.is_shutdown():
        try:
            publisher.publish(enabled)
        except:
            return
        try:
            # Look if other publisher publishes on same topic
            sniffer = rospy.wait_for_message(topic, bool, timeout=rate)
            if sniffer != enabled:
                # Another publishes on same topic with other value => end thread
                return
        except:
            pass

        rate.sleep()


def addOffset(offset: list, feature: str):
    """
    Adds an offset to an topic. Used for injecting faults by FleetIAS

    Args:
        offset (float): The value which should be added to the topic
        feature(str): Feature for which the offset should be added

    Returns:
        str: Response message
    """
    if feature == "Localization":
        # Create message
        data = rospy.wait_for_message(
            Topics.LOCALIZATION.value, PoseWithCovarianceStamped
        )
        data.pose.pose.position.x += offset[0]
        data.pose.pose.position.y += offset[1]
        # Publish message
        topic = Topics.ACML.value
        publisher = rospy.Publisher(PoseWithCovarianceStamped, topic)
        try:
            publisher.publish(data)
            return f"Success: Added offset {offset} to feature {feature}"
        except:
            return f"Error: Couldn't publish offset {offset} to feature {feature}"


def pushTarget(id=0, coordinate=(0, 0), type="workstation"):
    """
    Publishes a target so the navigation stacks starts the navigation.

    Args:
        type(str): Determines if target is workstation ("resource") or a coordinate ("coordinate")
        id (int, optional): ID of the target workstation. Only needed if type=="workstation"
        coordinate(set(int,int), optional): Coordinate to which the robotino should navigate. Only needed if type=="coordinate"

    Returns:
        str: Response message
    """
    if type == "resource":
        # Publish so id gets converted to coordinate
        publisher = rospy.Publisher(Int16, Topics.TARGET_ID.value)
        try:
            publisher.publish(Int16(id))
        except:
            return f"Error: Couldn't publish workstation target {id}"
        return f"Success: Started navigation to workstation {id}"
    elif type == "coordinate":
        target = PoseStamped()
        target.pose.position.x = coordinate[0]
        target.pose.position.y = coordinate[1]
        # Publish as a target coordinate directly
        publisher = rospy.Publisher(PoseStamped, Topics.TARGET.value)
        try:
            publisher.publish(target)
        except:
            return f"Error: Couldn't publish target coordinate {coordinate}"
        return f"Success: Started navigation to coordinate {coordinate}"
    else:
        return f'Error: Type "{type}" isn\'t a specified type'


def processMessage(data: dict):
    """
    Processes the message received by FleetIAS and does the corresponding actions

    Args:
        data (dict): The message from the socket. Must be already parsed to a dict
    """
    response = ""
    if data["command"].lower() == "pushtarget":
        if data["type"].lower() == "resource":
            rospy.logdebug(
                f'[FleetIAS-Client]: Received command "PushTarget with workstation target {data["workstationID"]}"'
            )
            response = pushTarget(type="resource", id=data["workstationID"])
        elif data["type"].lower() == "coordinate":
            rospy.logdebug(
                f'[FleetIAS-Client]: Received command "PushTarget with coordinate {data["coordinate"]}"'
            )
            response = pushTarget(type="coordinate", coordinate=data["coordinate"])
        else:
            pass
    elif data["command"].lower() == "activatefeature":
        rospy.logdebug(
            f'[FleetIAS-Client] Received to activate/deactivate feature {data["feature"]}: Enabled {data["value"]}'
        )
        response = activateFeature(feature=data["feature"], enabled=data["value"])
    elif data["command"].lower() == "addoffset":
        rospy.logdebug(
            f'[FleetIAS-Client] Received command "AddOffset" for feature {data["feature"]} with offset {data["value"]}'
        )
        response = addOffset(data["offset"], data["feature"])
    else:
        response = f'Error: Command "{data["command"]}" not implemented or not specified in message'

    return response


def runClient():
    ipAddr = server.gethostbyname(server.gethostname())
    # Setup socket
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((ipAddr, PORT))
    #  Start server
    server.listen()

    while True:
        try:
            # Connect with FleetIAS
            client, addr = server.accept()
            rospy.loginfo(f"[Fleetias-Client] {addr} connected to socket")
            # Receive message
            request = client.recv(512)
            if request:
                # Decode the message
                data = request.decode("utf-8")
                # Convert to dict
                data = json.loads(data)
                # Process the request
                response = processMessage(data=data)
                if not response.contains("Error") and data["command"] == "PushTarget":
                    # Wait for response from ROS task which was started by FleetIAS
                    response = rospy.wait_for_message(Topics.NAVIGATION_RESPONSE.value)
                else:
                    # Send error response to FleetIAS
                    rospy.logwarn(response)
                # Send response to FleetIAS
                client.sendall(bytes(response, encoding="utf-8"))
                # Close connection because FleetIAS connects per request
                client.close()
        except Exception as e:
            print(e)
            break


if __name__ == "__main__":
    rospy.init_node(Nodes.FLEETIAS_CLIENT.value)
    try:
        rospy.loginfo(f"Starting node {Nodes.FLEETIAS_CLIENT.value}")
        runClient()
        rospy.spin()
    except:
        rospy.loginfo(f"Shutdown node {Nodes.FLEETIAS_CLIENT.value}")
