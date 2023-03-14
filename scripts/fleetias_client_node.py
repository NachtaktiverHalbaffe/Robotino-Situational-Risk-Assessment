#!/usr/bin/env python3
import json
import rospy
import socket
from threading import Thread
from std_msgs.msg import Int16, Bool, String, Float32
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

from utils.constants import Topics, Nodes
from utils.evalmanager_client import EvalManagerClient

PORT = 13002

targetIdPub = rospy.Publisher(Topics.TARGET_ID.value, Int16, queue_size=10)
targetCorPub = rospy.Publisher(Topics.TARGET.value, PoseStamped, queue_size=10)
lidarFeaturePub = rospy.Publisher(
    Topics.LIDAR_BREAKDOWN.value, Bool, queue_size=10, latch=True
)
injectOffsetEnabledPub = rospy.Publisher(
    Topics.INJECT_OFFSET.value, Bool, queue_size=10, latch=True
)
offsetPub = rospy.Publisher(Topics.OFFSET.value, Float32, queue_size=10, latch=True)


def activateFeature(feature: str, enabled: bool):
    """
    Enables or disables a feature. Done via publishing to \
    topics. Implementation of enabling/disabling is done in Node itself

    Args:
        feature (str): Name of feature
        enabled (bool): If feature is enabled (True) or not (False)
    
    Returns:
        str: Response message
    """
    if "lidar" in feature.lower():
        EvalManagerClient().evalLogLIDAR(enabled)
        try:
            lidarFeaturePub.publish(not enabled)
        except:
            return
    else:
        rospy.logwarn(
            f'Couldn\'t activate/deactivate feature "{feature}": Unkown feature'
        )
        return f'Error: Feature "{feature}" unknown.'

    return f'Success: Feature "{feature}" activate/deactivated'


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
        try:
            injectOffsetEnabledPub.publish(True)
            offsetPub.publish(float(offset[0]))
            return f"Success: Added offset {offset} to feature {feature}"
        except Exception as e:
            return f"Error: Couldn't publish offset {offset} to feature {feature}: {e}"
    else:
        injectOffsetEnabledPub.publish(False)
        offsetPub.publish(0)


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

        try:
            targetIdPub.publish(Int16(id))
        except:
            return f"Error: Couldn't publish workstation target {id}"
        return f"Success: Started navigation to workstation {id}"
    elif type == "coordinate":
        target = PoseStamped()
        target.pose.position.x = coordinate[0]
        target.pose.position.y = coordinate[1]
        # Publish as a target coordinate directly
        try:
            targetCorPub.publish(target)
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
            f'[FleetIAS-Client] Received command "AddOffset" for feature {data["feature"]} with offset {data["offset"]}'
        )
        response = addOffset(data["offset"], data["feature"])
    else:
        response = f'Error: Command "{data["command"]}" not implemented or not specified in message'

    return response


def communicate(client, server):
    """
    Handles the communication between the TCP server and client. It receives and fetches a \
    message, processes them and send a response back to the client
    
    Args:
        client (socket): The client which send the request
        server (socket): The server itself
    """
    server.settimeout(None)
    request = client.recv(512)
    if request:
        # Decode the message
        data = request.decode("utf-8")
        # Convert to dict
        data = json.loads(data)
        # Process the request
        response = processMessage(data=data)
        if not "error" in response.lower() and data["command"].lower() == "pushtarget":
            # Wait for response from ROS task which was started by FleetIAS
            response = rospy.wait_for_message(
                Topics.STRATEGY_PLANNER_RESPONSE.value, String
            )
            response = response.data
        else:
            # Send error response to FleetIAS
            rospy.logwarn(response)
        # Send response to FleetIAS
        client.sendall(bytes(response, encoding="utf-8"))
        # Close connection because FleetIAS connects per request
        client.close()


def runClient():
    """
    Runs the FleetIAS client. Although being named a client, it's technically a TCP server.
    """
    ipAddr = rospy.get_param("~ip", default="129.69.102.180")
    # Setup socket
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    print(ipAddr)
    server.bind((ipAddr, PORT))
    #  Start server
    server.listen()

    while not rospy.is_shutdown():
        try:
            # Connect with FleetIAS
            # server.settimeout(2)
            client, addr = server.accept()
            rospy.loginfo(f"[Fleetias-Client] {addr} connected to socket")
            Thread(target=communicate, args=[client, server]).start()

        except Exception as e:
            print(e)
            break


if __name__ == "__main__":
    rospy.init_node(Nodes.FLEETIAS_CLIENT.value)
    # try:
    rospy.loginfo(f"Starting node {Nodes.FLEETIAS_CLIENT.value}")
    runClient()
    rospy.spin()
    # except:
    #     rospy.loginfo(f"Shutdown node {Nodes.FLEETIAS_CLIENT.value}")
