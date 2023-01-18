import rospy
import numpy as np

from geometry_msgs.msg import Twist, Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

from constants import Topics, Nodes

velocity_publisher_robot = rospy.Publisher("cmd_vel_real", Twist, queue_size=10)


def calc_command(target):
    """
    Calculates the distance to move and the angle to rotate to reach the specified target
    """
    # from [-1, 1] to [-pi, pi]
    currentPos = rospy.wait_for_message(
        Topics.LOCALIZATION.value, PoseWithCovarianceStamped
    )
    _, _, current_angle = euler_from_quaternion(currentPos.pose.pose.orientation)
    # current_angle = np.arcsin(current_angle) * 2
    # if current_angle < 0:  # from [-pi, pi] to [0, 2pi]
    #     current_angle = current_angle + (2 * np.pi)

    position = (currentPos.pose.pose.position.x, currentPos.pose.pose.position.y)
    distance = np.linalg.norm(target - position)

    # comm_angle is the rotation angle -> positive angles represent counter clockwise rotation
    beta = np.arccos(np.abs(target[0] - position[0]) / distance)
    # important: in comparison to the "coordinates" of the image, the y-Axis is inverted, so it's different from my test-script
    if current_angle > np.pi:
        # print('current_angle in rad:', current_angle)
        current_angle = current_angle - 2 * np.pi

    # 4. Quadrant
    if target[0] - position[0] >= 0 and target[1] - position[1] < 0:
        comm_angle = 2 * np.pi - (beta + current_angle)
    # 1. Quadrant
    elif target[0] - position[0] >= 0 and target[1] - position[1] >= 0:
        comm_angle = beta - current_angle
    # 3. Quadrant
    elif target[0] - position[0] < 0 and target[1] - position[1] < 0:
        comm_angle = np.pi + beta - current_angle
    # 2. Quadrant
    else:
        comm_angle = np.pi - (beta + current_angle)
    if comm_angle > np.pi:
        comm_angle = comm_angle - 2 * np.pi

    return comm_angle, distance


def move(target, dist):
    """
    Moves a specified distance forward

    Args:
        dist: The distance the Robotino should move
    """
    speed = 0.10

    msg_test_forward = Twist()
    msg_test_forward.linear.x = speed
    msg_test_forward.linear.y = 0
    msg_test_forward.linear.z = 0
    msg_test_forward.angular.x = 0
    msg_test_forward.angular.y = 0
    msg_test_forward.angular.z = 0

    msg_test_stop = Twist()
    msg_test_stop.linear.x = 0
    msg_test_stop.linear.y = 0
    msg_test_stop.linear.z = 0
    msg_test_stop.angular.x = 0
    msg_test_stop.angular.y = 0
    msg_test_stop.angular.z = 0

    t0 = rospy.Time.now().to_sec()
    current_dist = 0
    dist = np.abs(dist)

    while not rospy.is_shutdown():
        velocity_publisher_robot.publish(msg_test_forward)
        currentPos = rospy.wait_for_message(
            Topics.LOCALIZATION.value, PoseWithCovarianceStamped
        )
        position = (currentPos.pose.pose.position.x, currentPos.pose.pose.position.y)
        distance = np.linalg.norm(target - position)

        # Calculate current dist if needed for checking in future implementations
        t1 = rospy.Time.now().to_sec()
        current_dist = speed * (t1 - t0)

        if np.abs(distance) <= 0.1:
            break

    velocity_publisher_robot.publish(msg_test_stop)


def rotate(angle):
    """
    Rotates the Robotino until it has reached the specified angle

    Args:
        angle (float): The angle to which the RObotino should rotate. Is in range e[0,2pi]
    """
    rot_speed = 10 / (360) * (2 * np.pi)

    msg_test_stop = Twist()
    msg_test_stop.linear.x = 0
    msg_test_stop.linear.y = 0
    msg_test_stop.linear.z = 0
    msg_test_stop.angular.x = 0
    msg_test_stop.angular.y = 0
    msg_test_stop.angular.z = 0

    msg_test_rotate = Twist()
    msg_test_rotate.linear.x = 0
    msg_test_rotate.linear.y = 0
    msg_test_rotate.linear.z = 0
    msg_test_rotate.angular.x = 0
    msg_test_rotate.angular.y = 0
    if angle < 0:
        msg_test_rotate.angular.z = -rot_speed
    else:
        msg_test_rotate.angular.z = rot_speed

    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    angle = np.abs(angle)

    while not rospy.is_shutdown():
        velocity_publisher_robot.publish(msg_test_rotate)
        currentOdom = rospy.wait_for_message(Topics.ODOM.value, Odometry)
        _, _, currentAngle = euler_from_quaternion(currentOdom.pose.pose.orientation)

        # Calculate current angle if needed for checking in future implementations
        t1 = rospy.Time.now().to_sec()
        current_angle = rot_speed * (t1 - t0)

        if np.abs(currentAngle) <= angle:
            break

    velocity_publisher_robot.publish(msg_test_stop)


def navigateToPoint(target):
    """
    Navigates the real Robotino to an specified target by first rotating\
    towards the target and then driving forward

    Args:
        target (Point): The coordinate to which the Robotino should drive
    """
    comm_angle, comm_dist = calc_command(target)

    rospy.logdebug(f"Started moving to target {target}")
    rotate(comm_angle)
    move(target, comm_dist)
    rospy.logdebug(f"Arrived at target {target}")

    rospy.Publisher(Topics.NAVIGATION_RESPONSE.value, Bool).publish(True)


def control():
    """
    Runs the node itself and subscribes to all necessary topics.
    """
    rospy.init_node(Nodes.CONTROL.value)
    # Starts navigation to a point
    rospy.Subscriber(Topics.NAV_POINT, Point, navigateToPoint)

    # Prevents python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    control()
