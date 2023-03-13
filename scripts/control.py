#! /usr/bin/env python3
import os
import rospy
import actionlib
import numpy as np
from geometry_msgs.msg import Twist, Point

from prototype.msg import (
    PRMNavigationFeedback,
    PRMNavigationResult,
    PRMNavigationAction,
)
from utils.constants import Nodes
from utils.ros_logger import set_rospy_log_lvl


class PRMNavigationAction(object):
    _feedback = PRMNavigationFeedback()
    _result = PRMNavigationResult()
    velocity_publisher_robot = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def __init__(self):
        self._action_name = Nodes.CONTROL.value
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            PRMNavigationAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()

    def execute_cb(self, goal):
        self._feedback.currentAngle = 0
        self._feedback.currentDist = 0
        self._result.result = False
        rospy.loginfo(f"[Control] Started moving to target {goal.target}.")

        comm_angle, comm_dist = self.calc_command(
            goal.target, goal.positionAssumption, goal.angleAssumption
        )

        if self._as.is_preempt_requested():
            self._result.result = False
            return
        current_angle = self.rotate(comm_angle)

        if self._as.is_preempt_requested():
            self._result.result = False
            self._result.currentAngle = current_angle
            return
        current_dist = self.move(comm_dist)

        if self._as.is_preempt_requested():
            self._result.result = False
        else:
            self._result.result = True
        self._result.currentDist = current_dist
        self._result.currentAngle = current_angle

        rospy.loginfo(f"[Control] Arrived at target {goal.target}")
        self._as.set_succeeded(self._result)

    def calc_command(target: Point, positionAssumption: Point, angleAssumption: float):
        """
        Calculates the distance to move and the angle to rotate to reach the specified target

        Args:
            target (geometry_msgs.Point): The coordinate to which the Robotino should move

        Returns:
            comm_angle (float): The angle the Robotino has to rotate
            distance (float): The distance the Robotino has to move
        """
        # from [-1, 1] to [-pi, pi]

        if angleAssumption < 0:  # from [-pi, pi] to [0, 2pi]
            angleAssumption = angleAssumption + (2 * np.pi)

        position = np.array([positionAssumption.x, positionAssumption.y])
        target = np.array([target.x, target.y])
        distance = np.linalg.norm(target - position)

        # comm_angle is the rotation angle -> positive angles represent counter clockwise rotation
        beta = np.arccos(np.abs(target[0] - position[0]) / distance)
        # important: in comparison to the "coordinates" of the image, the y-Axis is inverted, so it's different from my test-script
        if angleAssumption > np.pi:
            # print('current_angle in rad:', current_angle)
            angleAssumption = angleAssumption - 2 * np.pi
        # 4. Quadrant
        if target[0] - position[0] >= 0 and target[1] - position[1] < 0:
            comm_angle = 2 * np.pi - (beta + angleAssumption)
        # 1. Quadrant
        elif target[0] - position[0] >= 0 and target[1] - position[1] >= 0:
            comm_angle = beta - angleAssumption
        # 3. Quadrant
        elif target[0] - position[0] < 0 and target[1] - position[1] < 0:
            comm_angle = np.pi + beta - angleAssumption
        # 2. Quadrant
        else:
            comm_angle = np.pi - (beta + angleAssumption)
        if comm_angle > np.pi:
            comm_angle = comm_angle - 2 * np.pi

        return comm_angle, distance

    def move(self, dist):
        """
        Moves a specified distance forward

        Args:
            dist: The distance the Robotino should move

        Returns:
            bool: If it moved successfully to the target (True) or failed (False)
        """
        # Little offset because its always a little to long
        OFFSET = 0.2
        speed = 0.10
        rospy.logdebug(f"[Control] Start moving {dist} meters")

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

        while current_dist < dist - OFFSET:
            if self._as.is_preempt_requested():
                self._result.currentDist = current_dist
                self._result.result = False
                return current_dist

            self.velocity_publisher_robot.publish(msg_test_forward)
            t1 = rospy.Time.now().to_sec()
            current_dist = speed * (t1 - t0)
            self._feedback.currentDist = current_dist
            self._as.publish_feedback(self._feedback)

        self.velocity_publisher_robot.publish(msg_test_stop)
        return current_dist

    def rotate(self, angle):
        """
        Rotates the Robotino until it has reached the specified angle

        Args:
            angle (float): The angle to which the Robotino should rotate. Is in range e[0,2pi]

        Returns:
            bool: If it rotated successfully towards the target (True) or failed (False)
        """
        # Little offset because its always a little to less rotation
        OFFSET = 0.005

        rospy.logdebug(f"[Control] Start rotating {angle} radians")
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

        while current_angle < angle + OFFSET:
            if self._as.is_preempt_requested():
                self._result.currentAngle = current_angle
                self._result.result = False
                return current_angle

            self.velocity_publisher_robot.publish(msg_test_rotate)
            t1 = rospy.Time.now().to_sec()
            current_angle = rot_speed * (t1 - t0)
            self._feedback.currentAngle = current_angle
            self._as.publish_feedback(self._feedback)

        self.velocity_publisher_robot.publish(msg_test_stop)

        return current_angle


if __name__ == "__main__":
    rospy.init_node(Nodes.CONTROL.value)
    s = PRMNavigationAction()
    set_rospy_log_lvl(rospy.INFO)
    rospy.spin()
