#! /usr/bin/env python

# TODO: import ????????_msgs.msg
import rospy
import copy
from  geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from nav_msgs.msg import Odometry
import tf.transformations as tft
import math


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        # TODO: Create publisher
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self._odom_sub = rospy.Subscriber('odom', Odometry, callback=self._odom_callback)
        self.cur_odom = None

    def _odom_callback(self, msg):
        self.cur_odom = msg

    def get_distance(self, start_odom):
        x1, y1 = start_odom.pose.pose.position.x, start_odom.pose.pose.position.y
        x0, y0 = self.cur_odom.pose.pose.position.x, self.cur_odom.pose.pose.position.y
        return math.sqrt((x1-x0)**2 + (y1-y0)**2)


    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        # rospy.sleep until the base has received at least one message on /odom
        while self.cur_odom is None:
            rospy.sleep(0.5)
        # record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self.cur_odom)
        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has traveled the desired distance
        # TODO: Be sure to handle the case where the distance is negative!
        while math.fabs(self.get_distance(start) - math.fabs(distance)) > speed:
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            rate.sleep()

    def get_cur_rad(self):
        q = self.cur_odom.pose.pose.orientation
        m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        x = m[0, 0] # The x value of the x-axis (first column)
        y = m[1, 0] # The y value of the x-axis
        return math.atan2(y, x)

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        while self.cur_odom is None:
            rospy.sleep(0.5)
        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self.cur_odom)
        q = start.pose.pose.orientation
        m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        x = m[0, 0] # The x value of the x-axis (first column)
        y = m[1, 0] # The y value of the x-axis
        start_rads = math.atan2(y, x)
        # print('start_rads: ', str(start_rads))
        # TODO: What will you do if angular_distance is greater than 2*pi or less than -2*pi?
        if angular_distance < -2*math.pi or angular_distance > 2*math.pi:
            sign = angular_distance / math.fabs(angular_distance)
            angular_distance = math.fabs(angular_distance) % (2*math.pi)
            angular_distance *= sign

        # target_rads must be in [-pi, pi]
        target_rads = (start_rads + math.pi + angular_distance) % (2*math.pi) - math.pi

        # print('angular_distance: ', str(angular_distance))
        # print('target_rads: ', str(target_rads))
        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has rotated the desired amount
        # TODO: Be sure to handle the case where the desired amount is negative!
        while math.fabs(self.get_cur_rad() - target_rads) > speed / 5.0:
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if angular_distance < 0 else 1
            self.move(0, direction * speed)
            rate.sleep()

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        # TODO: Create Twist msg
        # TODO: Fill out msg
        # TODO: Publish msg
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.pub.publish(twist)

    def stop(self):
        """Stops the mobile base from moving.
        """
        # TODO: Publish 0 velocity
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub.publish(twist)
