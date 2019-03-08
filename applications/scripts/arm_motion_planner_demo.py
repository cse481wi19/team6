#! /usr/bin/env python

import robot_api
from robot_api import ArmMotionPlanner
import gripper_program
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
from visualization_msgs.msg import Marker
import tf
import numpy as np
import rospy
import sys
import actionlib
import robot_controllers_msgs
import tf.transformations as tft
from robot_controllers_msgs.msg import ControllerState, QueryControllerStatesGoal
from ar_track_alvar_msgs.msg import AlvarMarkers

def wait_for_time():
    """
    Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class MarkerReader(object):
    def __init__(self):
        self.obj_marker = None
        print("mr init")

    def callback(self, msg):
        if msg.type == Marker.CUBE:
            self.obj_marker = msg

def print_usage():
    print('rosrun application load_exec_program <name>')


def main():
    rospy.init_node('arm_motion_planner_demo')
    wait_for_time()
    # argv = rospy.myargv()
    # if len(argv) < 2:
    #     print_usage()
    #     return
    # name = argv[1]
    # rospy.sleep(0.5)

    torso = robot_api.Torso()
    torso.set_height(robot_api.Torso.MAX_HEIGHT)

    gripper = robot_api.Gripper()
    gripper.open()
    reader = MarkerReader()
    marker_sub = rospy.Subscriber('object_markers', Marker, callback=reader.callback)
    # rate = rospy.Rate(10)
    # rate.sleep()
    rospy.sleep(2)
    arm = robot_api.Arm()
    arm_planner = ArmMotionPlanner(arm)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # print(reader.obj_marker)
        arm_planner.pick_up(reader.obj_marker)
        rate.sleep()

    # rospy.spin()


if __name__ == '__main__':
    main()
