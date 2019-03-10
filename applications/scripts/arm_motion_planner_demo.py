#! /usr/bin/env python
import copy
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
        self.obj_markers = set()
        print("Initialized MarkerReader")
        # self.obj_marker = None

    def callback(self, msg):
        if msg.type == Marker.CUBE and msg.pose.position.x < 0.6:
            self.obj_markers.add(msg)
            # self.obj_marker = msg

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
    torso.set_height(0)
    arm = robot_api.Arm()
    gripper = robot_api.Gripper()

    reader = MarkerReader()
    marker_sub = rospy.Subscriber('object_markers', Marker, callback=reader.callback)

    rospy.sleep(2)
    arm_planner = ArmMotionPlanner(arm, gripper)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # print(reader.obj_marker)
        obj_list = copy.deepcopy(reader.obj_markers)
        # obj_list = set(copy.deepcopy(obs_list))
        for obj_marker in obj_list:
            # for obs_marker in obs_list:
            #     print(obs_marker)
            # print()
            # print(obj_marker)
            # obs_list.remove(obj_marker)
            arm_planner.pick_up(obj_marker)
            # obs_list.add(obj_marker)
        # arm_planner.pick_up(reader.obj_marker)
        rate.sleep()

    # rospy.spin()


if __name__ == '__main__':
    main()
