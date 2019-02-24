#!/usr/bin/env python

import rospy
# from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import pickle
import os

class PoseState(object):
    def __init__(self, pose, gripper):
        self.pose_stamped = pose
        self.gripper_open = gripper

class GripperProgram(object):

    PROGRAM_PATH = '/home/team6/catkin_ws/src/cse481wi19/gripper_program/programs/'

    def __init__(self):
        self.program = []
        self.created = False

    def create_program(self):
        self.program = []
        self.created = True

    def save_pose(self, pose_stamped, gripper):
        if not self.created:
            print("program is not created!!!")
            return

        pose_state = PoseState(pose_stamped, gripper)
        self.program.append(pose_state)

    def save_program(self, name):
        if not self.created:
            print("program is not created!!!")
            return
        with open(GripperProgram.PROGRAM_PATH + name, 'wb') as f:
            pickle.dump(self.program, f)

    def load_program(self, name):
        if not os.path.exists(GripperProgram.PROGRAM_PATH + name):
            print( name + " program does not exits.")
            return None
        with open(GripperProgram.PROGRAM_PATH + name, 'rb') as f:
            return pickle.load(f)
