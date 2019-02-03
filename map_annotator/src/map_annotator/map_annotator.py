#!/usr/bin/env python

import rospy
# from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import pickle
import os

class MapAnnotator(object):

    POSE_DICT = '/home/team6/catkin_ws/src/cse481wi19/map_annotator/pose_dict/pose_dict'

    def __init__(self):
        self._amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, callback=self._amcl_callback)
        self.pub =  rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=5)
        self.cur_pose = None
        self.pose_dict = self.load_pose_dict()

    def load_pose_dict(self):
        with open(MapAnnotator.POSE_DICT, 'rb') as f:
            return pickle.load(f)

    def dump_pose_dict(self):
        with open(MapAnnotator.POSE_DICT, 'wb') as f:
            pickle.dump(self.pose_dict, f)

    def _amcl_callback(self, msg):
        self.cur_pose = msg

    # List saved poses.
    def list(self):
        return self.pose_dict.keys()

    # save <name>: Save the robot\'s current pose as <name>. Overwrites if <name> already exists.
    def save(self, name):
        self.pose_dict[name] = self.cur_pose
        self.dump_pose_dict()

    # delete <name>: Delete the pose given by <name>.
    def delete(self, name):
        self.pose_dict.pop(name, None)
        self.dump_pose_dict()

    # goto <name>: Sends the robot to the pose given by <name>
    def goto(self, name):
        goal = PoseStamped()
        goal.header = self.pose_dict[name].header
        goal.pose = self.pose_dict[name].pose.pose
        self.pub.publish(goal)
