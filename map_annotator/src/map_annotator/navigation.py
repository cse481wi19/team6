#!/usr/bin/env python

import rospy
import actionlib
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from map_annotator.msg import PoseNames, UserAction
# import tf
# import tf.transformations as tft
# from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus
import numpy as np

def transform_to_pose(matrix):
    pose = Pose()
    pose.position.x = matrix[0, 3]
    pose.position.y = matrix[1, 3]
    pose.position.z = matrix[2, 3]
    x, y, z, w = tft.quaternion_from_matrix(matrix)
    pose.orientation.x = x
    pose.orientation.y = y
    pose.orientation.z = z
    pose.orientation.w = w
    return pose

def pose_to_transform(pose):
    q = pose.orientation
    matrix = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    matrix[0, 3] = pose.position.x
    matrix[1, 3] = pose.position.y
    matrix[2, 3] = pose.position.z
    return matrix

class Navigator(object):
    def __init__(self):
        # self.pub =  rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=5)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.curr_pose = None
        self._amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, callback=self._amcl_callback)
        # self.tf_listener = tf.TransformListener()
        # self.transforms = {}
        # self.load_tf("base_link", 4)


    def _amcl_callback(self, msg):
        self.curr_pose = PoseStamped()
        self.curr_pose.header = msg.header
        self.curr_pose.pose = msg.pose.pose
        return self.curr_pose


    def current_pose(self):
        # rosrun tf view_frames
        return self.curr_pose



    # def load_tf(self, tgt_frame, timeout):
    #     if tgt_frame not in self.transforms:
    #         self.tf_listener.waitForTransform('map', tgt_frame, rospy.Time(), rospy.Duration(secs=4))
    #     now = rospy.Time.now()
    #     try:
    #         self.tf_listener.waitForTransform('map', tgt_frame, now, rospy.Duration(secs=timeout))
    #         tf_res = self.tf_listener.lookupTransform('map', tgt_frame, now)
    #         self.transforms[tgt_frame] = tf_res
    #         return True
    #     except:
    #         return False


    def goto(self, goal, timeout):
        # print(goal)
        # tgt_frame = goal.header.frame_id
        # if tgt_frame != 'map':
        #     self.load_tf(tgt_frame, 0.5)
        #     if tgt_frame not in self.transforms:
        #         return False

        move_base_goal = MoveBaseGoal()
        # if tgt_frame == 'map':
        move_base_goal.target_pose = goal
        # else:
        #     move_base_goal.target_pose.header.frame_id = 'map'
        #     pose = Pose()
        #     trans, rot = self.transforms[tgt_frame]
        #     pose.position.x = trans[0]
        #     pose.position.y = trans[1]
        #     pose.position.z = trans[2]
        #
        #     pose.orientation.x = rot[0]
        #     pose.orientation.y = rot[1]
        #     pose.orientation.z = rot[2]
        #     pose.orientation.w = rot[3]
        #
        #     tgt_frame_in_map = pose_to_transform(pose)
        #     tgt_in_tgt_frame = pose_to_transform(goal.pose)
        #     tgt_in_map = np.dot(tgt_frame_in_map, tgt_in_tgt_frame)
        #     move_base_goal.target_pose.pose = transform_to_pose(tgt_in_map)
        # print(move_base_goal.target_pose)
        self.move_base.send_goal(move_base_goal)
    	success = self.move_base.wait_for_result(rospy.Duration(timeout))
        state = self.move_base.get_state()
    	if not success or state != GoalStatus.SUCCEEDED:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to move forward for some reason")
            return False
        return True
