#!/usr/bin/env python
import copy
import rospy
import robot_api
import gripper_program
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
import numpy as np
import sys
import tf.transformations as tft
import actionlib
import robot_controllers_msgs
from robot_controllers_msgs.msg import ControllerState, QueryControllerStatesGoal

def pose_to_transform(pose):
    q = pose.orientation
    matrix = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    matrix[0, 3] = pose.position.x
    matrix[1, 3] = pose.position.y
    matrix[2, 3] = pose.position.z
    return matrix


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

def compute_absolute_pose(obj_ps, goal_tf):
    obj_tf = pose_to_transform(obj_ps.pose) # word^T_obj
    result_tf = np.dot(obj_tf, goal_tf) # word^T_obj * obj^T_goal
    result = PoseStamped()
    result.header = obj_ps.header
    result.pose = transform_to_pose(result_tf)
    return result

def extract_marker_info(obj_marker):
    obj_ps = PoseStamped()
    obj_ps.header.frame_id = 'base_link'
    obj_ps.pose = obj_marker.pose
    obj_scale = obj_marker.scale
    return obj_ps, obj_scale

def try_grasp_plan(obj_ps, obj_scale):
    pregrasp_in_obj = np.array([
        [1, 0, 0, obj_scale.y/2 - 0.1],
        [0, 1, 0,    0],
        [0, 0, 1,    0],
        [0, 0, 0,    1],
    ])
    if not self._arm.compute_ik(pregrasp_ps):
        return None
    return pregrasp_in_obj

class ArmMotionPlanner(object):
    def __init__(self, arm):
        self._arm = arm
        print("Initialized ArmMotionPlanner")
        # TODO: have a collision map?
        # self.planning_scene = PlanningSceneInterface(frame='base_link')

    # def open(self):
    #     self._gripper.open()


    def pick_up(self, obj_marker, obs_marker_list=None):
        # TODO: for each obstacle in list, add add_obstacle
        if obj_marker == None:
            rospy.logerr("Passed in None")
            return false

        obj_ps, obj_scale = extract_marker_info(obj_marker)
        pregrasp_in_obj = try_grasp_plan(obj_ps, obj_scale)
        if pregrasp_in_obj is None:
            return false
        grasp_in_obj = copy.deepcopy(pregrasp_in_obj)
        grasp_in_obj[0, 3] += 0.1 # x_obj += 0.15
        lift_in_obj = copy.deepcopy(grasp_in_obj)
        lift_in_obj[2, 3] += 0.3 # z_obj += 0.3
        pregrasp_ps = compute_absolute_pose(obj_ps, pregrasp_in_obj)
        grasp_ps = compute_absolute_pose(obj_ps, grasp_in_obj)
        lift_ps = compute_absolute_pose(obj_ps, lift_in_obj)
        kwargs = {
            'allowed_planning_time': 20,
            'execution_timeout': 20,
            'num_planning_attempts': 10,
            'replan': True
        }

        error = self._arm.move_to_pose(pregrasp_ps, **kwargs)
        if error is not None:
            rospy.logerr(error)
            return false

        return true
        # TODO: Need to find a suitable pre_grasp position
        # TODO: close gripper after grasph_ps
        # TODO: lift_ps should be visible
        # TODO2: when lifting, avoid twist of arm???
        # TODO: remove all_obstacles.

    def add_obstacle(self, obs_ps):
        self.planning_scene.removeCollisionObject('table')
        # Create table obstacle
        table_size_x = 0.5
        table_size_y = 1
        table_size_z = 0.03
        table_x = 0.8
        table_y = 0
        table_z = 0.6
        self.planning_scene.addBox('table', table_size_x, table_size_y, table_size_z,
                              table_x, table_y, table_z)
        return


# def shutdown():
#     arm.cancel_all_goals()
# rospy.on_shutdown(shutdown)
