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

PREGRASP = 0.24

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
    obj_ps.header.frame_id = obj_marker.header.frame_id
    obj_ps.pose = obj_marker.pose
    obj_scale = obj_marker.scale
    return obj_ps, obj_scale



class ArmMotionPlanner(object):
    def __init__(self, arm, gripper):
        self._arm = arm
        self._gripper = gripper
        print("Initialized ArmMotionPlanner")
        # TODO: have a collision map?
        # self.planning_scene = PlanningSceneInterface(frame='base_link')

    # def open(self):
    #     self._gripper.open()
    def try_grasp_plan(self, obj_ps, obj_scale):
        z_pos = obj_ps.pose.position.z
        if obj_scale.y < obj_scale.x:
            pregrasp_in_obj = np.array([
                [ 0, 0, 1,    0],
                [ 0, 1, 0,    0],
                [-1, 0, 0,     z_pos + obj_scale.z/2 + PREGRASP],
                [ 0, 0, 0,    1],
            ])
        else:
            pregrasp_in_obj = np.array([
                [ 0, -1, 0,    0],
                [ 0,  0, 1,    0],
                [-1,  0, 0,     z_pos + obj_scale.z/2 + PREGRASP],
                [ 0,  0, 0,    1],
            ])

        # print(pregrasp_in_obj)
        pregrasp_ps = compute_absolute_pose(obj_ps, pregrasp_in_obj)
        grasp_in_obj = copy.deepcopy(pregrasp_in_obj)
        grasp_in_obj[2, 3] -= 0.14
        # print("pregrasp_in_obj")
        # print(pregrasp_in_obj)
        # print("grasp_in_obj")
        # print(grasp_in_obj)
        grasp_ps = compute_absolute_pose(obj_ps, grasp_in_obj)
        grasp_ps.pose.position.z = max(grasp_ps.pose.position.z, 0.23)
        # print("pregrasp_ps")
        # print(pregrasp_ps)
        # print("grasp_ps")
        # print(grasp_ps)
        error = self._arm.check_pose(pregrasp_ps)
        if error is not None:
            print("pre_grasp failed")
            return None, None

        return pregrasp_ps, grasp_ps

    def pick_up(self, obj_marker, obs_marker_list=None):
        # TODO: for each obstacle in list, add add_obstacle
        if obj_marker is None:
            rospy.logerr("Passed in None")
            return False


        self.add_obstacles(obs_marker_list)

        self._gripper.open()
        start = PoseStamped()
        start.header.frame_id = 'base_link'
        start.pose.position.x = 0.77
        start.pose.position.y = -0.1
        start.pose.position.z = 0.55

        error = self._arm.move_to_pose(start)
        if error is not None:
            rospy.logerr(error)
            print("start position failed.")
            self.remove_obstacles(obs_marker_list)
            return False

        obj_ps, obj_scale = extract_marker_info(obj_marker)
        pregrasp_ps, grasp_ps = self.try_grasp_plan(obj_ps, obj_scale)
        if pregrasp_ps is None:
            print("Not reachable")
            self.remove_obstacles(obs_marker_list)
            return False


        kwargs = {
            'allowed_planning_time': 20,
            'execution_timeout': 20,
            'num_planning_attempts': 10,
            'replan': True
        }

        error = self._arm.move_to_pose(pregrasp_ps, **kwargs)
        if error is not None:
            rospy.logerr(error)
            self.remove_obstacles(obs_marker_list)
            return False

        error = self._arm.check_pose(grasp_ps)
        if error is not None:
            print("grasp failed")
            self.remove_obstacles(obs_marker_list)
            return False
        print(grasp_ps)
        error = self._arm.move_to_pose(grasp_ps, **kwargs)
        if error is not None:
            rospy.logerr(error)
            self.remove_obstacles(obs_marker_list)
            return False

        self._gripper.close()
        error = self._arm.move_to_pose(start)
        if error is not None:
            rospy.logerr(error)
            print("lift position failed.")
            self.remove_obstacles(obs_marker_list)
            return False

        self.remove_obstacles(obs_marker_list)
        return True
        # TODO: Need to find a suitable pre_grasp position
        # TODO: close gripper after grasph_ps
        # TODO: lift_ps should be visible
        # TODO2: when lifting, avoid twist of arm???
        # TODO: remove all_obstacles.

    def add_obstacles(self, obs_marker_list):
        if obs_marker_list is None:
            return
        i = 0
        for obs_marker in obs_marker_list:
            obs_ps, obs_scale = extract_marker_info(obs_marker)
            obs_size_x = obs_scale.x
            obs_size_y = obs_scale.y
            obs_size_z = obs_scale.z
            obs_x = obs_ps.pose.position.x
            obs_y = obs_ps.pose.position.x
            obs_z = obs_ps.pose.position.x
            self.planning_scene.addBox('obs' + i, obs_size_x, obs_size_y, obs_size_z,
                                  obs_x, obs_y, obs_z)
            i += 1
        return

    def remove_obstacles(self, obs_marker_list):
        if obs_marker_list is None:
            return
        i = 0
        for obs_ps in obs_ps_list:
            self.planning_scene.removeCollisionObject('obs' + i)
            i += 1
        return

# def shutdown():
#     arm.cancel_all_goals()
# rospy.on_shutdown(shutdown)
