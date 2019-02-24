#! /usr/bin/env python

import robot_api
import gripper_program
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
import numpy as np
import rospy
import sys
import tf.transformations as tft

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


def wait_for_time():
    """
    Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class ArTagReader(object):
    def __init__(self):
        self.markers = {}

    def callback(self, msg):
        for marker in msg.markers:
            self.markers[str(marker.id)] = marker

def print_usage():
    print('rosrun application load_exec_program <name>')


def main():
    rospy.init_node('load_exec_program_demo')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return
    name = argv[1]

    rospy.sleep(0.5)
    goal = QueryControllerStatesGoal()
    state = ControllerState()
    state.name = 'arm_controller/follow_joint_trajectory'
    state.state = ControllerState.RUNNING
    goal.updates.append(state)
    self._controller_client.send_goal(goal)
    self._controller_client.wait_for_result()

    torso = robot_api.Torso()
    torso.set_height(robot_api.Torso.MAX_HEIGHT)
    arm = robot_api.Arm()

    program = gripper_program.GripperProgram()
    gripper = robot_api.Gripper()

    tf_listener = tf.TransformListener()

    reader = ArTagReader()
    marker_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback=reader.callback) # Subscribe to AR tag poses, use reader.callback

    # rate = rospy.Rate(10)
    # rate.sleep()
    rospy.sleep(1)
    actions = program.load_program(name)
    if actions is None:
        exit(1)

    for action in actions:
        if action.gripper_open:
            gripper.open()
        else:
            gripper.close(robot_api.Gripper.MAX_EFFORT)

        pose_stamped = action.pose_stamped
        id = pose_stamped.header.frame_id
        if id not in reader.markers and id != 'base_link':
            print("Frame does not exits.")
            exit(1)
        if id in reader.markers:
            marker_tf = pose_to_transform(reader.markers[id].pose.pose)
            # print(reader.markers[id].pose)
            goal_tf = pose_to_transform(pose_stamped.pose)
            result_tf = np.dot(marker_tf, goal_tf)
            pose_stamped.header.frame_id = 'base_link'
            pose_stamped.pose = transform_to_pose(result_tf)
            # pose_stamped.pose = reader.markers[id].pose.pose
        print(pose_stamped)
        error = arm.move_to_pose(pose_stamped)
        if error is not None:
            rospy.logerr(error)

    # rospy.spin()


if __name__ == '__main__':
    main()
