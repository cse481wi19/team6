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
import actionlib
import robot_controllers_msgs
from robot_controllers_msgs.msg import ControllerState, QueryControllerStatesGoal
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

    def print_marker_id(self):
        for marker in self.markers:
            print(marker + ", "),
        print("base_link")


def print_usage():
    print('Commands:\n' +\
          '  1: Create program.\n' +\
          '  2: Save pose.\n' +\
          '  3: Open gripper.\n' +\
          '  4: Close gripper.\n' +\
          '  5 <name>: Save program with <name>\n' +\
          '  q: quit\n')


def main():
    rospy.init_node('create_save_program_demo')
    wait_for_time()
    print('Welcome to the gripper program!\n')
    print_usage()
    rospy.sleep(0.5)
    control_client = actionlib.SimpleActionClient("/query_controller_states", robot_controllers_msgs.msg.QueryControllerStatesAction)
    # TODO: Wait for server
    control_client.wait_for_server()
    goal = QueryControllerStatesGoal()
    state = ControllerState()
    state.name = 'arm_controller/follow_joint_trajectory'
    state.state = ControllerState.STOPPED
    goal.updates.append(state)

    control_client.send_goal(goal)
    control_client.wait_for_result()

    # program = gripper_program.GripperProgram()
    # gripper = robot_api.Gripper()
    # gripper_open = True
    # tf_listener = tf.TransformListener()
    # reader = ArTagReader()
    # marker_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback=reader.callback) # Subscribe to AR tag poses, use reader.callback
    #
    # rate = rospy.Rate(10)
    #
    # while True :
    #     print('\n> '),
    #     l = sys.stdin.readline().split()
    #     c = l[0]
    #     name = None
    #     if len(l) > 1:
    #         name = ' '.join(l[1:])
    #
    #     if c == 'q':
    #         break
    #     elif c == 'help':
    #         print_usage()
    #     elif c == '1':
    #         program.create_program()
    #         gripper.open()
    #         gripper_open = True
    #         print("program created")
    #     elif c == '2':
    #         print("You have the following options as reference frame:")
    #         reader.print_marker_id()
    #         print('frame: '),
    #         l = sys.stdin.readline().split()
    #         c = l[0]
    #         if c not in reader.markers and c is not 'base_link':
    #             print("reference name not found")
    #             continue
    #         # l = sys.stdin.readline().split()
    #         now = rospy.Time.now()
    #         tf_listener.waitForTransform('base_link', 'wrist_roll_link', now, rospy.Duration(4.0))
    #         trans, rot = tf_listener.lookupTransform('base_link', 'wrist_roll_link', now)
    #
    #         ps = PoseStamped()
    #         ps.header.frame_id = 'base_link'
    #         pose = Pose()
    #         pose.position.x = trans[0]
    #         pose.position.y = trans[1]
    #         pose.position.z = trans[2]
    #
    #         pose.orientation.x = rot[0]
    #         pose.orientation.y = rot[1]
    #         pose.orientation.z = rot[2]
    #         pose.orientation.w = rot[3]
    #
    #         if c in reader.markers:
    #             gripper_pos_tf = pose_to_transform(pose)
    #             dest_marker_tf = pose_to_transform(reader.markers[c].pose.pose)
    #             inverse = np.matrix(dest_marker_tf).I
    #             result_tf = np.dot(inverse, gripper_pos_tf)
    #             ps.header.frame_id = c
    #             ps.pose = transform_to_pose(result_tf)
    #         else:
    #             ps.pose = pose
    #
    #         program.save_pose(ps, gripper_open)
    #         print("pose saved")
    #     elif c == '3':
    #         gripper.open()
    #         gripper_open = True
    #         print("gripper opened")
    #     elif c == '4':
    #         gripper.close(robot_api.Gripper.MAX_EFFORT)
    #         gripper_open = False
    #         print("gripper closed")
    #     elif c == '5':
    #         if name is None:
    #             print("save program requires a name")
    #             continue
    #         program.save_program(name)
    #         print("program saved")

        # rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
