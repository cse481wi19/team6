#!/usr/bin/env python

import rospy
import robot_api
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, MenuEntry
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
import math
import numpy as np
import tf.transformations as tft
import copy

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

CLOSE_FINGER_POS = 0.0
OPEN_FINGER_POS = 0.1

def interactive_gripper_marker(pose_stamped, finger_distance):
    #  gripper marker
    gripper_marker = Marker()
    gripper_marker.pose.position.x = 0.166 # wrist_roll_link/gripper_link Translation: [0.166, 0.000, 0.000]
    gripper_marker.pose.orientation.w = 1 # wrist_roll_link/gripper_link Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
    gripper_marker.type = Marker.MESH_RESOURCE
    gripper_marker.mesh_resource = GRIPPER_MESH
    gripper_marker.mesh_use_embedded_materials = True

    finger_distance = max(CLOSE_FINGER_POS, min(finger_distance, OPEN_FINGER_POS))
    # left finger marker
    l_marker = Marker()
    l_marker.pose.position.x = 0.166
    l_marker.pose.position.y = - finger_distance / 2.0
    l_marker.pose.orientation.w = 1
    l_marker.type = Marker.MESH_RESOURCE
    l_marker.mesh_resource = L_FINGER_MESH
    l_marker.mesh_use_embedded_materials = True

    # right finger marker
    r_marker = Marker()
    r_marker.pose.position.x = 0.166
    r_marker.pose.position.y = finger_distance / 2.0
    r_marker.pose.orientation.w = 1
    r_marker.type = Marker.MESH_RESOURCE
    r_marker.mesh_resource = R_FINGER_MESH
    r_marker.mesh_use_embedded_materials = True

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.interaction_mode = InteractiveMarkerControl.NONE
    control.always_visible = True
    control.markers.append(gripper_marker)
    control.markers.append(l_marker)
    control.markers.append(r_marker)

    interactive_marker = InteractiveMarker()
    interactive_marker.header = pose_stamped.header
    interactive_marker.pose = pose_stamped.pose
    interactive_marker.controls.append(control)
    interactive_marker.scale = 0.3

    return interactive_marker

def six_dof_controls():
    """Returns a list of 6 InteractiveMarkerControls
    """
    controls = []
    # Create 6 DOF controls
    rx_control = InteractiveMarkerControl()
    rx_control.orientation.w = 1
    rx_control.orientation.x = 1
    rx_control.orientation.y = 0
    rx_control.orientation.z = 0
    rx_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rx_control.name = 'rotate_x'
    controls.append(rx_control)

    mx_control = InteractiveMarkerControl()
    mx_control.orientation.w = 1
    mx_control.orientation.x = 1
    mx_control.orientation.y = 0
    mx_control.orientation.z = 0
    mx_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    mx_control.name = 'move_x'
    controls.append(mx_control)

    rz_control = InteractiveMarkerControl()
    rz_control.orientation.w = 1
    rz_control.orientation.x = 0
    rz_control.orientation.y = 1
    rz_control.orientation.z = 0
    rz_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rz_control.name = 'rotate_z'
    controls.append(rz_control)

    mz_control = InteractiveMarkerControl()
    mz_control.orientation.w = 1
    mz_control.orientation.x = 0
    mz_control.orientation.y = 1
    mz_control.orientation.z = 0
    mz_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    mz_control.name = 'move_z'
    controls.append(mz_control)

    ry_control = InteractiveMarkerControl()
    ry_control.orientation.w = 1
    ry_control.orientation.x = 0
    ry_control.orientation.y = 0
    ry_control.orientation.z = 1
    ry_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    ry_control.name = 'rotate_y'
    controls.append(ry_control)

    my_control = InteractiveMarkerControl()
    my_control.orientation.w = 1
    my_control.orientation.x = 0
    my_control.orientation.y = 0
    my_control.orientation.z = 1
    my_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    my_control.name = 'move_y'
    controls.append(my_control)

    return controls

def menu_entry(id, title):
    menu_entry = MenuEntry()
    menu_entry.command_type = MenuEntry.FEEDBACK
    menu_entry.id = id
    menu_entry.title = title
    return menu_entry

def color_gripper(gripper_im, r, g, b, a):
    for marker in gripper_im.controls[0].markers:
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        # initial gripper pose
        ps = PoseStamped()
        ps.header.frame_id = 'base_link'
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0
        pose.position.z = 0.5
        pose.orientation.w = 1
        ps.pose = pose
        gripper_im = interactive_gripper_marker(ps, OPEN_FINGER_POS)
        gripper_im.name = 'gripper'
        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)

        # add menu
        gripper_im.controls[0].interaction_mode = InteractiveMarkerControl.MENU
        gripper_im.menu_entries.append(menu_entry(1, 'Move gripper here'))
        gripper_im.menu_entries.append(menu_entry(2, 'Open gripper'))
        gripper_im.menu_entries.append(menu_entry(3, 'Close gripper'))

        # add 6dof
        gripper_im.controls.extend(six_dof_controls())

        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

        # check ik
        self.check_ik(ps)

    def check_ik(self, pose_stamped):
        gripper_im = self._im_server.get('gripper')
        joints = self._arm.compute_ik(pose_stamped)
        if joints == False:
            color_gripper(gripper_im, 1, 0, 0, 1)
        else:
            color_gripper(gripper_im, 0, 1, 0, 1)
        self._im_server.insert(gripper_im)
        self._im_server.applyChanges()

    def handle_feedback(self, feedback):
        if feedback.marker_name != 'gripper':
            return

        # get current gripper pose
        gripper_im = self._im_server.get('gripper')
        ps = PoseStamped()
        ps.header = gripper_im.header
        ps.pose = gripper_im.pose

        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 1: # move gripper
                error = self._arm.move_to_pose(ps)
                if error is not None:
                    rospy.logerr(error)
            elif feedback.menu_entry_id == 2: # open gripper
                self._gripper.open()
            elif feedback.menu_entry_id == 3: # close gripper
                self._gripper.close()
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.check_ik(ps)


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

# pose1 -> pose2
# def copy_orientation(pose1, pose2):
#     pose2.orientation.x = pose1.orientation.x
#     pose2.orientation.y = pose1.orientation.y
#     pose2.orientation.z = pose1.orientation.z
#     pose2.orientation.w = pose1.orientation.w

class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        # create blue cube marker
        obj_marker = Marker()
        obj_marker.type = Marker.CUBE
        obj_marker.pose.orientation.w = 1
        obj_marker.scale.x = 0.06
        obj_marker.scale.y = 0.06
        obj_marker.scale.z = 0.06
        obj_marker.color.r = 9
        obj_marker.color.g = 97
        obj_marker.color.b = 239
        obj_marker.color.a = 1.0

        obj_control = InteractiveMarkerControl()
        obj_control.interaction_mode = InteractiveMarkerControl.MENU
        obj_control.always_visible = True
        obj_control.markers.append(obj_marker)

        obj_im = InteractiveMarker()
        obj_im.name = 'object'
        obj_im.header.frame_id = 'base_link'
        obj_im.pose.position.x = 0.7
        obj_im.pose.position.y = 0
        obj_im.pose.position.z = 0.5
        obj_im.pose.orientation.w = 1
        obj_im.scale = 0.3
        obj_im.menu_entries.append(menu_entry(1, 'Pick from front'))
        obj_im.menu_entries.append(menu_entry(2, 'Open gripper'))
        obj_im.controls.append(obj_control)
        obj_im.controls.extend(six_dof_controls())

        self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

        self.update_gripper()

    def update_gripper(self):
        obj_im = self._im_server.get('object')
        obj_ps = PoseStamped()
        obj_ps.header = obj_im.header
        obj_ps.pose = obj_im.pose


        pregrasp_ps = copy.deepcopy(obj_ps)
        pregrasp_ps.pose.x = obj_ps.pose.x - 0.3

        grasp_ps = copy.deepcopy(pregrasp_ps)
        grasp_ps.pose.x = pregrasp_ps.pose.x + 0.15

        lift_ps = copy.deepcopy(grasp_ps)
        lift_ps.pose.x = grasp_ps.pose.z + 0.3


    def handle_feedback(self, feedback):
        pass

def main():
    rospy.init_node('gripper_teleop_node')

    # raise torso
    torso = robot_api.Torso()
    torso.set_height(0.4)
    # title head to look at the ground
    head = robot_api.Head()
    head.pan_tilt(0, math.pi/8)

    arm = robot_api.Arm()
    gripper = robot_api.Gripper()

    # im_server = InteractiveMarkerServer('gripper_im_server')
    # teleop = GripperTeleop(arm, gripper, im_server)
    # teleop.start()

    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server')
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    auto_pick.start()

    rospy.spin()


if __name__ == '__main__':
  main()
