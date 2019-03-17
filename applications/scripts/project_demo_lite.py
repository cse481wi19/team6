#!/usr/bin/env python
import copy
import math
import random
import time

import rospy
import robot_api
from map_annotator import Navigator
from robot_api import ArmMotionPlanner

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

MOVE_DISTANCE = 0.4
MOVE_TIMEOUT = 10

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def get_distance(msg):
    x = msg.pose.position.x ** 2
    y = msg.pose.position.y ** 2
    return (x + y) ** (0.5)


# Object detection
class ObjectMarkerReader(object):
    def __init__(self):
        self.object_markers = {}

    def callback(self, msg):
        # Process markers only if still looking for objects
        # Ignore the msg if not looking for objects
        if msg.type == Marker.CUBE:
            if msg.action == Marker.DELETE:
                self.object_markers.pop(msg.id, None)
            elif msg.action == Marker.ADD:
                self.object_markers[msg.id] = msg
            else:
                print("Unidentified marker action...", msg.action)
            # self.object_markers[msg.id] = msg

    def get_object(self, id=None):
        if id is not None:
            return self.object_markers.get(id, None)

        if len(self.object_markers) <= 0:
            return None

        # sort markers based on distance to robot
        sorted_markers = sorted(self.object_markers.values(), key=lambda marker: marker.pose.position.x)
        return sorted_markers[0]

    def object_detected(self):
        return len(self.object_markers) > 0


# Face detection
class FaceDetector(object):
    def __init__(self):
        self.face_marker = None

    def callback(self, msg):
        # self.face_location = PoseStamped()
        # self.face_location.header = msg.header
        # self.face_location.pose = msg.pose

        if msg.pose.position.x == 0 and msg.pose.position.y == 0 and msg.pose.position.z == 0:
            # No face detected
            self.face_marker = None
        else:
            self.face_marker = msg

    def get_face(self):
        return self.face_marker

    def face_detected(self):
        return self.face_marker is not None


def move_random(navigator, base):
    moved = False

    turn = [0, math.pi / 2, - math.pi]
    try_turn = 0
    rate = rospy.Rate(1)
    while not moved:
        base.turn(turn[try_turn])
        if try_turn is not 0:
            break
        cur_pose = navigator.current_pose()
        pose_baselink = navigator.transform(cur_pose, 'base_link')

        if pose_baselink is not None:
            target_pose = copy.deepcopy(pose_baselink)
            target_pose.pose.position.x += MOVE_DISTANCE
            moved = navigator.goto(target_pose, MOVE_TIMEOUT)

            if not moved:
                try_turn = (try_turn + 1) % len(turn)
        rate.sleep()
        time.sleep(0.5)


def main():
    print("Initializing...")
    rospy.init_node('project_master_node')
    wait_for_time()

    found_object = False
    picked_object = False
    face_detected = False

    print("Initializing base...")
    base = robot_api.Base()

    print("Initializing head...")
    head = robot_api.Head()

    print("Initializing gripper...")
    gripper = robot_api.Gripper()
    gripper.open()

    print("Initializing torso...")
    torso = robot_api.Torso()
    torso.set_height(0)

    print("Initializing arm motion planner...")
    arm = robot_api.Arm()
    arm.move_to_initial_pose()
    arm_planner = ArmMotionPlanner(arm, gripper)

    print("Initializing object detector...")
    object_reader = ObjectMarkerReader()
    object_marker_sub = rospy.Subscriber('object_markers', Marker, callback=object_reader.callback)

    print("Initializing face detector...")
    face_detector = FaceDetector()
    face_marker_sub = rospy.Subscriber('face_marker', Marker, callback=face_detector.callback)

    print("Initializing navigation...")
    navigator = Navigator()

    print("Initialized")

    while not rospy.is_shutdown():
        # Part 1
        # Look for objects
        #   If not found, turn around and repeat
        head.pan_tilt(0.0, 0.85)
        rospy.sleep(1)  # wait extra time after tilting for the first point cloud to arrive
        # #
        head.pan_tilt(0.0, 0.9)
        TURN_STEPS = 10
        angular_distance = 2*math.pi / TURN_STEPS
        object_marker = None
        while True:
            print("Looking for objects...")
            rospy.sleep(8)
            # time.sleep(2)
            if object_reader.object_detected():
                print("Found object")
                object_marker = object_reader.get_object()
                break

            print("Could not find object. turning around...")
            base.turn(angular_distance)
            print("Retrying...")

        # Part 2
        # Attempt to pickup object
        #       If success, go to next step
        #       If failure, restart from Part 1
        if object_marker is None:
            print("ERROR: Detected Object Gone...")
            return

        arm.move_to_hold_pose()  # synchronized
        picked_object = arm_planner.pick_up(object_marker)
        if not picked_object:
            print("Could not pick the object. Restart from looking objects...")
            continue

        print("Picked object")
        arm.move_to_hold_pose()

        # Part 3
        # Look for face
        #   If found, move to face location
        #   If not found, turn around and repeat
        head.pan_tilt(0.0, 0.0)
        face_detected = False
        face_location = None
        max_dist = 2.5
        while True:
            print("Looking for people...")
            rospy.sleep(5)

            if face_detector.face_detected():
                print("Found person")
                face_marker = face_detector.get_face()
                face_location = transform_marker(navigator, face_marker)
                if face_location.pose.position.x < max_dist:
                    break
                print("Person out of range...")
            else:
                print("Could not find face...")

            print("Turning around...")
            base.turn(angular_distance)
            print("Retrying...")


        # Part 4
        # Go to the face location
        # Wait for 5 seconds and open the gripper
        # Go to Part 1
        while True:
            print("Moving to person...")
            reached = go_forward(navigator, base, face_location)
            # reached = go_to(navigator, face_location)
            if not reached:
                print("Could not reach the face. Retrying...")
                # face_location.pose.position.x -= 0.05
                # print("updated face_location_baselink x:", face_location.pose.position.x)
                continue

            print("Delivering object...")
            time.sleep(5)
            gripper.open()
            print("Object delivered")
            # arm.move_to_initial_pose()

            print("Demo lite round 1 complete...")
            return


# Transforms marker to a PoseStamped in base link
def transform_marker(navigator, marker):
    rate = rospy.Rate(2)
    target_pose = PoseStamped()
    target_pose.header = marker.header
    target_pose.pose = marker.pose

    target_pose_baselink = None
    while target_pose_baselink is None:
        target_pose_baselink = navigator.transform(target_pose, 'base_link')
        rate.sleep()

    print("target_pose_baselink x: ", target_pose_baselink.pose.position.x)
    print("target_pose_baselink y: ", target_pose_baselink.pose.position.y)

    return target_pose_baselink


def go_forward(navigator, base, location_pose, offset=0.9):
    rate = rospy.Rate(2)
    cur_pose = navigator.current_pose()
    # print("cur_pose", cur_pose)
    cur_pose_baselink = None
    while cur_pose_baselink is None:
        cur_pose_baselink = navigator.transform(cur_pose, 'base_link')
        rate.sleep()
    print("cur_pose_baselink", cur_pose_baselink)
    # target_pose = copy.deepcopy(pose_baselink)
    dist = location_pose.pose.position.x - offset - cur_pose_baselink.pose.position.x
    dist = max(0, dist)
    base.go_forward(dist)
    return True

def go_to(navigator, location_pose, offset=0.8, timeout=10):
    rate = rospy.Rate(2)
    cur_pose = navigator.current_pose()
    print("cur_pose", cur_pose)
    curr_pose_baselink = None
    while curr_pose_baselink is None:
        curr_pose_baselink = navigator.transform(cur_pose, 'base_link')
        rate.sleep()

    print("curr_pose_baselink", curr_pose_baselink)
    # target_pose = copy.deepcopy(pose_baselink)
    goto_dist = location_pose.pose.position.x - offset
    print("goto_dist", goto_dist)
    curr_pose_baselink.pose.position.x = max(0, goto_dist)
    curr_pose_baselink.pose.position.y = location_pose.pose.position.y
    print("pose_baselink", curr_pose_baselink)
    # target_pose.pose.position.x += dist
    return navigator.goto(curr_pose_baselink, timeout)


    # target_pose = copy.deepcopy(target_pose_baselink)
    # # print("target_pose_start", target_pose.pose.position.x)
    #
    # current_pose_baselink = None
    # while current_pose_baselink is None:
    #     current_pose_baselink = navigator.transform(navigator.current_pose(), 'base_link')
    #     rate.sleep()
    # print("current_pose_baselink x: ", current_pose_baselink.pose.position.x)
    #
    # goto_pose_baselink = copy.deepcopy(current_pose_baselink)
    # move_dist_x = max(0, target_pose.pose.position.x - offset)
    # goto_pose_baselink.pose.position.x += move_dist_x
    # # move_dist_y = max(0, target_pose.pose.position.y - current_pose_baselink.pose.position.y)
    # # goto_pose_baselink.pose.position.y += move_dist_y
    # # target_pose.pose.orientation = current_pose_baselink.pose.orientation
    # # current_position_baselink =
    # # target_pose.pose.position.x = max(current_position_baselink.x, target_pose.pose.position.x - 0.55)
    #
    # if goto_pose_baselink.pose.position.x == current_pose_baselink.pose.position.x:
    #     return None
    # # target_pose.pose.position.y = curr_position_baselink.y
    # # target_pose.pose.position.z = current_position_baselink.z
    #
    # print("goto_pose_baselink x: ", goto_pose_baselink.pose.position.x)
    # print("goto_pose_baselink y: ", goto_pose_baselink.pose.position.y)
    #
    # return goto_pose_baselink  # in base_link


if __name__ == '__main__':
    main()
