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

MOVE_DISTANCE = 0.5
MOVE_TIMEOUT = 10

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

# Object detection
class ObjectMarkerReader(object):
    def __init__(self):
        self.object_id = set()
        self.object_marker = []

    def callback(self, msg):
        # Process markers only if still looking for objects
        # Ignore the msg if not looking for objects
        if msg.type == Marker.CUBE:
            if msg.id not in self.object_id:
                self.object_id.add(msg.id)
                self.object_marker.append((msg, 0))

    def report_fail(self):
        num_tries = self.object_marker[0][1] + 1
        if num_tries < 3:
            self.object_marker[0] = (self.object_marker[0][0], self.object_marker[0][1])
        else:
            self.object_marker.pop(0)

    def report_success(self):
        self.object_marker.pop(0)

    def object_detected(self):
        return False and len(self.object_marker) > 0

# Face detection
class FaceDetector(object):
    def __init__(self):
        self.looking = False
        self.face_location = None

    def callback(self, msg):
        # Process markers only if still looking for faces
        # Ignore the msg if not looking for faces
        if self.looking:
            self.face_location = PoseStamped()
            self.face_location.header = msg.header
            self.face_location.pose = msg.pose

            self.looking = False

    def start_looking(self):
        self.face_location = None
        self.looking = True

def move_random(navigator, base):
    moved = False

    turn = [0, math.pi / 2, - math.pi]
    try_turn = 0
    while not moved:
        base.turn(turn[try_turn])
        cur_pose = navigator.current_pose()
        pose_baselink = navigator.transform(cur_pose, 'base_link')

        target_pose = copy.deepcopy(pose_baselink)
        target_pose.pose.position.x += MOVE_DISTANCE

        moved = navigator.goto(target_pose, MOVE_TIMEOUT)
        time.sleep(3)
        try_turn = (try_turn + 1) % len(turn)

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

    print("Initializing torso...")
    torso = robot_api.Torso()
    torso.set_height(0.1)

    print("Initializing arm motion planner...")
    arm = robot_api.Arm()
    arm.move_to_initial_pose()
    arm_planner = ArmMotionPlanner(arm, gripper)

    print("Initializing object detector...")
    object_reader = ObjectMarkerReader()
    object_marker_sub = rospy.Subscriber('object_markers', Marker, callback=object_reader.callback)

    print("Initializing face detector...")
    face_detector = FaceDetector()
    face_marker_sub = rospy.Subscriber('face_locations', PoseStamped, callback=face_detector.callback)

    print("Initializing navigation...")
    navigator = Navigator()

    rospy.sleep(1)

    rate = rospy.Rate(10)

    print("Initialized")
    while not rospy.is_shutdown():
        # Part 1
        # Look for objects
        #   If found, move to object location
        #   If not found, roam around and repeat
        while not found_object:
            print("Looking for objects...")
            head.pan_tilt(0.0, 0.8)
            time.sleep(3)

            if object_reader.object_detected():
                found_object = True
                print("Found object")
            else:
                print("Could not find object. Moving around...")
                move_random(navigator, base)
                time.sleep(3)
                rate.sleep()
                print("Retrying...")

        # Part 2
        # Pickup object
        #   If success, go to next step
        #   If failure, go to Part 1
        while found_object and not picked_object:
            target_pose = PoseStamped()
            target_pose.header = object_reader.object_marker[0][0].header
            target_pose.pose = object_reader.object_marker[0][0].pose

            pose_baselink = navigator.transform(target_pose, 'base_link')
            target_pose = copy.deepcopy(pose_baselink)
            target_pose.pose.position.x = max(0, target_pose.pose.position.x - 0.3)
            target_pose.pose.position.z = 0
            target_pose.pose.orientation = navigator.current_pose().pose.orientation
            print(target_pose)

            reached = navigator.goto(target_pose, MOVE_TIMEOUT)
            if reached:
                picked_object = arm_planner.pick_up(object_reader.object_marker[0][0])

            if not picked_object:
                print("Could not pick the object. Retrying...")
                found_object = False
                object_reader.report_fail()
            else:
                print("Picked object")
                object_reader.report_success()

            arm.move_to_initial_pose()

        # Part 3
        # Look for face
        #   If found, move to face location
        #   If not found, roam around and repeat
        while found_object and picked_object and not face_detected:
            print("Looking for people...")
            head.pan_tilt(0.0, 0.0)
            face_detector.start_looking()
            time.sleep(3)

            if face_detector.face_marker is not None:
                print("Found person")
                print("Moving to person...")
                face_detected = navigator.goto(face_detector.face_location, MOVE_TIMEOUT)
            if not face_detected:
                print("Could not reach the person. Retrying...")
                move_random(navigator, base)
                time.sleep(3)

        # Part 4
        # Wait for 5 seconds at face location and open the gripper
        # Go to Part 1
        if found_object and picked_object and face_detected:
            print("Delivering object...")
            time.sleep(3)
            gripper.open()
            found_object = False
            picked_object = False
            face_detected = False
            print("Object delivered")

        rate.sleep()


if __name__ == '__main__':
    main()
