#!/usr/bin/env python

import rospy

import robot_api
from map_annotator import Navigator
from robot_api import ArmMotionPlanner

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

# Object detection
class ObjectMarkerReader(object):
    def __init__(self):
        self.looking = False
        self.object_marker = None

    def callback(self, msg):
        # Process markers only if still looking for objects
        # Ignore the msg if not looking for objects
        if self.looking and msg.type == Marker.CUBE:
            self.object_marker = msg
            self.looking = False

    def start_looking(self):
        self.looking = True

# Face detection
class FaceDetector(object):
    def __init__(self):
        self.looking = False
        self.face_location = None

    def callback(self, msg):
        # Process markers only if still looking for faces
        # Ignore the msg if not looking for faces
        if self.looking:
            self.face_location = msg
            self.looking = False

    def start_looking(self):
        self.looking = True


def main():
    print("Initializing...")
    rospy.init_node('project_master_node')
    wait_for_time()

    found_object = False
    picked_object = False
    face_detected = False

    print("Initializing head...")
    head = robot_api.Head()

    print("Initializing gripper...")
    gripper = robot_api.Gripper()

    print("Initializing torso...")
    torso = robot_api.Torso()
    torso.set_height(0.1)

    print("Initializing arm motion planner...")
    # arm = robot_api.Arm()
    # arm_planner = ArmMotionPlanner(arm)
    arm_planner = None

    print("Initializing object detector...")
    object_reader = ObjectMarkerReader()
    object_marker_sub = rospy.Subscriber('object_markers', Marker, callback=object_reader.callback)

    print("Initializing face detector...")
    face_detector = FaceDetector()
    face_marker_sub = rospy.Subscriber('face_locations', Marker, callback=face_detector.callback)

    print("Initializing navigation...")
    navigator = Navigator()

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
            object_reader.start_looking()
            rate.sleep()

            if object_reader.object_marker is not None:
                found_object = True
                print("Found object")
            else:
                print("Could not find object. Moving around...")
                # TODO: Move around randomly
                # navigator.goto(?, 0.5)
                rate.sleep()
                print("Retrying...")

        # Part 2
        # Pickup object
        #   If success, go to next step
        #   If failure, go to Part 1
        while found_object and not picked_object:
            picked_object = arm_planner.pick_up(object_reader.object_marker)
            if not picked_object:
                print("Could not pick the object. Retrying...")
                found_object = False
            else:
                print("Picked object")

        # Part 3
        # Look for face
        #   If found, move to face location
        #   If not found, roam around and repeat
        while found_object and picked_object and not face_detected:
            print("Looking for people...")
            head.pan_tilt(0.0, 0.0)
            face_detector.start_looking()
            rate.sleep()

            if face_detector.face_marker is not None:
                print("Found person")
                print("Moving to person...")
                face_detected = navigator.goto(face_detector.face_location, 0.5)
            if not face_detected:
                print("Could not reach the person. Retrying...")
                # TODO: Move around randomly
                # navigator.goto(?, 0.5)
                rate.sleep()

        # Part 4
        # Wait for 5 seconds at face location and open the gripper
        # Go to Part 1
        if found_object and picked_object and face_detected:
            print("Delivering object...")
            rospy.sleep(5)
            gripper.open()
            found_object = False
            picked_object = False
            face_detected = False
            print("Object delivered")


if __name__ == '__main__':
    main()
