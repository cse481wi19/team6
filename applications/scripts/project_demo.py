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
        # sort markers based on distance to robot
        if id is not None:
            return self.object_markers.get(id, None)

        sorted_markers = sorted(self.object_markers.values(), key=lambda x: x.pose.position.x)
        if len(sorted_markers) == 0:
            return None
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

        if msg.position.x == 0 and msg.position.y == 0 and msg.position.z == 0:
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

    rate = rospy.Rate(0.5)

    print("Initialized")
    while not rospy.is_shutdown():
        # Part 1
        # Look for objects
        #   If found, move to object location
        #   If not found, roam around and repeat
        while not found_object:
            print("Looking for objects...")
            head.pan_tilt(0.0, 1.0)
            time.sleep(10)
            rate.sleep()

            if object_reader.object_detected():
                found_object = True
                print("Found object")
            else:
                print("Could not find object. Moving around...")
                move_random(navigator, base)
                time.sleep(2)
                rate.sleep()
                print("Retrying...")

            time.sleep(3)
            rate.sleep()

        # Part 2
        # Pickup object
        #   Goto the object location and attempt to pick it up
        #       If success, go to next step
        #       If failure, go to Part 1
        while found_object and not picked_object:
            object = object_reader.get_object()
            if object is None:
                print("Detected Object Gone...")
                found_object = False
                arm.move_to_initial_pose()
                continue
            target_pose = transform_marker(navigator, object)
            reached = True
            if target_pose == None:
                reached = True
            else:
                reached = navigator.goto(target_pose, MOVE_TIMEOUT)


            if not reached:
                print("Could not reach the object. Retrying...")
                continue
            else:
                head.pan_tilt(0.0, 1.0)
                time.sleep(10)
                rate.sleep()
                object_to_pick = object_reader.get_object(object.id)
                if object_to_pick is None:
                    print("Object Gone...")
                    found_object = False
                    arm.move_to_initial_pose()
                    continue
                print(object_to_pick)
                arm.move_to_hold_pose()
                picked_object = arm_planner.pick_up(object_to_pick)
            if not picked_object:
                print("Could not pick the object. Retrying...")
                found_object = False
                arm.move_to_initial_pose()
            else:
                print("Picked object")
                arm.move_to_hold_pose()



        # Part 3
        # Look for face
        #   If found, move to face location
        #   If not found, roam around and repeat
        while found_object and picked_object and not face_detected:
            print("Looking for people...")
            head.pan_tilt(0.0, 0.0)
            time.sleep(3)

            if face_detector.face_detected():
                face_detected = True
                print("Found person")
            else:
                print("Could not find face. Moving around...")
                move_random(navigator, base)
                time.sleep(2)
                rate.sleep()
                print("Retrying...")

        # Part 4
        # Go to the face location
        # Wait for 5 seconds and open the gripper
        # Go to Part 1
        if found_object and picked_object and face_detected:
            print("Moving to person...")
            face_location = transform_marker(navigator, face_detector.get_face())
            face_detected = navigator.goto(face_location, MOVE_TIMEOUT)

            reached = navigator.goto(face_location, MOVE_TIMEOUT)
            if not reached:
                print("Count not reach the face. Retrying...")
                continue
            else:
                print("Delivering object...")
                time.sleep(3)
                gripper.open()
                found_object = False
                picked_object = False
                face_detected = False
                print("Object delivered")
                arm.move_to_initial_pose()

        rate.sleep()


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
    target_pose = copy.deepcopy(target_pose_baselink)
    print("target_pose_start", target_pose.pose.position.x)

    current_pose_baselink = None
    while current_pose_baselink is None:
        current_pose_baselink = navigator.transform(navigator.current_pose(), 'base_link')
        rate.sleep()
    print("current_pose_baselink", current_pose_baselink.pose.position.x)

    final_target_pose = copy.deepcopy(current_pose_baselink)
    move_dist_x = max(0, target_pose.pose.position.x - 0.55)
    final_target_pose.pose.position.x += move_dist_x
    move_dist_y = max(0, target_pose.pose.position.y - current_pose_baselink.pose.position.y)
    final_target_pose.pose.position.y += move_dist_y
    # target_pose.pose.orientation = current_pose_baselink.pose.orientation
    current_position_baselink = current_pose_baselink.pose.position
    # target_pose.pose.position.x = max(current_position_baselink.x, target_pose.pose.position.x - 0.55)

    print("target_pose_final", final_target_pose.pose.position.x)
    if final_target_pose.pose.position.x == current_position_baselink.x:
        return None
    # target_pose.pose.position.y = curr_position_baselink.y
    # target_pose.pose.position.z = current_position_baselink.z

    return final_target_pose


if __name__ == '__main__':
    main()
