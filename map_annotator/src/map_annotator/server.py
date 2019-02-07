#!/usr/bin/env python

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from map_annotator.msg import PoseNames, UserAction
# from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry
import pickle
import os

def wait_for_time():
    """
    Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class MarkerManager(object):
    def __init__(self, annotator, int_marker_server):
        self._annotator = annotator
        self._int_marker_server = int_marker_server
        for name in self._annotator.list():
            pose = self._annotator.get(name)
            int_marker = self._marker_style(name, pose)
            self._int_marker_server.insert(int_marker)
            self._int_marker_server.setCallback(name, self.change_pose_callback)
        self._int_marker_server.applyChanges()


    def create(self, name):
        pose = Pose()
        pose.position.z = 0.1
        pose.orientation.w = 1.0
        int_marker = self._marker_style(name, pose)
        self._int_marker_server.insert(int_marker)
        self._int_marker_server.setCallback(name, self.change_pose_callback)
        self._int_marker_server.applyChanges()

    def delete(self, name):
        self._int_marker_server.erase(name)
        self._int_marker_server.applyChanges()

    def get(self, name):
        return self._int_marker_server.get(name)

    def _marker_style(self, name, pose):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'map'
        int_marker.name = name
        int_marker.description = name
        int_marker.pose = pose

        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.pose.orientation.w = 1
        arrow_marker.scale.x = 0.5
        arrow_marker.scale.y = 0.07
        arrow_marker.scale.z = 0.05
        arrow_marker.color.r = 0.5
        arrow_marker.color.g = 0.5
        arrow_marker.color.b = 0.5
        arrow_marker.color.a = 1.0

        # text_marker = Marker()
        # text_marker.type = Marker.TEXT_VIEW_FACING
        # text_marker.pose.orientation.w = 1
        # text_marker.pose.position.z = 1.5
        # text_marker.scale.x = 0.2
        # text_marker.scale.y = 0.2
        # text_marker.scale.z = 0.2
        # text_marker.color.r = 0.5
        # text_marker.color.g = 0.5
        # text_marker.color.b = 0.5
        # text_marker.color.a = 1
        # text_marker.text = name

        arrow_control = InteractiveMarkerControl()
        arrow_control.orientation.w = 1
        arrow_control.orientation.y = 1
        arrow_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        arrow_control.markers.append(arrow_marker)
        # arrow_control.markers.append(text_marker)
        arrow_control.always_visible = True
        int_marker.controls.append(arrow_control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.y = 1
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        return int_marker

    def change_pose_callback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self._annotator.save(feedback.marker_name, feedback.pose)
            self._int_marker_server.setPose(feedback.marker_name, feedback.pose)
            self._int_marker_server.applyChanges()

class MarkerMapAnnotator(object):
    POSE_DICT = '/home/team6/catkin_ws/src/cse481wi19/map_annotator/pose_dict/pose_dict'

    def __init__(self):
        self.pub =  rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=5)
        self.pose_dict = self.load_pose_dict()

    def load_pose_dict(self):
        with open(MarkerMapAnnotator.POSE_DICT, 'rb') as f:
            return pickle.load(f)

    def dump_pose_dict(self):
        with open(MarkerMapAnnotator.POSE_DICT, 'wb') as f:
            pickle.dump(self.pose_dict, f)

    # List saved poses.
    def list(self):
        return self.pose_dict.keys()

    # save <name>: Save the robot\'s current pose as <name>. Overwrites if <name> already exists.
    def save(self, name, pose):
        self.pose_dict[name] = pose
        self.dump_pose_dict()

    # delete <name>: Delete the pose given by <name>.
    def delete(self, name):
        self.pose_dict.pop(name, None)
        self.dump_pose_dict()

    # get the pose given by name
    def get(self, name):
        return self.pose_dict.get(name, None)

    # goto <name>: Sends the robot to the pose given by <name>
    def goto(self, name):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time().now()
        goal.pose = self.pose_dict[name]
        self.pub.publish(goal)


class Server(object):
    def __init__(self, annotator, pose_pub, marker_manager):
        self.annotator = annotator
        self.pose_pub = pose_pub
        self.marker_manager = marker_manager


    def pub_pose(self):
        pose_names = PoseNames()
        pose_names.names.extend(self.annotator.list())
        self.pose_pub.publish(pose_names)

    def user_actions_callback(self, action):
        if action.command == UserAction.CREATE:
            self.marker_manager.create(action.name)
            pose = self.marker_manager.get(action.name).pose
            self.annotator.save(action.name, pose)
            self.pub_pose()
        elif action.command == UserAction.DELETE:
            self.marker_manager.delete(action.name)
            self.annotator.delete(action.name)
            self.pub_pose()
        elif action.command == UserAction.GOTO:
            self.annotator.goto(action.name)
        else:
            rospy.logwarn("Not such command")

def main():
    rospy.init_node('map_annotate_server')
    wait_for_time()

    annotator = MarkerMapAnnotator()
    int_marker_server = InteractiveMarkerServer('map_annotator/map_poses')
    marker_manager = MarkerManager(annotator, int_marker_server)
    pose_pub = rospy.Publisher('map_annotator/pose_names', PoseNames, latch=True, queue_size=1)
    server = Server(annotator, pose_pub, marker_manager)
    sub = rospy.Subscriber('map_annotator/user_actions', UserAction, server.user_actions_callback)

    rospy.sleep(0.5)
    def handle_shutdown():
        pn = PoseNames()
        pose_pub.publish(pn)

    rospy.on_shutdown(handle_shutdown)
    rospy.spin()
    # rate = rospy.Rate(10)

if __name__ == '__main__':
    main()
