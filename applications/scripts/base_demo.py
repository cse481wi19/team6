#! /usr/bin/env python

import math
import robot_api
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

def handle_f_input(input):
    base = robot_api.Base()
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        base.go_forward(0.5)

def handle_cw_input(input):
    base = robot_api.Base()
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        base.turn(-30 * math.pi / 180)

def handle_ccw_input(input):
    base = robot_api.Base()
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        base.turn(30 * math.pi / 180)

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def print_usage():
    print 'Usage: rosrun applications base_demo.py move 0.1'
    print '       rosrun applications base_demo.py rotate 30'


def main():
    rospy.init_node('base_demo')
    wait_for_time()
    server = InteractiveMarkerServer("drive_marker")

    int_f_marker = InteractiveMarker()
    int_f_marker.header.frame_id = "base_link"
    int_f_marker.name = "foward_marker"
    int_f_marker.description = "Forward Control"
    int_f_marker.pose.position.x = 2
    int_f_marker.pose.orientation.w = 1

    f_marker = Marker()
    f_marker.type = Marker.CUBE
    f_marker.pose.orientation.w = 2
    f_marker.scale.x = 0.45
    f_marker.scale.y = 0.45
    f_marker.scale.z = 0.45
    f_marker.color.r = 0.0
    f_marker.color.g = 0.5
    f_marker.color.b = 0.5
    f_marker.color.a = 1.0

    int_cw_marker = InteractiveMarker()
    int_cw_marker.header.frame_id = "base_link"
    int_cw_marker.name = "cw_marker"
    int_cw_marker.description = "Clockwise Turn Control"
    int_cw_marker.pose.position.y = -2
    int_cw_marker.pose.orientation.w = 1

    f_button_control = InteractiveMarkerControl()
    f_button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    f_button_control.always_visible = True
    f_button_control.markers.append(f_marker)
    int_f_marker.controls.append(f_button_control)

    cw_marker = Marker()
    cw_marker.type = Marker.SPHERE
    cw_marker.pose.orientation.w = 2
    cw_marker.scale.x = 0.45
    cw_marker.scale.y = 0.45
    cw_marker.scale.z = 0.45
    cw_marker.color.r = 0.0
    cw_marker.color.g = 0.5
    cw_marker.color.b = 0.5
    cw_marker.color.a = 1.0

    cw_button_control = InteractiveMarkerControl()
    cw_button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    cw_button_control.always_visible = True
    cw_button_control.markers.append(cw_marker)
    int_cw_marker.controls.append(cw_button_control)

    int_ccw_marker = InteractiveMarker()
    int_ccw_marker.header.frame_id = "base_link"
    int_ccw_marker.name = "ccw_marker"
    int_ccw_marker.description = "Counter-Clockwise Turn Control"
    int_ccw_marker.pose.position.y = 2
    int_ccw_marker.pose.orientation.w = 1

    ccw_marker = Marker()
    ccw_marker.type = Marker.SPHERE
    ccw_marker.pose.orientation.w = 1
    ccw_marker.scale.x = 0.45
    ccw_marker.scale.y = 0.45
    ccw_marker.scale.z = 0.45
    ccw_marker.color.r = 0.5
    ccw_marker.color.g = 0.0
    ccw_marker.color.b = 0.5
    ccw_marker.color.a = 1.0

    ccw_button_control = InteractiveMarkerControl()
    ccw_button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    ccw_button_control.always_visible = True
    ccw_button_control.markers.append(ccw_marker)
    int_ccw_marker.controls.append(ccw_button_control)

    server.insert(int_f_marker, handle_f_input)
    server.insert(int_cw_marker, handle_cw_input)
    server.insert(int_ccw_marker, handle_ccw_input)
    rospy.sleep(0.5)
    server.applyChanges()
    rospy.spin()
    # argv = rospy.myargv()
    # if len(argv) < 3:
    #     print_usage()
    #     return
    # command = argv[1]
    # value = float(argv[2])
    #
    #
    # if command == 'move':
    #     base.go_forward(value)
    # elif command == 'rotate':
    #     base.turn(value * math.pi / 180)
    # else:
    #     print_usage()


if __name__ == '__main__':
    main()
