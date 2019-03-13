#! /usr/bin/env python

import robot_api
from map_annotator import Navigator
from geometry_msgs.msg import PoseStamped
import rospy
import sys

def wait_for_time():
    """
    Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


direction = []
def main():
    rospy.init_node('navigator_demo')
    wait_for_time()
    rospy.sleep(0.5)

    navigator = Navigator()
    rate = rospy.Rate(10)
    while True :
        pose = navigator.current_pose()

        if pose is not None:
            # pose.pose.orientation.w = -1
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = 2
            goal.pose.position.y = 3
            goal.pose.position.z = 0
            # pose.pose.orientation.x = 0
            # pose.pose.orientation.y = 0
            # pose.pose.orientation.z = 0
            # goal.pose.orientation.w = 1
            goal = navigator.transform(goal, "base_link")
            if goal is not None:
                success = navigator.goto(goal, 5)
                print(success)

        rate.sleep()

if __name__ == '__main__':
    main()
