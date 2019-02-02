#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry
import time

class NavPath(object):
    def __init__(self):
        self.pub = rospy.Publisher('path_marker', Marker, queue_size=5)
        self.id = 0
        self.path = []
        self.point = None

    def callback(self, msg):
        self.point = msg.pose.pose.position

    def update(self):
        self.path.append(self.point)
        marker = Marker(
                    type=Marker.LINE_STRIP,
                    id=self.id,
                    lifetime=rospy.Duration(0),
                    scale=Vector3(0.06, 0.06, 0.06),
                    ns='path_points',
                    points=self.path,
                    header=Header(frame_id='odom'),
                    color=ColorRGBA(0.0, 1.0, 1.0, 0.8))
        self.pub.publish(marker)

def main():
    rospy.init_node('nav_node')
    nav_path = NavPath()
    rospy.Subscriber('odom', Odometry, nav_path.callback)

    rospy.sleep(0.5)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        nav_path.update()
        rate.sleep()


if __name__ == '__main__':
  main()
