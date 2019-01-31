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
        self.marker = None

    def callback(self, msg):
        self.marker = Marker(
                    type=Marker.SPHERE,
                    id=self.id,
                    action=Marker.ADD,
                    lifetime=rospy.Duration(0),
                    scale=Vector3(0.1, 0.1, 0.1),
                    ns='path_points',
                    pose=msg.pose.pose,
                    header=Header(frame_id='odom'),
                    color=ColorRGBA(0.0, 1.0, 1.0, 0.8))

    def update(self):
        self.id+=1
        self.pub.publish(self.marker)

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
