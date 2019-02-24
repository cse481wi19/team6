#!/usr/bin/env python

from sensor_msgs.msg import PointCloud2
import perception
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('publish_saved_cloud')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 2:
        is_sim = False
    else:
        is_sim = True
        path = argv[1]
    camera = perception.MockCamera()

    # rospy.sleep(0.5)


    pub = rospy.Publisher('mock_point_cloud', PointCloud2, queue_size=1)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        if is_sim:
            cloud = camera.read_cloud(path)
        else:
            cloud = rospy.wait_for_message('head_camera/depth_registered/points', PointCloud2)

        if cloud is None:
            rospy.logerr('Could not load point cloud from {}'.format(path))
            exit(1)
        
        cloud.header.stamp = rospy.Time.now()
        pub.publish(cloud)
        rate.sleep()


if __name__ == '__main__':
    main()
