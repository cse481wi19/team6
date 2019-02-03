#! /usr/bin/env python

import robot_api
from map_annotator import MapAnnotator
import rospy
import sys

def wait_for_time():
    """
    Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def print_usage():
    print('Commands:\n' +\
          '  list: List saved poses.\n' +\
          '  save <name>: Save the robot\'s current pose as <name>. Overwrites if <name> already exists.\n' +\
          '  delete <name>: Delete the pose given by <name>.\n' +\
          '  goto <name>: Sends the robot to the pose given by <name>.\n' +\
          '  help: Show this list of commands\n' +\
          '  q: quit\n')

def print_list(poses):
    print('Poses:')
    for pose in poses:
        print('  ' + pose)


def main():
    rospy.init_node('map_annotate_demo')
    wait_for_time()
    print('Welcome to the map annotator!\n')
    print_usage()
    rospy.sleep(0.5)

    annotator = MapAnnotator()
    rate = rospy.Rate(10)
    while True :
        print('\n> '),
        l = sys.stdin.readline().split()
        c = l[0]
        name = None
        if len(l) > 1:
            name = ' '.join(l[1:])

        if c == 'q':
            break
        elif c == 'help':
            print_usage()
        elif c == 'list':
            print_list(annotator.list())
        elif c == 'save':
            annotator.save(name)
        elif c == 'delete':
            annotator.delete(name)
        elif c == 'goto':
            annotator.goto(name)

        rate.sleep()

if __name__ == '__main__':
    main()
