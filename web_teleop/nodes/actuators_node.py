#!/usr/bin/env python

import robot_api
import rospy
from web_teleop.srv import SetTorso, SetTorsoResponse
from web_teleop.srv import SetHead, SetHeadResponse
from web_teleop.srv import SetGrip, SetGripResponse
from web_teleop.srv import SetArm, SetArmResponse

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = robot_api.Torso()
        self._head = robot_api.Head()
        self._grip = robot_api.Gripper()
        self._arm = robot_api.Arm()

    def handle_set_torso(self, request):
        # TODO: move the torso to the requested height
        self._torso.set_height(request.height)
        return SetTorsoResponse()

    def handle_set_head(self, request):
        # TODO: move the torso to the requested height
        self._head.pan_tilt(request.pan, request.tilt)
        return SetHeadResponse()

    def handle_set_arm(self, request):
        j1, j2, j3 = request.joint1, request.joint2, request.joint3
        j4, j5, j6, j7 = request.joint4, request.joint5, request.joint6, request.joint7
        self._arm.move_to_joints(robot_api.ArmJoints.from_list([j1, j2, j3, j4, j5, j6, j7]))
        return SetArmResponse()

    def handle_set_grip(self, request):
        if request.open:
            self._grip.open()
        else:
            self._grip.close()
        return SetGripResponse()


def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso,
                                  server.handle_set_torso)

    head_service = rospy.Service('web_teleop/set_head', SetHead,
                                  server.handle_set_head)

    arm_service = rospy.Service('web_teleop/set_arm', SetArm,
                                  server.handle_set_arm)

    grip_service = rospy.Service('web_teleop/set_grip', SetGrip,
                                  server.handle_set_grip)

    rospy.spin()


if __name__ == '__main__':
    main()
