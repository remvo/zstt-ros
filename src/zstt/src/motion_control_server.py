#!/usr/bin/env python
from zstt.srv import MotionControl, MotionControlResponse
import constants as c

import rospy
import rospkg
import os
import yaml

NAME = 'motion_control_server'


previous_command = None
previous_data = None
commands = {}


def get_command_from_file():
    global commands

    package_path = rospkg.RosPack().get_path('zstt')
    file_path = os.path.join(package_path, 'resource', 'command.yaml')

    try:
        with file(file_path, 'r') as stream:
            command_data = yaml.load(stream)
            rospy.loginfo('Get command data from file: ' + file_path)
            commands = command_data['commands']
            rospy.loginfo(commands)

    except Exception as e:
        rospy.logerr('except: {}'.format(e))
        commands = {}


def handle_motion_control(req):
    global commands, previous_command, previous_data

    rospy.loginfo('Service: {}, {}, {}, {}, {}'.format(
        req.name, req.duration, req.head_lr, req.head_ud, req.head_init))
    if (req.name in commands.keys()):
        # using initial values from file
        new_data = list(commands[req.name])

        if (not req.head_init):
            # using values from previous command
            if (previous_data):
                new_data[c.HEAD_LEFT_RIGHT] = previous_data[c.HEAD_LEFT_RIGHT]
                new_data[c.HEAD_UP_DOWN] = previous_data[c.HEAD_UP_DOWN]

            # using new values from caller
            if (req.head_lr >= 0):
                new_data[c.HEAD_LEFT_RIGHT] = req.head_lr
            if (req.head_ud >= 0):
                new_data[c.HEAD_UP_DOWN] = req.head_ud

        previous_command = req.name
        previous_data = new_data

        # Todo: If request command is different with previous one, we need to initWalk for 1 sec.
        # Todo: Send the data to serial
        if (req.duration > 0):
            rospy.sleep(req.duration)

        return MotionControlResponse(new_data)
    else:
        return MotionControlResponse([])


def motion_control_server():
    rospy.init_node(NAME, log_level=rospy.INFO)
    get_command_from_file()
    s = rospy.Service('motion_control', MotionControl, handle_motion_control)

    rospy.loginfo('Ready to motion_control')
    # spin() keeps Python from exiting until node is shutdown
    rospy.spin()


if __name__ == '__main__':
    motion_control_server()
