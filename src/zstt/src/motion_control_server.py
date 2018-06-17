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


def generate_command(name, head_lr, head_ud, head_init):
    global commands, previous_data

    # using initial values from file
    new_data = list(commands[name])

    if (head_init):
        return new_data

    # using values from previous command
    if (previous_data):
        new_data[c.HEAD_LEFT_RIGHT] = previous_data[c.HEAD_LEFT_RIGHT]
        new_data[c.HEAD_UP_DOWN] = previous_data[c.HEAD_UP_DOWN]

    # using new values from caller
    if (head_lr >= 0):
        new_data[c.HEAD_LEFT_RIGHT] = head_lr
    if (head_ud >= 0):
        new_data[c.HEAD_UP_DOWN] = head_ud

    return new_data


def handle_motion_control(req):
    global commands, previous_command, previous_data

    rospy.loginfo('Service: {}, {}, {}, {}, {}'.format(req.name, req.duration, req.head_lr, req.head_ud, req.head_init))
    if (req.name in commands.keys()):
        # If request command is different with previous one, we need to initWalk for 1 sec.
        if (req.name != previous_command):
            new_data = generate_command('init_walk', req.head_lr, req.head_ud, req.head_init)
            # Todo: Send the data to serial
            rospy.sleep(1)

        new_data = generate_command(req.name, req.head_lr, req.head_ud, req.head_init)
        # Todo: Send the data to serial
        if (req.duration > 0):
            rospy.sleep(req.duration)

        previous_command = req.name
        previous_data = new_data
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
