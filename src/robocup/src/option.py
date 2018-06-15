#!/usr/bin/env python
import rospy
from dynamic_reconfigure.server import Server
from robocup.cfg import optionConfig

def callback(config, level):
    return config

def main():
    rospy.init_node('option', anonymous=False)

    # DYNAMIC RECONFIGURE SETTING ON
    Server(optionConfig, callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')

if __name__ == '__main__':
    main()
