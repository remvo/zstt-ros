#!/usr/bin/env python
import rospy
from dynamic_reconfigure.server import Server
from robocup.cfg import ballConfig
       

def callback(config, level):
    return config

def main():
    rospy.init_node('ball_color')

    # DYNAMIC COLOR SETTING ON
    Server(ballConfig, callback)
    

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')

if __name__ == '__main__':
    main()
