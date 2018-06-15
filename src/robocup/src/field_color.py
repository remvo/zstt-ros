#!/usr/bin/env python
import rospy
from dynamic_reconfigure.server import Server
from robocup.cfg import fieldConfig
       

def callback(config, level):
    return config

def main():
    rospy.init_node('field_color')

    # DYNAMIC COLOR SETTING ON
    Server(fieldConfig, callback)
    

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')

if __name__ == '__main__':
    main()
