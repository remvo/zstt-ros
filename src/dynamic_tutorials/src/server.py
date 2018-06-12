#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from dynamic_tutorials.cfg import TutorialsConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
          {str_param}, {bool_param}, {size}""".format(**config))
    #rospy.loginfo(config, level)
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_tutorials", anonymous=False)
    #rospy.set_param('gains', '2')
    rospy.set_param('~gains', "{'p': 1, 'i': 2, 'd': 3}")
    rospy.set_param('~gains/P', "{'p': 1, 'i': 2, 'd': 3}")

    srv = Server(TutorialsConfig, callback)
    rospy.spin()

