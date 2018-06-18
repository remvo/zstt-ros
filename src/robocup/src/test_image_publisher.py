#!/usr/bin/env python
import os

import cv2
import rospkg
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def publisher():
    image_pub = rospy.Publisher('image_raw', Image, queue_size=10)
    rospy.init_node('image_publisher', anonymous=False)
    rate = rospy.Rate(1)

    bridge = CvBridge()
    file_path = os.path.join(rospkg.RosPack().get_path('robocup'),
                             'resource',
                             '7.png')

    image = cv2.imread(file_path)
    try:
        msg = bridge.cv2_to_imgmsg(image, 'bgr8')
    except CvBridgeError as error:
        rospy.logerr(error)
        return

    while not rospy.is_shutdown():
        image_pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except KeyboardInterrupt:
        print('Shutting down')

    cv2.destroyAllWindows()
