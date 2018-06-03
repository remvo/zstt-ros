#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('image_lab', Image, queue_size=10)
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as error:
            rospy.logerr(error)

        try:
            lab_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(lab_image, 'bgr8'))
        except CvBridgeError as error:
            rospy.logerr(error)

def main():
    ImageConverter()
    rospy.init_node('image_converter', anonymous=False)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
