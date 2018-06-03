#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# from dynamic_reconfigure.server import Server
# from robocup2018.cfg import colorsConfig

class ObjectDetector(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None

        self.cv_sub = rospy.Subscriber('image_raw', Image, self.cv_callback)
        self.lab_sub = rospy.Subscriber('image_lab', Image, self.lab_callback)

        # self.field_pub = rospy.Publisher('field_pub', Image)
        # self.ball_pub  = rospy.Publisher('ball_pub', Image)
        # self.goal_pub  = rospy.Publisher('goal_pub', Image)

        # INIT MASK COLORS
        self.field_lab = {'upper': [],'lower': []}
        # self.field_lab = np.array([[0, 0, 0], [0, 0, 0]])
        # self.ball_white_lab = np.array([[0, 0, 0], [0, 0, 0]])
        # self.ball_black_lab = np.array([[0, 0, 0], [0, 0, 0]])
        # self.goal_lab = np.array([[0, 0, 0], [0, 0, 0]])

    def cv_callback(self, image_message):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
        except CvBridgeError as error:
            rospy.logerr(error)

        self.cv_image = cv_image

    def lab_callback(self, image_message):
        try:
            lab_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
        except CvBridgeError as error:
            rospy.logerr(error)

        # TYPE1
        # 0. GET MASK COLOR FROM DYNAMIC RECONFIGURE SETTING
        self.color_setting()

        field_lower = np.array(self.field_lab['lower'])
        field_upper = np.array(self.field_lab['upper'])
        mask = cv2.inRange(lab_image, field_lower, field_upper)
        res = cv2.bitwise_and(lab_image, lab_image, mask=mask)

        cv2.imshow('Image', res)
        if self.cv_image is not None:
            cv2.imshow('CV Image', self.cv_image)
        cv2.waitKey(1)

    ###############################
    # TYPE1. NO ADDITIONAL NODE
    ###############################
    def color_setting(self):
        # FIELD MASK LOWER
        self.field_lab['lower'] = [
            rospy.get_param('/object_detector/field/lower/L', 0),
            rospy.get_param('/object_detector/field/lower/A', 0),
            rospy.get_param('/object_detector/field/lower/B', 0)
        ]

        # FIELD MASK UPPER
        self.field_lab['upper'] = [
            rospy.get_param('/object_detector/field/upper/L', 255),
            rospy.get_param('/object_detector/field/upper/A', 255),
            rospy.get_param('/object_detector/field/upper/B', 255)
        ]

        # # BALL WHITE MASK LOWER
        # self.ball_white_lab[0][0] = rospy.get_param('/object_detector/ball_white_l_l')
        # self.ball_white_lab[0][1] = rospy.get_param('/object_detector/ball_white_l_a')
        # self.ball_white_lab[0][2] = rospy.get_param('/object_detector/ball_white_l_b')

        # # BALL WHITE MASK UPPER
        # self.ball_white_lab[1][0] = rospy.get_param('/object_detector/ball_white_u_l')
        # self.ball_white_lab[1][1] = rospy.get_param('/object_detector/ball_white_u_a')
        # self.ball_white_lab[1][2] = rospy.get_param('/object_detector/ball_white_u_b')

        # # BALL BLACK MASK LOWER
        # self.ball_black_lab[0][0] = rospy.get_param('/object_detector/ball_black_l_l')
        # self.ball_black_lab[0][1] = rospy.get_param('/object_detector/ball_black_l_a')
        # self.ball_black_lab[0][2] = rospy.get_param('/object_detector/ball_black_l_b')

        # # BALL BLACK MASK UPPER
        # self.ball_black_lab[1][0] = rospy.get_param('/object_detector/ball_black_u_l')
        # self.ball_black_lab[1][1] = rospy.get_param('/object_detector/ball_black_u_a')
        # self.ball_black_lab[1][2] = rospy.get_param('/object_detector/ball_black_u_b')

        # # GOAL MASK LOWER
        # self.goal_lab[0][0] = rospy.get_param('/object_detector/goal_l_l')
        # self.goal_lab[0][1] = rospy.get_param('/object_detector/goal_l_a')
        # self.goal_lab[0][2] = rospy.get_param('/object_detector/goal_l_b')

        # # GOAL MASK UPPER
        # self.goal_lab[1][0] = rospy.get_param('/object_detector/goal_u_l')
        # self.goal_lab[1][1] = rospy.get_param('/object_detector/goal_u_a')
        # self.goal_lab[1][2] = rospy.get_param('/object_detector/goal_u_b')


def dynamic_colors(config, level):
    return config

def main():
    rospy.init_node('object_detector', anonymous=False)

    # DYNAMIC COLOR SETTING ON
    # Server(colorsConfig, dynamic_colors)

    # OBJECT DETECT START
    ObjectDetector()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
