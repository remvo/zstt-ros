#!/usr/bin/env python
import math
import numpy as np
import cv2
import rospy
from math import hypot
from numpy.linalg import norm
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Bool, Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import Image



class ObjectDetector(object):

    def __init__(self):
        self.bridge = CvBridge()

        self.cv_image = None
        self.lab_image = None
        self.view_image = None

        self.field = None
        self.field_mask = None

        self.ball = None

        self.cv_sub = rospy.Subscriber('image_raw', Image, self.cv_callback)
        self.lab_sub = rospy.Subscriber('image_lab', Image, self.lab_callback)

        self.field_pub = rospy.Publisher('field_pub', Bool)
        self.ball_pub  = rospy.Publisher('ball_pub', Int32MultiArray)
        self.goal_pub  = rospy.Publisher('goal_pub', Float32MultiArray)

        # INIT MASK COLORS
        self.field_lab = {'upper': [],'lower': []}
        self.ball_white_lab = {'upper': [],'lower': []}
        self.ball_black_lab = {'upper': [],'lower': []}
        self.goal_lab = {'upper': [],'lower': []}

        # INIT HOUGH CIRCLE OPTIONS
        self.hough_circle = {}

    def cv_callback(self, image_message):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
        except CvBridgeError as error:
            rospy.logerr(error)

        self.cv_image = cv_image

        #cv2.waitKey(3)

    def lab_callback(self, image_message):
        try:
            lab_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
        except CvBridgeError as error:
            rospy.logerr(error)

        #TODO MOVE BLUR TO CONVERT

        # DYNAMIC RECONFIGURE _ BLUR AMOUNT
        blur = rospy.get_param('/detector/option/blur', 5)

        # SAVE LAB IMAGE WITH BLUR
        self.lab_image = cv2.GaussianBlur(lab_image.copy(), (blur, blur), 0)

        ##### STEP 1. GET MASK COLOR FROM DYNAMIC RECONFIGURE SETTING
        self.dynamic_setting()

        ##### STEP 2. FIND FIELD OBJECT
        self.find_field()
        
        ##### STEP 3. FIND BALL OBJECT
        self.find_ball()

        ##### STEP 4. FIND GOAL OBJECT
        self.find_goal()

        #TODO SEND TOPIC : MERGE IMAGE (CONTOUR DISPLAY)

        if self.field is not None:
            if self.ball is not None:
                cv2.circle(self.view_image, self.ball[0], self.ball[1], (255, 255, 0), 2)
            #if self.goal is not None:
            # TODO
            
            # TEST
            cv2.imshow('view_image', self.view_image)


        cv2.waitKey(3)

    def find_field(self):

        ##### STEP 2-1. GET MASK VALUE
        lower = np.array(self.field_lab['lower'])
        upper = np.array(self.field_lab['upper'])

        ##### STEP 2-2. SET MASK TO LAB_IMAGE 
        # construct a mask for the color between lower and upper, then perform
        # a series of dilations and erosions to remove any small blobs left in the mask
        f_mask = cv2.inRange(self.lab_image.copy(), lower, upper)
        f_mask = cv2.erode(f_mask, None, iterations=2)
        f_mask = cv2.dilate(f_mask, None, iterations=2)

        # TEST
        # cv2.imshow('TEST', cv2.bitwise_and(self.cv_image, self.cv_image, mask=f_mask))

        ##### STEP 2-3. FIND FILED CONTOUR
        # find contours in the mask and initialize the current center of the field
        # we need to using copy, because findContours function modify the input image
        contour = cv2.findContours(f_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        try:
            # only proceed if at least one contour was found
            if len(contour) <= 0:
                self.field_mask = None
                self.field_pub.publish(False)
            else:
                min_field = rospy.get_param('/detector/option/min_field', 100)

                ##### STEP 2-4. MERGE CONTOUR AND FIND LARGEST ONE

                # merge closed contours
                contour = merge_contours(contour)

                # return the largest contour in the mask
                max_contour = max(contour, key=cv2.contourArea)
                
                if cv2.contourArea(max_contour) < min_field:
                    self.field_mask = None
                    self.field_pub.publish(False)
                else:
                    self.field = cv2.convexHull(max_contour)

                    field_mask = np.zeros(self.lab_image.shape, dtype=np.uint8)

                    ##### SETP 2-5. FILL BLACK COLOR TO NON-FIELD AREA
                    cv2.fillPoly(field_mask, [self.field], (255,) * self.lab_image.shape[2])

                    # SET MASK IMAGE FOR FINDING BALL
                    self.field_mask = cv2.bitwise_and(self.lab_image.copy(), field_mask)
                    self.view_image = cv2.bitwise_and(self.cv_image.copy(), field_mask)

                    # TEST
                    # cv2.imshow('FIELD', self.field_mask)
                    # cv2.imshow('FIELD', cv2.bitwise_and(self.cv_image.copy(), field_mask))

                    self.field_pub.publish(True)
        except CvBridgeError as error:
            rospy.logerr(e)
    
    def find_ball(self):

        obj_ball = Int32MultiArray()

        ##### STEP 3-1. CHECK FIELD AREA
        if self.field_mask is None:
            #rospy.loginfo("***** NO FIELD *****")
            self.ball_pub.publish(obj_ball)
            return
        
        f_image = self.field_mask.copy()

        ##### STEP 3-2. GET MASK VALUE & SET MASK
        w_lower = np.array(self.ball_white_lab['lower'])
        w_upper = np.array(self.ball_white_lab['upper'])
        white_mask = cv2.inRange(f_image, w_lower, w_upper)

        b_lower = np.array(self.ball_black_lab['lower'])
        b_upper = np.array(self.ball_black_lab['upper'])
        black_mask = cv2.inRange(f_image, b_lower, b_upper)

        b_mask = cv2.bitwise_or(white_mask, black_mask)
        b_mask = cv2.erode(b_mask, None, iterations=2)
        b_mask = cv2.dilate(b_mask, None, iterations=2)
        
        # TEST
        # cv2.imshow('TEST', b_mask)

        image = self.cv_image.copy()

        ##### STEP 3-3. SET MASK TO FIELD LAB_IMAGE 
        b_image = cv2.bitwise_and(image, image, mask=b_mask)

        # TEST
        # cv2.imshow('BALL', b_image)

        # bilateralFilter Parameters
        d = rospy.get_param('/detector/option/filter_d', 9)
        color = rospy.get_param('/detector/option/filter_color', 75)
        space = rospy.get_param('/detector/option/filter_space', 75)

        ##### STEP 3-4. SET BLUR AND PARAMETERS

        # image, d, sigmaColor, sigmaSpace
        gray = cv2.bilateralFilter(b_image, d, color, space)
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
        gray = cv2.dilate(gray, None, iterations=2)
        gray = cv2.erode(gray, None, iterations=2)

        # DYNAMIC RECONFIGURE PARAMETER
        hc = self.hough_circle

        # image, method, dp, minDist, param1, param2, minRadius, maxRadius
        circles = cv2.HoughCircles(image = gray, method=cv2.HOUGH_GRADIENT, dp=hc['dp'], minDist=hc['min_d'], param1=hc['p1'], param2=hc['p2'], minRadius=hc['min_r'], maxRadius=hc['max_r'])

        ball = None

        if circles is None:
            #rospy.loginfo("***** NO CIRCLE *****")
            self.ball_pub.publish(obj_ball)
        else:
            # CHANGE CIRCLE DATA'S ORDER
            circles = [((circle[0], circle[1]), circle[2]) for circle in circles[0]]
            
            # FIND BALL
            ball = get_ball_from_circles(circles)


        if ball is None:
            #rospy.loginfo("***** NO BALL *****")
            self.ball = None
            self.ball_pub.publish(obj_ball)

        else:
            self.ball = ball_to_int(ball)

            (x, y), radius = self.ball
            
            obj_ball.data = [x, y, radius]

            #rospy.loginfo(ball)
            #rospy.loginfo(obj_ball)

            self.ball_pub.publish(obj_ball)

    def find_goal(self):

        obj_goal = Float32MultiArray()

        ##### STEP 4-1. CHECK FIELD AREA
        if self.field_mask is None:
            #rospy.loginfo("***** NO FIELD *****")
            self.goal_pub.publish(obj_goal)
            return

        ##### STEP 4-2. SET MASK TO LAB_IMAGE 
        lower = np.array(self.goal_lab['lower'])
        upper = np.array(self.goal_lab['upper'])
        g_mask = cv2.inRange(self.field_mask.copy(), lower, upper)

        # TEST
        # cv2.imshow('GOAL', cv2.bitwise_and(self.cv_image, self.cv_image, mask=g_mask))

        ##### STEP 4-3. FIND FILED CONTOUR AND REMOVE TOO SMALL OR TOO BIG
        contours = cv2.findContours(g_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        min_goal = rospy.get_param('/detector/option/min_goal')
        max_goal = rospy.get_param('/detector/option/max_goal')

        contours = [contour for contour in contours
                      if min_goal < cv2.contourArea(contour) < max_goal]

        if len(contours) <= 0:
            #rospy.loginfo("##### NO GOAL POST #####")
            self.goal_pub.publish(obj_goal)
        else:
            goal_post = set()

            thres_goal_dist = rospy.get_param('/detector/option/thres_goal_distance')
            thres_goal_angle = rospy.get_param('/detector/option/thres_goal_angle')

            image = self.field_mask.copy()
            field = self.field
                
            for i in range(len(field)):
                p1 = field[i][0]
                p2 = field[(i + 1) % len(field)][0]
                p3 = field[(i + 2) % len(field)][0]
                    
                angle = angle_between_three_points(p1, p2, p3)
                    
                # if the three points on single line, calculate distance between line and goal post candidates
                if abs(180 - angle) < thres_goal_angle:
                    for contour in contours:
                        M = cv2.moments(contour)
                        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                            
                        if center[1] > image.shape[0] / 2:
                            continue

                        dist = int(norm(np.cross(p2 - p1, p1 - center)) / norm(p2 - p1))

                        if dist < thres_goal_dist:
                            goal_post.add((center, cv2.contourArea(contour)))

                if len(goal_post) == 2:
                    #rospy.loginfo(goal_post)
                    #(x1, y1), point1 = goal_post[0]
                    #(x2, y2), point2 = goal_post[1]
                    #obj_goal.data = [x1, y1, point1, x2, y2, point2]
                    break

            if len(goal_post) > 0:
                (x1, y1), point1 = goal_post[0] 
                obj_goal.data[0], obj_goal.data[1], obj_goal.data[2] = x1, y1, point1

            if len(goal_post) > 1:
                (x2, y2), point2 = goal_post[1]
                obj_goal.data[3], obj_goal.data[4], obj_goal.data[5] = x2, y2, point2

            #rospy.loginfo("%%%%% GOAL POST %%%%%")
            #rospy.loginfo(obj_goal)

            self.goal_pub.publish(obj_goal)

    def dynamic_setting(self):
        # FIELD MASK LOWER
        self.field_lab['lower'] = [
            rospy.get_param('/detector/field_color/lower_L'),
            rospy.get_param('/detector/field_color/lower_A'),
            rospy.get_param('/detector/field_color/lower_B')
        ]

        # FIELD MASK UPPER
        self.field_lab['upper'] = [
            rospy.get_param('/detector/field_color/upper_L'),
            rospy.get_param('/detector/field_color/upper_A'),
            rospy.get_param('/detector/field_color/upper_B')
        ]
        

        # BALL WHITE MASK LOWER
        self.ball_white_lab['lower'] = [
            rospy.get_param('/detector/ball_color/w_lower_L', 0),
            rospy.get_param('/detector/ball_color/w_lower_A', 0),
            rospy.get_param('/detector/ball_color/w_lower_B', 0)
        ]

        # BALL WHITE MASK UPPER
        self.ball_white_lab['upper'] = [
            rospy.get_param('/detector/ball_color/w_upper_L', 255),
            rospy.get_param('/detector/ball_color/w_upper_A', 255),
            rospy.get_param('/detector/ball_color/w_upper_B', 255)
        ]

        # BALL BLACK MASK LOWER
        self.ball_black_lab['lower'] = [
            rospy.get_param('/detector/ball_color/b_lower_L', 0),
            rospy.get_param('/detector/ball_color/b_lower_A', 0),
            rospy.get_param('/detector/ball_color/b_lower_B', 0)
        ]

        # BALL BLACK MASK UPPER
        self.ball_black_lab['upper'] = [
            rospy.get_param('/detector/ball_color/b_upper_L', 255),
            rospy.get_param('/detector/ball_color/b_upper_A', 255),
            rospy.get_param('/detector/ball_color/b_upper_B', 255)
        ]

        # GOAL MASK LOWER
        self.goal_lab['lower'] = [
            rospy.get_param('/detector/goal_color/lower_L', 0),
            rospy.get_param('/detector/goal_color/lower_A', 0),
            rospy.get_param('/detector/goal_color/lower_B', 0)
        ]
        # GOAL MASK UPPER
        self.goal_lab['upper'] = [
            rospy.get_param('/detector/goal_color/upper_L', 255),
            rospy.get_param('/detector/goal_color/upper_A', 255),
            rospy.get_param('/detector/goal_color/upper_B', 255)
        ]

        # HOUGH CIRCLE PRAMETER
        self.hough_circle = {
            'dp'    : rospy.get_param('/detector/option/dp'),
            'min_d' : rospy.get_param('/detector/option/min_distance'),
            'p1'    : rospy.get_param('/detector/option/param1'),
            'p2'    : rospy.get_param('/detector/option/param2'),
            'min_r' : rospy.get_param('/detector/option/min_radius'),
            'max_r' : rospy.get_param('/detector/option/max_radius')
        }


def angle_between_three_points(point1, point2, point3):
    """
    Calculate angle between point1 - point2 - point3
      param  point1 : (x1, y1)
      param  point2 : (x2, y2)
      param  point3 : (x3, y3)
      return absolute angle in degree integer
    """
    angle = math.atan2(point1[1] - point2[1], point1[0] - point2[0])
    angle -= math.atan2(point3[1] - point2[1], point3[0] - point2[0])
    angle = int(math.degrees(angle))
    return angle


def midpoint(point1, point2):
    return (point1[0] + point2[0]) * 0.5, (point1[1] + point2[1]) * 0.5


def distance(point1, point2):
    """
    Calculate distance between point1 and point2
      param  point1 : (x1, y1)
      param  point2 : (x2, y2)
      return distance in int type
    """
    return int(hypot(point1[0] - point2[0], point1[1] - point2[1]))


def merge_contours(cnts):
    """
    Merge closed contours
      param   cnts : contour list
      return  merged list
    """

    while True:
        cnts, merged = merge_contours_sub(cnts)
        if merged is False:
            return cnts


def merge_contours_sub(cnts):
    """
    Merge closed contours
    If two contours are merged successfully, we return immediately (merge performed only once).
      param  cnts : contour list
      return merged list, merged or not
    """

    min_contour = rospy.get_param('/detector/option/min_contour', 100)
    merge_field = rospy.get_param('/detector/option/merge_field', 250)

    for i in range(len(cnts) - 1):

        # if the contour is not sufficiently large, ignore it
        if cv2.contourArea(cnts[i]) < min_contour:
            continue

        center1, ret = get_center_from_contour(cnts[i])

        if ret is False:
            return cnts, False

        for j in range(i + 1, len(cnts)):
            if cv2.contourArea(cnts[j]) < min_contour:
                continue

            center2, ret = get_center_from_contour(cnts[j])
            if ret is False:
                return cnts, False

            dist = hypot(center1[0] - center2[0], center1[1] - center2[1])
            threshold = (math.sqrt(cv2.contourArea(cnts[i]))
                            + math.sqrt(cv2.contourArea(cnts[j])))
            if dist < threshold - merge_field:
                cnts.append(np.concatenate((cnts[i], cnts[j])))
                cnts.pop(j)
                cnts.pop(i)
                return cnts, True
    return cnts, False


def get_center_from_contour(contour):
    try:
        M = cv2.moments(contour)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        return center, True
    except ZeroDivisionError:
        return None, False

def get_ball_from_circles(circles):
    """
    FIND BALL FROM CIRCLE LIST
    """
    if circles is None:
        return None

    min_radius = rospy.get_param('/detector/option/min_ball_radius')
    max_radius = rospy.get_param('/detector/option/max_ball_radius')

    for circle in circles:
        if min_radius < circle[1] < max_radius:
            #TODO DISPLAY BALL'S MOVING ?
            #rospy.loginfo("!!!!! BALL !!!!!")
            #rospy.loginfo(circle)
            return circle

    return None

def ball_to_int(ball):
    """
    CHANGE BALL TO INT FROM FLOAT
    """
    if ball is None:
        return None

    return (int(ball[0][0]), int(ball[0][1])), int(ball[1])


def main():
    rospy.init_node('object_detector', anonymous=False)

    # OBJECT DETECT START
    ObjectDetector()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
