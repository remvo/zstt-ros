#!/usr/bin/env python
import sys
from enum import Enum
from math import floor

import rospy
from cv_bridge import CvBridge, CvBridgeError
from gamecontroller_msgs.msg import GameState as GameStateMsg
from gamecontroller_msgs.msg import Data, Header, RobotInfo, TeamInfo
from sensor_msgs.msg import Image
from std_msgs.msg import (Bool, Float32MultiArray, Int32, Int32MultiArray,
                          String, UInt8)
from zstt import *
from zstt.srv import MotionControl

from object_detector import angle_between_three_points, midpoint

# TODO SET DISTANCE
"""
Distance info.
Key is head_up_down degree.
Value is (y pixel, distance) tuples
"""
DISTANCE = {
    90: [(480, 0)],
    100: [(215, 1000),
          (245, 700),
          (280, 500),
          (310, 400),
          (360, 300),
          (400, 250),
          (480, 190)],
    120: [(50, 700),
          (90, 500),
          (120, 400),
          (170, 300),
          (255, 200),
          (320, 150),
          (480, 85)],
    140: [(20, 240),
          (145, 150),
          (265, 90),
          (355, 60),
          (480, 30)],
    160: [(100, 80),
          (170, 60),
          (240, 40),
          (280, 30),
          (320, 20),
          (360, 10),
          (410, 0)]
}


class GameState(Enum):
    STATE_INITIAL = 0
    STATE_READY = 1
    STATE_SET = 2
    STATE_PLAYING = 3
    STATE_FINISHED = 4


class SecondaryState(Enum):
    STATE_NORMAL = 0
    STATE_PENALTYSHOOT = 1
    STATE_OVERTIME = 2
    STATE_TIMEOUT = 3
    STATE_DIRECT_FREEKICK = 4
    STATE_INDIRECT_FREEKICK = 5
    STATE_PENALTYKICK = 6
    DROPBALL = 128
    UNKNOWN = 255


class PlayState(Enum):
    PS_FIELD = 0
    PS_BALL = 1
    PS_BALL_IN_LINE = 2
    PS_BALL_DISTANCE = 3
    PS_GOAL = 4
    PS_GOAL_IN_LINE = 5
    PS_DRIBBLE = 6


class Penalties(Enum):
    NONE = 0
    SUBSTITUTE = 14  # TODO check if different for SPL than HL and what value is
    MANUAL = 15  # TODO check if different for SPL than HL and what value is

    HL_BALL_MANIPULATION = 30
    HL_PHYSICAL_CONTACT = 31
    HL_ILLEGAL_ATTACK = 32
    HL_ILLEGAL_DEFENSE = 33
    HL_PICKUP_OR_INCAPABLE = 34
    HL_SERVICE = 35


class StateController(object):

    def __init__(self):

        self.game_sub = rospy.Subscriber(
            '/robocup/gamecontroller_msgs', Int32, self.game_callback)

        self.game_state = -1
        self.second_state = -1
        self.play_state = -1
        self.play_step = -1

        self.sensor_sub = rospy.Subscriber(
            '/dataMsg', UInt8, self.sensor_callback)
        self.sensor_count = 0

        self.yaw = -1
        self.pitch = 0
        self.roll = 0

        self.base_yaw = -1
        self.target_yaw = -1

        self.cv_sub = rospy.Subscriber('image_raw', Image, self.cv_callback)
        self.cv_image = None
        self.bridge = CvBridge()

        self.field_sub = rospy.Subscriber('field_pub', Bool, self.set_state)
        self.ball_sub = rospy.Subscriber(
            'ball_pub', Int32MultiArray, self.ball_callback)
        self.goal_sub = rospy.Subscriber(
            'goal_pub', Float32MultiArray, self.goal_callback)

        self.field = None

        self.ball = None
        self.ball_distance = -1
        self.ball_angle = 0

        self.goal = None
        self.goal_target = None

        # TODO GET ANGLE TERM FROM RECONFIURE
        # 0 - 20 - 40 - 60 - 80 - 100 - 120 - 140 - 160 - 180 - 200 -220 - 240
        self.find_side = [0]*13

        # 90 - 100 - 110 - 120
        self.find_updown = [0]*4

        self.turn_count = 0

        # CURRENT ROBOT MOTION : GET BY RETURN FROM MOTION CONTROLLER
        # 0     SELECT_MODE
        # 1  2  STRIDE_LEFT_LEG / STRIDE_RIGHT_LEG
        # 3     SPEED
        # 4  5  SWING_LEFT_LEG / SWING_RIGHT_LEG
        # 6  7  UP_LEFT_LEG / UP_RIGHT_LEG
        # 8  9  TURN_LEFT_ANGLE / TURN_RIGHT_ANGLE
        # 10 11 OFFSET_LEFT_LEG / OFFSET_RIGHT_LEG
        # 12 13 HEAD_LEFT_RIGHT / HEAD_UP_DOWN
        self.motion = []

        self.is_running = False
        self.get_ready = False

    def sensor_callback(self, value):

        if value == 255:
            self.sensor_count = 0
        elif sensor_count == 0:
            self.sensor_count = 1
            self.yaw = value

            if self.base_yaw == -1:
                self.base_yaw = value

        elif sensor_count == 1:
            self.sensor_count = 2
            self.pitch = value
        elif sensor_count == 2:
            self.sensor_count = 3
            self.roll = value
        elif value == 254:
            self.sensor_count = -1

    def cv_callback(self, image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                image, desired_encoding='passthrough')
        except CvBridgeError as error:
            rospy.logerr(error)

        self.cv_image = cv_image

    def game_callback(self, message):
        self.game_state = message

    def field_callback(self, msg):
        # TRUE OR FALSE
        self.field = msg

    def ball_callback(self, msg):

        # rospy.loginfo(msg)

        if len(msg.data) != 0:
            self.ball = ((msg.data[0], msg.data[1]), msg.data[2])
            rospy.loginfo("%%%%%%%%%% BALL ")
            rospy.loginfo(self.ball)
        else:
            self.ball = None

    def goal_callback(self, msg):

        # rospy.loginfo(msg)

        if len(msg.data) != 0:
            if len(msg.data) > 2:
                self.goal[0] = ((msg.data[0], msg.data[1]), msg.data[2])
            if len(msg.data) > 5:
                self.goal[1] = ((msg.data[3], msg.data[4]), msg.data[5])
            rospy.loginfo("********** GOAL ")
            rospy.loginfo(self.goal)
        else:
            self.goal = None

    def set_state(self, msg):

        if self.is_running == True:
            return
        else:
            self.is_running == True

        while is_running:

            # INITIALIZE VARIABLE IN ALL LOOP
            motion = None

            """
            state, time = self.rec.get_last_state()
            game_state, secondary_state = self.rec.get_game_state()
            self.head_counter -= 1
            self.turn_counter -= 1

            if self.source == Source.VIDEO:
                # capture frame-by-frame (frame is BGR format)
                (grabbed, frame) = video.read()

                # if we are viewing a video and we did not grab a frame,
                # then we have reached the end of the video
                if not grabbed or frame is None:
                    break

            elif self.source == Source.WEBCAM:
                (grabbed, frame) = camera.read()

                if not grabbed:
                    continue

            # resize the frame, blur it, and convert it to the HSV color space
            frame = imutils.resize(frame, width=settings.IMAGE_WIDTH)
            self.display_frame = frame.copy()
            """

            # STATE_INITIAL
            if self.game_state == GameState.STATE_INITIAL:

                """
                self.mode = GameState.STATE_INITIAL
                self.did_ready = False
                self.doing_ready = False
                """

                # INITIALIZE VARIABLE FOR PLAYING
                self.play_step = 1
                self.play_state = PlayState.PS_FIELD

            # STATE_READY : START FROM SIDE AND MOVE TO CENTER
            if self.game_state == GameState.STATE_READY:

                """
                self.mode = GameState.STATE_READY

                # initialize some variable
                self.ball_ready = False
                self.all_ready = False

                if not self.doing_ready and not self.did_ready:
                    self.doing_ready = True
                    self.did_ready = False

                    rc = self.action.do_ready()
                    self.did_ready = True if rc else False
                    self.doing_ready = False if rc else False
                """

                if self.get_ready == False:
                    self.get_ready = True

                    # TODO CHECK WALKING TIME
                    motion = ['forward', 22]

            # STATE_SET : FIND BALL USING HEAD MOVING ONLY
            if self.game_state == GameState.STATE_SET:
                """
                self.mode = GameState.STATE_SET
                self.did_ready = False
                self.doing_ready = False

                # FIND FIELD
                self.field = detect_field(frame)

                # FIND BALL IN FIELD
                self.ball = None
                if self.field is not None:

                    # FILL BLACK COLOR TO NON-FIELD AREA
                    mask = np.zeros(frame.shape, dtype=np.uint8)
                    channel_count = frame.shape[2]  # i.e. 3 or 4 depending on your image
                    ignore_mask_color = (255,) * channel_count
                    cv2.fillPoly(mask, [self.field], ignore_mask_color)
                    field_image = cv2.bitwise_and(frame, mask)

                    # FIND_BALL
                    self.ball, self.ball_detect_method = detect_ball(field_image, self.action.head_up_down)

                if self.ball is None:
                    self.head_move_to_find_ball()
                else:
                    self.calc_ball_distance_angle(frame)
                """

                motion = self.find_object()

            if self.game_state == GameState.STATE_PLAYING:
                """
                self.mode = GameState.STATE_PLAYING
                self.did_ready = False
                self.doing_ready = False

                for team in state.teams:
                    if team.team_number == settings.TEAM_NUMBER:
                        if team.players[0].penalty != Penalties.NONE.name:
                            if not self.doing_penalty:
                                print('Penalty: {} ({}sec)'.format(team.players[0].penalty,
                                                                   team.players[0].secs_till_unpenalised))
                                self.doing_penalty = True
                                self.action.do_stop()
                        elif self.doing_penalty:
                            print('End of Penalty: Re-enter to field!')
                            self.doing_penalty = False
                            # go forward during 10 seconds
                            self.action.do_forward(delay=7)
                        break
                """

                # TODO IF PANELTY // STOP

                if self.play_step == 1:

                    # STEP 1-1. FIND FIELD
                    if self.play_state == PlayState.PS_FIELD:
                        if self.field == True:
                            self.play_state = PlayState.PS_BALL

                            # INIT HEAD BEFORE NEXT STATE
                            motion = ['stop', -1, -1, True]
                        else:
                            motion = self.find_object()

                    # STEP 1-2. FIND BALL
                    elif self.play_state == PlayState.PS_BALL:
                        # empty check
                        if self.BALL is not None:
                            self.play_state = PlayState.PS_BALL_IN_LINE

                            # INIT HEAD BEFORE NEXT STATE
                            motion = ['stop', -1, -1, True]
                        else:
                            motion = self.find_object()

                    # STEP 1-3. ADJUST ANGLE BETWEEN ROBOT AND BALL
                    elif self.play_state == PlayState.PS_BALL_IN_LINE:

                        # GET ANGLE AND DISTANCE OF BALL
                        self.calc_ball_distance_angle()

                        turn = self.ball_in_line()

                        if turn is None:
                            self.play_state = PlayState.PS_BALL_DISTANCE
                        elif turn == 'L':
                            motion = ['turnLeft', 3]
                        elif turn == 'R':
                            motion = ['turnRight', 3]

                    # STEP 1-4. ADJUST DISTANCE BETWEEN ROBOT AND BALL
                    elif self.play_state == PlayState.PS_BALL_DISTANCE:

                        # GET ANGLE AND DISTANCE OF BALL
                        self.calc_ball_distance_angle()

                        close_distance = rospy.get_param(
                            '/detector/option/min_to_forward', 150)

                        if self.ball_distance > close_distance:
                            motion = ['forward', 3]
                        else:
                            self.play_state = PlayState.PS_GOAL
                            self.play_step = 2

                elif self.play_step == 2:

                    # STEP 2-1. FIND GOAL
                    if self.play_state == PlayState.PS_GOAL:

                        if self.goal is not None:
                            self.play_state = PlayState.PS_GOAL_IN_LINE

                            if len(self.goal_post) > 1:
                                self.goal_target = midpoint(
                                    list(self.goal_post)[0][0], list(self.goal_post)[1][0])
                            elif len(self.goal_post) > 0:
                                self.goal_target = list(self.goal_post)[0][0]

                            # CHECK GOAL IS NOT MINE

                        else:
                            motion = self.find_object()

                    # STEP 2-2. ADJUST ANGLE BETWEEN ROBOT, BALL AND GOAL
                    elif self.play_state == PlayState.PS_GOAL_IN_LINE:

                        offset = rospy.get_param(
                            '/detector/option/yaw_offset', 7)

                        if self.base_yaw - offset < self.yaw < self.base_yaw + offset:
                            play.play_state = PlayState.PS_DRIBBLE
                            play.play_step = 3
                        else:
                            diff = self.action.base_yaw - self.action.yaw
                            if diff < 0 and abs(diff) <= 90:
                                motion = ['sideLeft', 125, -1]
                            elif diff > 0 and abs(diff) > 90:
                                motion = ['sideLeft', 125, -1]
                            elif diff < 0 and abs(diff) > 90:
                                motion = ['sideRight', 125, -1]
                            elif diff > 0 and abs(diff) <= 90:
                                motion = ['sideRight', 125, -1]

                        # rospy.loginfo('')

                elif self.play_step == 3:
                    # STEP 3-3. ADJUST DISTANCE BETWEEN BALL AND GOAL
                    if self.play_state == PlayState.PS_DRIBBLE:

                        # GET DISTANCE OF BALL
                        self.calc_ball_distance_angle()

                        if self.ball_distance < 50:
                            motion = ['leftKick', 1]
                        else:
                            motion = ['forward', 3]

            # STATE_FINISHED
            if self.game_state == GameState.STATE_FINISHED:
                """
                self.mode = GameState.STATE_FINISHED
                self.did_ready = False
                self.doing_ready = False
                self.action.do_stop()
                """
                rospy.loginfo('')

            if motion is not None:

                base_ud = self.motion[13]

                # CALL TO MOTION CONTROLLER
                if motion[0] == 'stop':
                    self.motion = motion_control_client(
                        motion[0], head_lr=motion[1], head_ud=motion[2], head_init=motion[3])
                elif motion[0] == 'sideLeft':
                    self.motion = motion_control_client(
                        motion[0], head_lr=motion[1], head_ud=motion[2], head_init=False)
                elif motion[0] == 'sideRight':
                    self.motion = motion_control_client(
                        motion[0], head_lr=motion[1], head_ud=motion[2], head_init=False)
                else:
                    self.motion = motion_control_client(
                        motion[0], duration=motion[1], head_init=False)

                # TODO CHANGE HEAD UP_DOWN , dynamic reconfigure VALUE ADJUST

            # 1 CALLBACK, 1 LOOP, ONLY ONE RUN
            self.is_running = False

    def find_object(self):

        # STEP1. FIND LEFT/RIGHT
        # 0 - 20 - 40 - 60 - 80 - 100 - 120 - 140 - 160 - 180 - 200 -220 - 240
        # self.find_side = [0-12]

        # GET CURRENT INDEX
        if(self.motion[12] > 240):
            idx = 12
        else:
            idx = floor(self.motion[12] / 20)

        # UPDATE FIND MAP / 0 : NOT YET / -1 : SEARCHED BUT FAIL
        self.find_side[idx] = -1

        # 12 HEAD_LEFT_RIGHT
        angle = self.motion[12]

        # FIND LEFT SIDE
        if idx > 0:
            for i in (idx-1, -1, -1):
                if self.find_side[i] == 0:
                    if angle - (20*(idx-1-i)) < 0:
                        angle = 0
                    else:
                        angle = angle - (20*(idx-1-i))
                    return ['stop', angle, -1, False]

        # FIND RIGHT SIDE
        if idx < 12:
            for i in (idx+1, 13):
                if self.find_side[i] == 0:
                    if angle + (20*(i-idx-1)) > 240:
                        angle = 240
                    else:
                        angle = angle + (20*(i-idx-1))
                    return ['stop', angle, -1, False]

        # STEP2. FIND UP DOWN
        # 90 - 100 - 110 - 120
        # self.find_updown = [0-3]

        # INIT LEFT_RIGHT MAP
        self.find_side = [0]*13

        # GET CURRENT INDEX
        idx = floor(self.motion[13] / 10) - 9

        # UPDATE FIND MAP / 0 : NOT YET / -1 : SEARCHED BUT FAIL
        self.find_updown[idx] = -1

        # 13 HEAD_UP_DOWN
        angle = self.motion[13]

        # FIND DOWN
        if idx > 0:
            for i in (idx-1, -1, -1):
                if self.find_updown[i] == 0:
                    if angle - (10*(idx-1-i)) < 0:
                        angle = 0
                    else:
                        angle = angle - (10*(idx-1-i))
                    return ['stop', -1, angle, False]

        # FIND UP
        if idx < 3:
            for i in (idx+1, 13):
                if self.find_updown[i] == 0:
                    if angle + (10*(i-idx-1)) > 120:
                        angle = 120
                    else:
                        angle = angle + (20*(i-idx-1))
                    return ['stop', -1, angle, False]

        # STEP3. FIND IN OTHER ANGLE

        # INIT FIND MAP
        self.find_side = [0]*13
        self.find_updown = [0]*4

        # TURN LEFT / MAXIMUM 6 TIMES
        if self.turn_count == 6:
            # TODO WAIT FEW SECONDS
            self.turn_count = 0
            return ['stop', -1, -1, True]
        else:
            # TODO TURN SECOND SETTING
            self.turn_count += 1
            return ['turnLeft', 3]

        return ['stop', -1, -1, True]

    def calc_ball_distance_angle(self):

        if self.ball is None:
            return
        if self.cv_image is None:
            return

        self.ball_distance = -1
        self.ball_angle = 0

        angle = floor(self.motion[13]/10) * 10

        # 90 100 110 120
        distance_info = DISTANCE.get(angle)

        for idx in range(1, len(distance_info)):
            if distance_info[idx][0] > self.ball[0][1]:
                y_diff = abs(distance_info[idx - 1][0] - distance_info[idx][0])
                d_diff = abs(distance_info[idx - 1][1] - distance_info[idx][1])
                ball_distance = distance_info[idx][1]
                ball_distance += (d_diff / y_diff) * \
                    (distance_info[idx][0] - self.ball[0][1])
                self.ball_distance = int(ball_distance)
                break
        # GET ANGLE BETWEEN BALL AND ROBOT (angle < 0: left side, angle > 0: right side)
        self.ball_angle = angle_between_three_points(self.ball[0],
                                                     (int(
                                                         self.cv_image.shape[1] / 2), self.cv_image.shape[0]),
                                                     (self.cv_image.shape[1] / 2, 0))

        # BALL UNDER 1CM (EX. LEG DETECTED BY BALL)
        if self.ball_distance < 0:
            self.ball = None
            self.ball_distance = -1
            self.ball_angle = 0

    def ball_in_line(self):

        turn = None

        max_bound = rospy.get_param('/detector/option/ball_inline_bound', 5)
        min_bound = max_bound * (-1)

        angle = self.ball_angle + (self.motion[12] - 125) * -1
        turn_angle = round(abs(angle), -1) // 15 * 25

        # TODO CHECK THIS LOGIC IS RIGHT
        # if turn_angle in in-line bound, stop turn and change state
        if min_bound < turn_angle < max_bound:
            return turn

        if turn_angle > 0 and angle < 0:
            turn = 'L'
        elif turn_angle > 0 and angle > 0:
            turn = 'R'

        return turn


def motion_control_client(motion, duration=0, head_lr=-1, head_ud=-1, head_init=False):

    rospy.wait_for_service('motion_control')

    try:
        motion_control = rospy.ServiceProxy('motion_control', MotionControl)
        resp1 = motion_control(motion, duration, head_lr, head_ud, head_init)
        return resp1.data
    except rospy.ServiceException, e:
        rospy.logerr('Service call failed: %s' % e)


def main():

    rospy.init_node('state_controller', anonymous=False)

    # STATE CONTROL START
    StateController()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')


if __name__ == '__main__':
    main()
