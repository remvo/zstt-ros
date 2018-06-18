#!/usr/bin/env python
#-*- coding: utf-8 -*-
import sys
from enum import Enum
from math import floor

import rospy
from cv_bridge import CvBridge, CvBridgeError
from gamecontroller_msgs.msg import GameState as GameStateMsg
from gamecontroller_msgs.msg import Data, Header, RobotInfo, TeamInfo
from sensor_msgs.msg import Image
from std_msgs.msg import (Bool, Float32MultiArray,
                          Int32, Int32MultiArray, String, UInt8)
from zstt.srv import MotionControl

import gc_constants
import zstt_constants
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

BALL_CLOSE_DISTANCE = 60


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


class StateController(object):

    def __init__(self):
        # GameController Data
        self.game_sub = rospy.Subscriber(
            '/robocup/gamecontroller_msgs', GameStateMsg, self.game_callback)
        self.game_contorller = None
        self.second_state = -1
        self.play_state = -1
        self.play_step = -1
        self.doing_penalty = False
        self.ball_ready = False
        self.all_ready = False

        # Sensor Data
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

        self.field_sub = rospy.Subscriber(
            'field_pub', Bool, self.field_callback)
        self.ball_sub = rospy.Subscriber(
            'ball_pub', Int32MultiArray, self.ball_callback)
        self.goal_sub = rospy.Subscriber(
            'goal_pub', Float32MultiArray, self.goal_callback)

        self.field = None

        self.ball = None
        self.ball_distance = -1
        self.ball_angle = 0

        self.goal_posts = None
        self.target_goal_post = None

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

        # Varaiabled for each game state
        self.init_state()

    def sensor_callback(self, value):
        # ??? sensor_count ???
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
            self.cv_image = self.bridge.imgmsg_to_cv2(
                image, desired_encoding='passthrough')
        except CvBridgeError as error:
            rospy.logerr(error)

    def game_callback(self, msg):
        self.game_contorller = msg.data
        self.work()

    def field_callback(self, msg):
        self.field = msg

    def ball_callback(self, msg):
        if len(msg.data) < 3:
            self.ball = None
            return
        self.ball = ((msg.data[0], msg.data[1]), msg.data[2])

    def goal_callback(self, msg):
        if len(msg.data) < 3:
            self.goal_posts = None
            return

        self.goal_posts = []
        goal_post = (msg.data[0], msg.data[1]), msg.data[2]
        self.goal_posts.append(goal_post)
        if len(msg.data) >= 6:
            goal_post = (msg.data[3], msg.data[4]), msg.data[5]
            self.goal_posts.append(goal_post)

    def work(self):
        if self.game_contorller is None:
            return

        # INITIAL
        if self.game_contorller.game_state == gc_constants.State.INITIAL.value:
            self.do_initial()

        # READY
        elif self.game_contorller.game_state == gc_constants.State.READY.value:
            self.do_ready()

        # SET
        elif self.game_contorller.game_state == gc_constants.State.SET.value:
            self.do_set()

        # PLAYING
        elif self.game_contorller.game_state == gc_constants.State.PLAYING.value:
            self.do_playing()

        # FINISHED
        elif self.game_contorller.game_state == gc_constants.State.FINISHED.value:
            self.do_finish()

        else:
            rospy.loginfo(self.game_contorller.game_state)

    def init_state(self):
        # Varaiabled for each game state
        self.ready_doing = False
        self.ready_done = False

        self.last_command = None
        self.last_data = None

        # Head info.
        self.move_up_down = 0
        self.move_left_right = -1

        # Ball info.
        self.ball = None
        self.ball_distance = -1
        self.ball_angle = 0

    # INITIAL_STATE: INITIATE VARIABLES
    def do_initial(self):
        self.init_state()

        self.play_step = 1
        self.play_state = PlayState.PS_FIELD

    # READY STATE: START FROM SIDE AND MOVE TO CENTER
    def do_ready(self):
        if not self.ready_done and not self.ready_doing:
            self.ready_doing = True
            self.last_command = 'forward'
            self.last_data = motion_control_client('forward', 2)

        self.ready_done = True

    # SET STATE: FIND BALL USING HEAD MOVING ONLY
    def do_set(self):
        self.did_ready = False
        self.doing_ready = False

        # If we already found the ball, do nothinig
        if self.ball is None:
            self.head_move_to_find_ball()
        else:
            self.calc_ball_distance_angle()

    # PLAYING STATE
    def do_playing(self):
        self.did_ready = False
        self.doing_ready = False
        if self.last_data is None:
            self.last_command = 'stop'
            self.last_data = motion_control_client('stop')

        # Check team penalty
        for team in self.game_contorller.teams_info:
            if team.team_number == zstt_constants.TEAM_NUMBER:
                if team.players[0].penalty != gc_constants.SPLPenalty.NONE.value:
                    if not self.doing_penalty:
                        rospy.logdebug('Penalty: {} ({}sec)'.format(team.players[0].penalty, team.players[0].secs_till_unpenalised))
                        self.doing_penalty = True
                        self.last_command = 'stop'
                        self.last_data = motion_control_client('stop')
                elif self.doing_penalty:
                    print('End of Penalty: Re-enter to field!')
                    self.doing_penalty = False
                    # go forward during 10 seconds
                    self.last_command = 'forward'
                    self.last_data = motion_control_client('forward', 7)

                break

        # Find the field
        if self.field is None:
            return

        # Find the ball
        if self.ball is None and not self.ball_ready and not self.all_ready:
            self.head_move_to_find_ball()
            return
        elif self.ball is not None:
            self.calc_ball_distance_angle()

        if self.all_ready and self.ball_distance < 50:
            self.last_command = 'rightKick'
            self.last_data = motion_control_client('rightKick')
            return
        
        if self.ball is not None and not self.ball_ready and not self.all_ready:
            # 머리 각도 및 공과의 각도에 따라 좌우로 회전
            angle = self.ball_angle + (self.last_data[zstt_constants.HEAD_LEFT_RIGHT] - 125) * -1
            turn_angle = round(abs(angle), -1) // 15 * 25
            turn_angle = 45 if turn_angle > 45 else turn_angle

            # 공이 가까이 있을 경우 머리 숙이기
            next_head_up_down = self.last_data[zstt_constants.HEAD_UP_DOWN]
            if self.last_data[zstt_constants.HEAD_UP_DOWN] <= 120 and self.ball_distance <= 130:
                next_head_up_down = 140
            elif self.last_data[zstt_constants.HEAD_UP_DOWN] <= 140 and self.ball_distance <= 80:
                next_head_up_down = 160

            # 회전이 필요한 경우
            if turn_angle > 0:
                turn_left_angle = turn_angle if angle < 0 else 0
                turn_right_angle = 0 if angle < 0 else turn_angle
                rospy.logdebug('Turn L: {}'.format(turn_angle) if angle < 0 else 'Turn R: {}'.format(turn_angle))

                # 머리가 정면을 보고있지 않은 경우, 몸을 회전시키며 머리도 회전
                next_head_angle = 0
                if abs(self.last_data[zstt_constants.HEAD_LEFT_RIGHT] - 125) > 15 and abs(self.ball_angle) < 30:
                    next_head_angle = (self.last_data[zstt_constants.HEAD_LEFT_RIGHT] - 125) * -1
                    if abs(next_head_angle) > 15:
                        next_head_angle = 15 * np.sign(next_head_angle)

                # 공이 가까이 있다면 'turn'을 사용하고, 멀리 있다면 앞으로 가며 회전하기 위해 'forward_0' 사용
                motion = 'turnLeft' if angle < 0 else 'turnRight'
                self.last_command = motion
                self.last_data = motion_control_client(motion, 0.5, 
                                                       head_lr=self.last_data[zstt_constants.HEAD_LEFT_RIGHT] + next_head_angle,
                                                       head_ud=next_head_up_down)
            # 회전이 필요없는 경우(공이 정면에 있을때)
            else:
                rospy.loginfo('Do not need turn, HLR({})'.format(self.last_data[zstt_constants.HEAD_LEFT_RIGHT]))
                # 공이 가까이 있다면 'stop'으로 멈추고, 멀리 있다면 앞으로 가며 회전하기 위해 'forward_0' 사용
                # 머리의 각도가 125 +- 15 사이일 경우 정면을 보도록 조정
                if abs(self.last_data[zstt_constants.HEAD_LEFT_RIGHT] - 125) < 15:
                    if self.ball_distance > BALL_CLOSE_DISTANCE:
                        self.last_command = 'forward'
                        self.last_data = motion_control_client('forward', 0.5, head_lr=125, head_ud=next_head_up_down)
                    else:
                        offset = 7
                        if self.base_yaw - offset < self.yaw < self.base_yaw + offset:
                            self.last_command = 'init_walk'
                            self.last_data = motion_control_client('init_walk', 1, head_lr=125, head_ud=next_head_up_down)
                            self.ball_ready = True
                            rospy.loginfo('Change State to ball_ready!')
                        # 공을 중심으로 골대(base_yaw)와 로봇이 직선상에 오도록 이동
                        else:
                            diff = self.base_yaw - self.yaw
                            if diff < 0 and abs(diff) <= 90:
                                motion = 'sideLeft'
                            elif diff > 0 and abs(diff) > 90:
                                motion = 'sideLeft'
                            elif diff < 0 and abs(diff) > 90:
                                motion = 'sideRight'
                            elif diff > 0 and abs(diff) <= 90:
                                motion = 'sideRight'

                            self.last_command = motion
                            self.last_data = motion_control_client(motion, 0.5, head_lr=125)

                # 머리의 각도가 정면에서 15도 이상 회전되었을 경우, 최대 15도 씩 정면을 보도록 회전
                else:
                    next_head_angle = (self.last_data[zstt_constants.HEAD_LEFT_RIGHT] - 125) * -1
                    if abs(next_head_angle) > 15:
                        next_head_angle = 15 * np.sign(next_head_angle)

                    self.last_command = 'init_walk'
                    self.last_data = motion_control_client('init_walk',
                                                            head_lr=self.last_data[zstt_constants.HEAD_LEFT_RIGHT] + next_head_angle,
                                                            head_ud=next_head_up_down)

                    if self.ball is not None and self.ball_distance < 50:
                        self.last_command = 'rightKick'
                        self.last_data = motion_control_client('rightKick', 2)

        # 공 - 골대 - 로봇이 직선상에 있을 경우
        if self.all_ready:
            rospy.loginfo('Everything is Ready! Shot!!!')
            self.last_command = 'rightKick'
            self.last_data = motion_control_client('rightKick', 2)

            # Kick
            # if secondary_state == SecondaryState.STATE_PENALTYKICK.name or secondary_state == SecondaryState.STATE_PENALTYSHOOT.name:
                # rc = self.action.do_kick(self.ball_distance)
            # else:
                # rc = self.do_action('forward_0', delay=2, head_left_right=125, head_up_down=160)
            self.all_ready = False
            self.target_goal_post = None

        # 공은 일직선상에 준비가 되어있고, 골대를 찾지 못한 경우
        if self.ball_ready and self.target_goal_post is None:
            target = None
            post_angle = None
            # 골대를 못찾았을 경우 머리를 상하로 조절
            rospy.loginfo('Try to find goal post')
            if self.goal_posts is None or len(self.goal_posts) == 0:
                next_head_up_down = self.last_data[zstt_constants.HEAD_UP_DOWN]
                motion = 'init_walk'
                # 위아래로 움직이기 (125: 처음 탐색, 124: 머리를 위아래로 움직이기 전 다시 봤을 경우)
                if next_head_up_down == 160:
                    next_head_up_down = 140
                elif next_head_up_down == 140:
                    next_head_up_down = 120
                elif next_head_up_down == 120:
                    next_head_up_down = 160
                self.last_command = motion
                self.last_data = motion_control_client(motion, head_ud=next_head_up_down, head_lr=125)
            # 찾아진 골대 중 기준으로 삼을 target_goal_post 설정
            elif len(self.goal_posts) == 2:
                target = midpoint(list(self.goal_posts)[0][0], list(self.goal_posts)[1][0])
            elif len(self.goal_posts) > 0:
                target = list(self.goal_posts)[0][0]

            # 골대 중앙과 로봇 정면과 각도 계산
            if target is not None:
                post_angle = angle_between_three_points(target,
                                                        (int(self.cv_image.shape[1] / 2), self.cv_image.shape[0]),
                                                        (self.cv_image.shape[1] / 2, 0))

                self.target_goal_post = target
                self.target_goal_post_angle = post_angle
                self.target_yaw = self.yaw - int(post_angle / 2)
                # self.action.base_yaw -= int(post_angle / 2)
                motion = 'stop' if self.last_command == 'stop' else 'init_walk'
                self.last_command = motion
                self.last_data = motion_control_client(motion, 0.5, head_ud=160, head_lr=125)
                self.all_ready = True
                self.ball_ready = False
                rospy.loginfo('Change State to all_ready!')
            elif self.action.head_up_down == 120:
                motion = 'stop' if self.last_command == 'stop' else 'init_walk'
                self.last_data = motion_control_client(motion, 0.5, head_ud=160, head_lr=125)
                self.all_ready = True
                self.ball_ready = False
                rospy.loginfo('Change State to all_ready!')

        elif self.ball_ready:
            self.all_ready = True
            self.ball_ready = False
            self.last_command = motion
            self.last_data = motion_control_client(motion, head_ud=160, head_lr=125)
            rospy.loginfo('Change State to all_ready!')


    # FINISHED STATE
    def do_finish(self):
        motion_control_client('stop')

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

    def head_move_to_find_ball(self):
        """Move head to find the ball"""
        if self.ball is not None or self.last_data is None:
            return

        next_head_up_down = self.last_data[zstt_constants.HEAD_UP_DOWN]
        next_head_left_right = self.last_data[zstt_constants.HEAD_LEFT_RIGHT]
        next_head_left_right += 60 * self.move_left_right

        # motion = 'head_move' if self.action.is_stop else 'stop'
        motion = 'stop'
        self.last_command = motion
        # robot see right side
        if next_head_left_right < 5:
            self.last_data = motion_control_client(
                motion, 0.5, head_ud=next_head_up_down, head_lr=124)
            self.move_left_right = 1
        # robot see left side
        elif next_head_left_right > 245:
            self.last_data = motion_control_client(
                motion, 0.5, head_ud=next_head_up_down, head_lr=126)
            self.move_left_right = -1

        # robot see center, again
        # (125: first search, 124: before robot need to head move up/down)
        elif self.last_data[zstt_constants.HEAD_LEFT_RIGHT] == 126:
            if self.last_data[zstt_constants.HEAD_UP_DOWN] == 160:
                next_head_up_down = 140
            elif self.last_data[zstt_constants.HEAD_UP_DOWN] == 140:
                next_head_up_down = 120
            elif self.last_data[zstt_constants.HEAD_UP_DOWN] == 120:
                next_head_up_down = 160
            elif self.last_data[zstt_constants.HEAD_UP_DOWN] == 90:
                next_head_up_down = 160

            self.last_data = motion_control_client(
                motion, 0.5, head_ud=next_head_up_down, head_lr=125)
        else:
            self.last_data = motion_control_client(
                motion, 0.5, head_ud=next_head_up_down, head_lr=next_head_left_right)

    def calc_ball_distance_angle(self):
        if self.ball is None or self.cv_image is None or self.last_data is None:
            return

        self.ball_distance = -1
        self.ball_angle = 0

        # angle = floor(self.motion[13]/10) * 10

        # 90 100 110 120
        distance_info = DISTANCE.get(
            self.last_data[zstt_constants.HEAD_UP_DOWN])
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
