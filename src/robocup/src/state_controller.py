#!/usr/bin/env python
from enum import Enum

import rospy
from std_msgs.msg import String, Int32, Bool, Int32MultiArray

class GameState(Enum):
    STATE_INITIAL  = 0
    STATE_READY    = 1
    STATE_SET      = 2
    STATE_PLAYING  = 3
    STATE_FINISHED = 4

class SecondaryState(Enum):
    STATE_NORMAL            = 0
    STATE_PENALTYSHOOT      = 1
    STATE_OVERTIME          = 2
    STATE_TIMEOUT           = 3
    STATE_DIRECT_FREEKICK   = 4
    STATE_INDIRECT_FREEKICK = 5
    STATE_PENALTYKICK       = 6
    DROPBALL                = 128
    UNKNOWN                 = 255

class PlayState(Enum):
    PS_FIELD         = 0
    PS_BALL          = 1
    PS_BALL_IN_LINE  = 2
    PS_BALL_DISTANCE = 3
    PS_GOAL          = 4
    PS_GOAL_IN_LINE  = 5
    PS_DRIBBLE       = 6

class Penalties(Enum):
    NONE       = 0
    SUBSTITUTE = 14  # TODO check if different for SPL than HL and what value is
    MANUAL     = 15  # TODO check if different for SPL than HL and what value is

    HL_BALL_MANIPULATION   = 30
    HL_PHYSICAL_CONTACT    = 31
    HL_ILLEGAL_ATTACK      = 32
    HL_ILLEGAL_DEFENSE     = 33
    HL_PICKUP_OR_INCAPABLE = 34
    HL_SERVICE             = 35

class StateController(object):

    def __init__(self):

        # TODO
        #self.game_sub  = rospy.Subscriber('', Int32, self.game_callback)
        #self.motor_sub = rospy.Subscriber('', Int32, self.game_callback)

        self.field_sub = rospy.Subscriber('field_pub', Bool, self.field_callback)
        self.ball_sub  = rospy.Subscriber('ball_pub', Int32MultiArray, self.ball_callback)
        self.goal_sub  = rospy.Subscriber('goal_pub', Float32MultiArray, self.goal_callback)

        #self.view_side = 1

        self.game_state   = -1
        self.second_state = -1
        self.play_state   = -1
        self.play_step    = -1

        self.field = None
        
        self.ball          = None
        self.ball_distance = -1
        self.ball_angle    = 0

        self.goal          = None
        self.goal_distance = -1
        self.goal_angle    = 0

        #TODO
        #self.find_side     =
        #self.find_updown   = 

        # START STATE CONTROLL
        self.set_state()

    def game_callback(self, message):
        self.game_state = message

    def field_callback(self, message):
        rospy.loginfo(message)

    def ball_callback(self, message):
        rospy.loginfo(message)

    def goal_callback(self, message):
        rospy.loginfo(message)

    def set_state(self):

        while True:

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
                self.play_step    = 1
                self.play_state   = PlayState.PS_FIELD


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

                # TODO
                motion = "GO_FORWARD"

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

                motion = "FIND_BALL"

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

                if self.play_step == 1:

                    # STEP 1-1. FIND FIELD
                    if self.play_state == PlayState.PS_FIELD:
                        rospy.loginfo('')

                    # STEP 1-2. FIND BALL
                    elif self.play_state == PlayState.PS_BALL:
                        rospy.loginfo('')

                    # STEP 1-3. ADJUST ANGLE BETWEEN ROBOT AND BALL
                    elif self.play_state == PlayState.PS_BALL_IN_LINE:
                        rospy.loginfo('')

                    # STEP 1-4. ADJUST DISTANCE BETWEEN ROBOT AND BALL
                    elif self.play_state == PlayState.PS_BALL_DISTANCE:
                        rospy.loginfo('')

                elif self.play_step == 2:

                    # STEP 2-1. FIND GOAL
                    if self.play_state == PlayState.PS_GOAL:
                        rospy.loginfo('')

                    # STEP 2-2. ADJUST ANGLE BETWEEN ROBOT, BALL AND GOAL
                    elif self.play_state == PlayState.PS_GOAL_IN_LINE:
                        rospy.loginfo('')

                elif self.play_step == 3:
                    # STEP 3-3. ADJUST DISTANCE BETWEEN BALL AND GOAL
                    if self.play_state == PlayState.PS_DRIBBLE:
                        rospy.loginfo('')


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
                # TODO SERVICE CALL TO MOTION CONTROLLER
                rospy.loginfo('')
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
