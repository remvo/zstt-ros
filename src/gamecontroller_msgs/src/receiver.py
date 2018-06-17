#!/usr/bin/env python
#-*- coding:utf-8 -*-

from __future__ import unicode_literals, print_function

"""
This module shows how the GameController Communication protocol can be used
in python and also allows to be changed such that every team using python to
interface with the GC can utilize the new protocol.

.. moduleauthor:: Nils Rokita <0rokita@informatik.uni-hamburg.de>
.. moduleauthor:: Robert Kessler <8kessler@informatik.uni-hamburg.de>

"""

import rospy
import socket
import time
import logging

# from std_msgs.msg import String
from gamecontroller_msgs.msg import GameState as GameStateMsg, Header, Data, TeamInfo, RobotInfo

import constants
from construct import Container, ConstError
from gamestate import GameState, ReturnData


class GameStateReceiver(object):
    """ This class puts up a simple UDP Server which receives the
    *addr* parameter to listen to the packages from the game_controller.

    If it receives a package it will be interpreted with the construct data
    structure and the :func:`on_new_gamestate` will be called with the content.

    After this we send a package back to the GC """

    def __init__(self, team, player, addr=(constants.DEFAULT_LISTENING_HOST, constants.GAMECONTROLLER_LISTEN_PORT), answer_port=constants.GAMECONTROLLER_ANSWER_PORT):
        # Information that is used when sending the answer to the game controller
        self.team = team
        self.player = player
        self.man_penalize = False

        # The address listening on and the port for sending back the robots meta data
        self.addr = addr
        self.answer_port = answer_port

        # The state and time we received last form the GC
        self.state = None
        self.time = None

        # The socket and whether it is still running
        self.socket = None
        self.running = True

        self._open_socket()

    def _open_socket(self):
        """ Erzeugt das Socket """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.addr)
        self.socket.settimeout(0.5)
        self.socket2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    def receive_forever(self):
        """ Waits in a loop that is terminated by setting self.running = False """
        while self.running:
            try:
                self.receive_once()
            except IOError as e:
                rospy.logdebug('Fehler beim Senden des KeepAlive: %s' % (str(e)))

    def receive_once(self):
        """ Receives a package and interprets it.
            Calls :func:`on_new_gamestate`
            Sends an answer to the GC """
        try:
            self.state = None

            data, peer = self.socket.recvfrom(GameState.sizeof())

            # print(len(data))    # 640
            # Throws a ConstError if it doesn't work
            parsed_state = GameState.parse(data)

            # Assign the new package after it parsed successful to the state
            self.state = parsed_state
            self.time = rospy.get_rostime()

            # Call the handler for the package
            # self.on_new_gamestate(self.state)

            # Answer the GameController
            self.answer_to_gamecontroller(peer)

        except AssertionError as ae:
            rospy.logerr(ae.message)
        except socket.timeout:
            rospy.logwarn('Socket timeout')
        except ConstError:
            rospy.logwarn('Parse Error: Probably using an old protocol!')
        except Exception as e:
            rospy.logfatal(e)
            pass

    def answer_to_gamecontroller(self, peer):
        """ Sends a life sign to the game controller """
        return_message = 0 if self.man_penalize else 2

        data = Container(
            header=b'RGrt',
            version=constants.GAMECONTROLLER_RESPONSE_VERSION,
            team=self.team,
            player=self.player,
            message=return_message)
        try:
            destination = peer[0], constants.GAMECONTROLLER_ANSWER_PORT
            self.socket.sendto(ReturnData.build(data), destination)
        except Exception as e:
            rospy.logerr('Network Error: %s' % str(e))

    def on_new_gamestate(self, state):
        """ Is called with the new game state after receiving a package
            Needs to be implemented or set
            :param state: Game State
        """
        raise NotImplementedError()

    def get_last_state(self):
        return self.state, self.time

    def get_time_since_last_package(self):
        return time.time() - self.time

    def stop(self):
        self.running = False

    def set_manual_penalty(self, flag):
        self.man_penalize = flag


def receiver():
    rec = GameStateReceiver(team=30, player=1)
    pub = rospy.Publisher('/robocup/gamecontroller_msgs', GameStateMsg, queue_size=10)
    rospy.init_node('gamecontroller_client', anonymous=False)
    rate = rospy.Rate(3)    # 3hz => 3 issues per sec
    while not rospy.is_shutdown():
        rec.receive_once()

        try:
            msg = GameStateMsg()

            header = {
                'header': rec.state['header'],
                'version': rec.state['version'],
                'packet_number': rec.state['packet_number'],
                'stamp': rec.time
            }
            msg.header = Header(**header)

            teams_info = [{
                'team_number': int(team.team_number),
                'team_color': int(team.team_color),
                'score': int(team.score),
                'penalty_shot': int (team.penalty_shot),
                'single_shots': int(team.single_shots),
                'coach_sequence': int(team.coach_sequence),
                'coach_message': str(team.coach_message).replace('\x00',''),
                'players': [RobotInfo(
                    penalty=int(player.penalty),
                    secs_till_unpenalised=int(player.secs_till_unpenalised),
                    number_of_yellow_cards=int(player.number_of_yellow_cards),
                    number_of_red_cards=int(player.number_of_red_cards),
                ) for player in team.players if int(player.penalty) != constants.SPLPenalty.SUBSTITUTE.value]
            } for team in rec.state.teams]

            data = {
                'players_per_team': int(rec.state['players_per_team']),
                'game_type': int(rec.state['game_type']),
                'game_state': int(rec.state['game_state']),
                'first_half': int(rec.state['first_half']),
                'kick_of_team': int(rec.state['kick_of_team']),
                'secondary_state': int(rec.state['secondary_state']),
                'secondary_state_info': str(rec.state['secondary_state_info']).replace('\x00',''),
                'drop_in_team': int(rec.state['drop_in_team']),
                'drop_in_time': int(rec.state['drop_in_time']),
                'seconds_remaining': int(rec.state['seconds_remaining']),
                'secondary_seconds_remaining': int(rec.state['secondary_seconds_remaining']),
                'teams_info': [TeamInfo(**team_info) for team_info in teams_info]
            }
            msg.data = Data(**data)

            rospy.logdebug(rec.state)
            pub.publish(msg)
        except TypeError:
            pass

        rate.sleep()

if __name__ == '__main__':
    try:
        receiver()
    except rospy.ROSInterruptException:
        pass
