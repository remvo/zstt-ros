#!/usr/bin/env python
# -*- coding:utf-8 -*-

import constants
from construct import Byte, Struct, Enum, Bytes, Const, Array, Renamed, Int16ul

Short = Int16ul

RobotInfo = "robot_info" / Struct(
    "penalty" / Enum(Byte, constants.SPLPenalty),
    "secs_till_unpenalised" / Byte,
    "number_of_yellow_cards" / Byte,
    "number_of_red_cards" / Byte
)

TeamInfo = "team" / Struct(
    "team_number" / Byte,
    "team_color" / Enum(Byte, constants.SPLTeamColor),
    "score" / Byte,
    "penalty_shot" / Byte,  # penalty shot counter
    "single_shots" / Short,  # bits represent penalty shot success
    "coach_sequence" / Byte,
    "coach_message" / Bytes(253),
    "coach"/ RobotInfo,
    "players" / Array(11, RobotInfo)
)

GameState = "gamedata" / Struct(
    "header" / Const(constants.GAMECONTROLLER_STRUCT_HEADER, Bytes(4)),
    "version" / Const(constants.GAMECONTROLLER_STRUCT_VERSION, Short),
    "packet_number" / Byte,
    "players_per_team" / Byte,
    "game_type" / Byte,
    "game_state" / Enum(Byte, constants.State),
    "first_half" / Byte,
    "kick_of_team" / Byte,
    "secondary_state" / Enum(Byte, constants.State2),
    "secondary_state_info" / Bytes(4),
    "drop_in_team" / Byte,
    "drop_in_time" / Short,
    "seconds_remaining" / Short,
    "secondary_seconds_remaining" / Short,
    "teams" / Array(2, TeamInfo)
)

ReturnData = "returndata" / Struct(
    "header" / Const(b"RGrt", Bytes(4)),
    "version" / Const(constants.GAMECONTROLLER_RESPONSE_VERSION, Byte),
    "team" / Byte,
    "player" / Byte,
    "message" / Byte
)
