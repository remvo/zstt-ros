from enum import IntEnum

# Socket info.
DEFAULT_LISTENING_HOST = '0.0.0.0'
GAMECONTROLLER_LISTEN_PORT = 3838
GAMECONTROLLER_ANSWER_PORT = 3939

# Game Controller message info.
GAMECONTROLLER_STRUCT_HEADER = b'RGme'
GAMECONTROLLER_STRUCT_VERSION = 12
GAMECONTROLLER_RESPONSE_VERSION = 2

class SPLTeamColor(IntEnum):
    BLUE = 0    # cyan, blue, violet
    RED = 1     # magenta, pink (not red/orange)
    YELLOW = 2  # yellow
    BLACK = 3   # black, dark gray
    WHITE = 4   # white
    GREEN = 5   # green
    ORANGE = 6  # orange
    PURPLE = 7  # purple, violet
    BROWN = 8   # brown
    GRAY = 9    # lighter grey

class State(IntEnum):
    INITIAL = 0
    READY = 1
    SET = 2
    PLAYING = 3
    FINISHED = 4

class State2(IntEnum):
    NORMAL = 0
    PENALTYSHOOT = 1
    OVERTIME = 2
    TIMEOUT = 3
    DIRECT_FREEKICK = 4
    INDIRECT_FREEKICK = 5
    PENALTYKICK = 6
    DROPBALL = 128
    UNKNOWN = 255

class SPLPenalty(IntEnum):
    NONE = 0
    SUBSTITUTE = 14
    MANUAL = 15
    BALL_MANIPULATION = 30
    PUSHING = 31
    ILLEGAL_ATTACK = 32
    ILLEGAL_DEFENSE = 33
    PICKUP = 34
    SERVICE = 35
