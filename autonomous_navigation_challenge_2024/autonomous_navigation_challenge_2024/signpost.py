import enum

class Signpost(enum.IntEnum):
    INVALID = -1
    FORWARD = 0
    LEFT = 1
    RIGHT = 2
    BACKWARD = 3
    STOP = 4