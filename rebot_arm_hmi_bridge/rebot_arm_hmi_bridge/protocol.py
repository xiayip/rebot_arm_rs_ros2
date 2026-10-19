"""reBot HMI CAN v1; wire specification lives in arm_hmi_node/README.md."""
import struct
from enum import IntEnum

REQUEST, RESULT, STATE = 0x6F0, 0x6F1, 0x6F2
VERSION = 1
READY, BUSY, TRAJECTORY, FAULT = 1, 2, 4, 8


class Mode(IntEnum):
    UNKNOWN = 0
    POSITION = 1
    DAMPING = 2
    IMPEDANCE = 3


class Result(IntEnum):
    ACCEPTED = 0
    DONE = 1
    BAD_REQUEST = 2
    STALE_TOKEN = 3
    NOT_READY = 4
    TRAJECTORY_BUSY = 5
    SWITCH_FAILED = 6
    VERIFY_FAILED = 7
    SERVICE_TIMEOUT = 8
    REQUEST_BUSY = 9
    UNKNOWN_CONFIG = 10


def packet(mode, token, value=0, extra=0):
    return struct.pack('<BBIBB', VERSION, mode, token, value, extra)


def request(data):
    if len(data) != 8:
        return None
    version, mode, token, a, b = struct.unpack('<BBIBB', data)
    if version != VERSION or mode not in (1, 2, 3) or not token or a or b:
        return None
    return mode, token
