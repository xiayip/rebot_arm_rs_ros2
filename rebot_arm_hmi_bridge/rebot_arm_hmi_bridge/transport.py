"""Bounded Linux SocketCAN transport; accepts only standard classic requests."""
import fcntl
import os
import socket
import struct
from .protocol import REQUEST

FRAME = struct.Struct('=IB3x8s')


class CanTransport:
    def __init__(self, interface):
        if not interface or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in interface):
            raise ValueError('invalid CAN interface')
        # One bridge per interface/user. Do not take ownership of interface setup.
        path = f'/tmp/rebot-hmi-{os.getuid()}-{interface}.lock'
        self.lock = open(path, 'a')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                               struct.pack('=II', REQUEST, 0xE00007FF))
        self.socket.setblocking(False)
        self.socket.bind((interface,))

    def receive(self):
        try:
            raw = self.socket.recv(16)
        except BlockingIOError:
            return None
        if len(raw) != 16:
            return b''
        can_id, length, data = FRAME.unpack(raw)
        return data if can_id == REQUEST and length == 8 else b''

    def send(self, can_id, data):
        self.socket.send(FRAME.pack(can_id, 8, data))

    def close(self):
        self.socket.close()
        self.lock.close()
