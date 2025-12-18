#!/usr/bin/env python3
import zmq
import struct

class TestMsg:
    FORMAT = '=10s6i'

    def __init__(self, cmd='', data=None):
        self.cmd = cmd.encode('utf-8')[:10].ljust(10, b'\x00')
        self.data = data if data is not None else [0] * 6

    def pack(self):
        return struct.pack(self.FORMAT, self.cmd, *self.data)


# === 左手控制（端口 5590）===
print("Sending left hand command...")
context_l = zmq.Context()
socket_l = context_l.socket(zmq.REQ)
socket_l.connect("tcp://127.0.0.1:5590")

msg_l = TestMsg(cmd='lhand', data=[1000, 1000, 1000, 993, 1, 113])
socket_l.send(msg_l.pack())
response_l = socket_l.recv()
print("Left hand: Received response")

socket_l.close()
context_l.term()

# === 右手控制（端口 5591）===
print("Sending right hand command...")
context_r = zmq.Context()
socket_r = context_r.socket(zmq.REQ)
socket_r.connect("tcp://127.0.0.1:5591")

msg_r = TestMsg(cmd='rhand', data=[1000, 1000, 1000, 491, 1000, 0])
socket_r.send(msg_r.pack())
response_r = socket_r.recv()
print("Right hand: Received response")

socket_r.close()
context_r.term()
