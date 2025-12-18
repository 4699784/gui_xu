#!/usr/bin/env python3
#左右灵巧手的控制
import zmq
import struct

class _TestMsg:
    FORMAT = '=10s6i'
    def __init__(self, cmd='', data=None):
        self.cmd = cmd.encode('utf-8')[:10].ljust(10, b'\x00')
        self.data = data if data is not None else [0] * 6
    def pack(self):
        return struct.pack(self.FORMAT, self.cmd, *self.data)
def _send_hand(cmd, data, port, timeout=2000):
    if len(data) != 6:
        raise ValueError("data must be a list/tuple of 6 integers")

    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.RCVTIMEO, timeout)
    socket.connect(f"tcp://127.0.0.1:{port}")

    try:
        msg = _TestMsg(cmd=cmd, data=data)
        socket.send(msg.pack())
        socket.recv()
        return True
    except zmq.Again:
        print(f"[ERROR] Timeout: hand '{cmd}' on port {port} not responding.")
        return False
    except Exception as e:
        print(f"[ERROR] Failed to send command to {cmd}: {e}")
        return False
    finally:
        socket.close()
        context.term()
def send_left_hand(data, port=5590):
    return _send_hand('lhand', data, port)
def send_right_hand(data, port=5591):
    return _send_hand('rhand', data, port)
def send_dual_hand(left_data=None, right_data=None):
    success = True
    if left_data is not None:
        success &= send_left_hand(left_data)
    if right_data is not None:
        success &= send_right_hand(right_data)
    return success