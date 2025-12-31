#!/usr/bin/env python3
"""
Torso 控制客户端（基于 ZMQ）
- 与 zmq_torso.py 服务端配套使用
- 支持 movej、get_pos、poweron/off、enable
- 内置微小运动测试（直接运行本文件即可调试）
"""

import zmq
import struct
import time


class TorsoClient:
    _MSG_FORMAT = '=10s6d'  # 10-byte cmd + 6 doubles

    def __init__(self, host='127.0.0.1', port=5601, timeout_ms=2000):
        self.host = host
        self.port = port
        self.timeout_ms = timeout_ms
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self._socket.connect(f"tcp://{host}:{port}")
        print(f"[TorsoClient] Connected to torso server at {host}:{port}")

    def _send_recv(self, cmd: str, data=None):
        if data is None:
            data = [0.0] * 6
        cmd_padded = cmd.encode('utf-8')[:10].ljust(10, b'\x00')
        msg = struct.pack(self._MSG_FORMAT, cmd_padded, *data)
        try:
            self._socket.send(msg)
            reply = self._socket.recv()
            _, *joints = struct.unpack(self._MSG_FORMAT, reply)
            return [float(j) for j in joints]
        except zmq.Again:
            raise TimeoutError(f"Torso command '{cmd}' timed out after {self.timeout_ms}ms")
        except Exception as e:
            raise RuntimeError(f"ZMQ communication error: {e}")

    def movej(self, joints, velocity=100, acceleration=80, blend=100):
        if len(joints) != 4:
            raise ValueError("Torso has 4 DOF, please provide 4 joint values.")
        full_joints = list(joints) + [0.0, 0.0]
        return self._send_recv('movej', full_joints)

    def get_pos(self):
        joints = self._send_recv('get_pos')
        return joints[:4]

    def power_on(self):
        self._send_recv('poweron')
        time.sleep(0.5)

    def power_off(self):
        self._send_recv('poweroff')
        time.sleep(0.5)

    def enable(self):
        self._send_recv('enable')
        time.sleep(0.5)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self):
        if hasattr(self, '_socket'):
            self._socket.close()
        if hasattr(self, '_context'):
            self._context.term()
        print("[TorsoClient] Connection closed.")
# ==================== 调试：微小运动测试 ====================
def micro_move_test():
    """安全的微小运动测试：前倾 + 回正"""
    with TorsoClient() as torso:
        print("=== Torso 微小运动调试 ===")

        # 0. 确保上电使能（可选，如果你已手动操作可注释）
        # print("→ 上电 & 使能...")
        # torso.power_on()
        # torso.enable()
        
        # 1. 读取当前姿态（作为站立基准）
        current = torso.get_pos()
        print(f"当前姿态: {current}")

        # 2. 微小前倾：仅调整第一个关节（Pitch1）
        # 根据你的校准，+0.05 表示前倾（如果效果相反，改用 -0.05）
        delta = 0.05
        micro_lean = [
            current[0] + delta,
            current[1],
            current[2],
            current[3]
        ]
        print(f"→ 微倾目标 (Δ={delta:.2f}): {micro_lean}")
        torso.movej(micro_lean)
        time.sleep(1.2)

        # 3. 回到原位
        print("→ 回到原位...")
        torso.movej(current)
        time.sleep(1.2)

        # 4. 验证最终位置
        final = torso.get_pos()
        print(f"最终姿态: {final}")
        print("✅ 调试完成！观察是否有轻微前倾后回正。")


# ==================== 主入口 ====================
if __name__ == "__main__":
    micro_move_test()
