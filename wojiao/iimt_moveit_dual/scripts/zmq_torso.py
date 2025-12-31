import zmq
import socket
import time
import struct
import threading
import sys

###zmq服务端
###实现torso的控制

SERVER_IP = "192.168.8.110"
TCP_PORT = 30003
ZMQ_PORT = 5601

class TestMsg:
    FORMAT = '=10s6d'
    def __init__(self, cmd='', data=None):
        self.cmd = cmd.encode('utf-8')[:10].ljust(10, b'\x00')
        self.data = data if data is not None else [0.0] * 6
    #打包
    def pack(self):
        return struct.pack(self.FORMAT, self.cmd, *self.data)
    #拆包
    def unpack(self, binary_data):
        if len(binary_data) != struct.calcsize(self.FORMAT):
            raise ValueError(f"Binary data length {len(binary_data)} does not match expected {struct.calcsize(self.FORMAT)}")
        self.cmd, *self.data = struct.unpack(self.FORMAT, binary_data)
        self.cmd = self.cmd.decode('utf-8').rstrip('\x00')
        return self

class TorsoController:
    FORMAT_SOCKET = '6d'  # 6 doubles for joint positions
    JOINT_DATA_OFFSET = 244
    JOINT_DATA_SIZE = 48  # 6 * 8 bytes

    def __init__(self):
        self.client_socket = None
        self.running = True
        self.double_value = [0.0] * 6

        # Connect to torso device
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((SERVER_IP, TCP_PORT))
            print(f"Torso socket connected to {SERVER_IP}:{TCP_PORT}")
        except Exception as e:
            print(f"Failed to connect to torso device: {e}")
            sys.exit(1)

        # Start receiving thread
        self.recv_thread = threading.Thread(target=self.run_socket_resv, daemon=True)
        self.recv_thread.start()

    def send_cmd(self, string_cmd):
        """Send a plain-text command to the torso device."""
        if not self.running or self.client_socket is None:
            return
        try:
            print(f"Sending torso command: {string_cmd}")
            self.client_socket.sendall(string_cmd.encode('utf-8'))
        except Exception as e:
            print(f"Error sending command: {e}")

    #持续从TCP链接中读取二进制数据流
    def run_socket_resv(self):
        """Continuously receive binary data from torso and extract joint positions."""
        while self.running:
            try:
                data = self.client_socket.recv(1024)
                if not data:
                    print("Torso socket closed by peer.")
                    break
                if len(data) >= self.JOINT_DATA_OFFSET + self.JOINT_DATA_SIZE:
                    joint_bytes = data[self.JOINT_DATA_OFFSET : self.JOINT_DATA_OFFSET + self.JOINT_DATA_SIZE]
                    self.double_value = list(struct.unpack(self.FORMAT_SOCKET, joint_bytes))
                    # Optional: uncomment for debugging
                    print(f"Received joint positions: {self.double_value}")
                else:
                    print(f"Received packet too short: {len(data)} bytes")
                time.sleep(0.01)
            except Exception as e:
                if self.running:
                    print(f"Error in socket receive thread: {e}")
                break
        self.running = False

    #通过TCP发送给对应的设备
    def run_zmq_resv(self):
        """ZMQ REP server to handle control commands."""
        context = zmq.Context()
        socket_zmq = context.socket(zmq.REP)
        try:
            socket_zmq.bind(f"tcp://*:{ZMQ_PORT}")
            print(f"ZMQ server listening on port {ZMQ_PORT}")
        except Exception as e:
            print(f"Failed to bind ZMQ socket: {e}")
            return

        while self.running:
            try:
                message = socket_zmq.recv()
                result = TestMsg().unpack(message)
                print(f"Received ZMQ command: cmd='{result.cmd}', data={result.data}")

                if result.cmd == 'movej':
                    data_str = ','.join(f"{x:.6f}" for x in result.data)
                    stringdata = f"movej({{{data_str}}},100,80,100,0,0)"
                    self.send_cmd(stringdata)
                    print(f"Sent to torso: {stringdata}")

                elif result.cmd == 'poweroff':
                    stringdata = "robotPowerOff()"
                    self.send_cmd(stringdata)
                    print("Sent: poweroff (emergency stop)")

                elif result.cmd == 'poweron':
                    stringdata = "robotPowerOn()"
                    self.send_cmd(stringdata)
                    print("Sent: poweron")

                elif result.cmd == 'enable':
                    stringdata = "robotReleaseBrake()"  # Fixed typo: was 'stingdata'
                    self.send_cmd(stringdata)
                    print("Sent: enable (release brake)")

                elif result.cmd == 'get_pos':
                    print("Received get_pos request, returning current joint state.")

                else:
                    print(f"Unknown command: {result.cmd}")

                # Always reply with current joint positions
                reply_msg = TestMsg(cmd='real_pos', data=self.double_value)
                socket_zmq.send(reply_msg.pack())

            except Exception as e:
                if self.running:
                    print(f"ZMQ error: {e}")

        socket_zmq.close()
        context.term()

    def stop(self):
        """Gracefully shut down."""
        print("Shutting down TorsoController...")
        self.running = False
        if self.client_socket:
            try:
                self.client_socket.shutdown(socket.SHUT_RDWR)
            except:
                pass
            self.client_socket.close()
def main():
    torso_control = TorsoController()
    zmq_thread = threading.Thread(target=torso_control.run_zmq_resv, daemon=True)
    zmq_thread.start()

    try:
        # Keep main thread alive
        while zmq_thread.is_alive():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nReceived KeyboardInterrupt, shutting down...")
    finally:
        torso_control.stop()
        zmq_thread.join(timeout=2)
        print("TorsoController exited.")


if __name__ == "__main__":
    main()