import zmq
import struct

class CmdMsg:
    FORMAT = '=10s'

    def __init__(self, cmd='', data=None, data_length=0, data_format='d'):
        self.cmd = cmd.encode('utf-8')[:10].ljust(10, b'\x00')
        self.data = data if data is not None else [0] * data_length
        self.FORMAT += str(data_length) + data_format 

    def pack(self):
        return struct.pack(self.FORMAT, self.cmd, *self.data)
    
    def unpack(self, binary_data):
        self.cmd, *self.data = struct.unpack(self.FORMAT, binary_data)
        self.cmd = self.cmd.decode('utf-8').strip('\x00')
        return self

class ArmCmdMsg(CmdMsg):
    def __init__(self, cmd='', data=None):
        super().__init__(cmd, data, data_length=7, data_format='d')

class HandCmdMsg(CmdMsg):
    def __init__(self, cmd='', data=None):
        super().__init__(cmd, data, data_length=6, data_format='i')

class HeadCmdMsg(CmdMsg):
    def __init__(self, cmd='', data=None):
        super().__init__(cmd, data, data_length=2, data_format='d')

class TorsoCmdMsg(CmdMsg):
    def __init__(self, cmd='', data=None):
        super().__init__(cmd, data, data_length=6, data_format='d')

################################################################

class ZMQClient:
    def __init__(self, server_address):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(server_address)

    def send_request(self, msg):
        try:
            binary_data = msg.pack()
            self.socket.send(binary_data)
            
            # 设置接收超时，避免永久阻塞
            poller = zmq.Poller()
            poller.register(self.socket, zmq.POLLIN)
            
            if poller.poll(timeout=5000):  # 5秒超时
                response = self.socket.recv()
                # logger.info(f"Received response: {response}")
                
                # 解析响应数据
                result = msg.__class__().unpack(response)
                return result.data
            else:
                print("[ZMQClient] Timeout waiting for server response.")
                return None
                
        except zmq.ZMQError as e:
            print(f"ZMQ Error: {e}")
            return None
        except Exception as e:
            print(f"Unexpected error: {e}")
            return None

    def close(self):
        self.socket.close()
        self.context.term()