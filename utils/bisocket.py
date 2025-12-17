import socket
import pickle
from threading import Thread, Event

from utils.data_handler import debug_print

class BiSocket:
    '''
    用于同步client-server信息的类
    '''
    def __init__(self, conn: socket.socket, handler, send_back=False, enable_loop=True):
        '''
        输入:
        conn: 用于通讯的套接字, 要先初始化套接字连接的(ip, port), socket::socket
        handler: 会执行该函数函数, 函数输入为Dict[Any], function
        sendback: 如果开启senback就会在执行完handler后将当前执行的信息发给信息发送方, bool
        '''
        self.conn = conn
        self.handler = handler
        self.send_back = send_back
        self.running = Event()
        self.running.set()

        if enable_loop:
            self.receiver_thread = Thread(target=self._recv_loop, daemon=True)
            self.receiver_thread.start()
        else:
            self.receiver_thread = None

    def _recv_exact(self, n):
        data = b''
        while len(data) < n:
            try:
                packet = self.conn.recv(n - len(data))
            except Exception as e:
                debug_print("BiSocket", f"Recv error: {e}", "ERROR")
                self.close()
                return None
            if not packet:
                debug_print("BiSocket","Remote side closed connection.", "WARNING")
                self.close()
                return None
            data += packet
        return data

    def _recv_loop(self):
        try:
            while self.running.is_set():
                length_bytes = self._recv_exact(4)
                if length_bytes is None:
                    break

                length = int.from_bytes(length_bytes, 'big')
                data = self._recv_exact(length)
                if data is None:
                    break

                try:
                    message = pickle.loads(data)
                except Exception as e:
                    debug_print("BiSocket",f"Unpickle error: {e}", "WARNING")
                    continue

                if self.send_back:
                    try:
                        reply = self.handler(message)
                        self.send(reply)
                        
                        debug_print("BiSocket","Sent back response.", "DEBUG")
                    except Exception as e:
                        debug_print("BiSocket",f"Handler/send_back error: {e}", "ERROR")
                else:
                    try:
                        if message is not None:
                            self.handler(message)
                    except Exception as e:
                        debug_print("BiSocket",f"Handler error: {e}", "ERROR")
        finally:
            self.close()

    
    def send(self, data):
        '''
        发送信息:
        data: 发送的信息, Dict[Any]
        '''
        try:
            serialized = pickle.dumps(data)
            self.conn.sendall(len(serialized).to_bytes(4, 'big') + serialized)
        except Exception as e:
            debug_print("BiSocket",f"Send failed: {e}", "ERROR")
            self.close()
    
    def send_and_wait_reply(self, data, timeout: float = 5.0):
        '''
        阻塞式发送信息并等待对方回复（适合 Client 端使用）

        参数:
            data: 要发送的数据 (Dict[Any])
            timeout: 等待回复的最大秒数，float

        返回:
            对方回复的数据 (Dict[Any])，若超时或出错返回 None
        '''
        try:
            # 序列化并发送
            serialized = pickle.dumps(data)
            self.conn.sendall(len(serialized).to_bytes(4, 'big') + serialized)
            debug_print("BiSocket", f"Sent {len(serialized)} bytes, waiting for reply...", "DEBUG")

            # 设置 socket 超时
            self.conn.settimeout(timeout)

            # 读取返回包头（4 字节长度）
            length_bytes = self._recv_exact(4)
            if length_bytes is None:
                debug_print("BiSocket", "No reply length header received.", "WARNING")
                return None

            length = int.from_bytes(length_bytes, 'big')

            # 读取返回主体
            data_bytes = self._recv_exact(length)
            if data_bytes is None:
                debug_print("BiSocket", "No reply data received.", "WARNING")
                return None

            # 反序列化
            reply = pickle.loads(data_bytes)
            debug_print("BiSocket", f"Reply received successfully ({len(data_bytes)} bytes).", "DEBUG")

            # 若定义了 handler，可自动执行（例如 client.move）
            if self.handler:
                try:
                    self.handler(reply)
                except Exception as e:
                    debug_print("BiSocket", f"Handler (after reply) error: {e}", "ERROR")

            return reply

        except socket.timeout:
            debug_print("BiSocket", "Timeout waiting for reply.", "WARNING")
            return None

        except Exception as e:
            debug_print("BiSocket", f"Send/wait error: {e}", "ERROR")
            self.close()
            return None

        finally:
            # 恢复非超时模式
            try:
                self.conn.settimeout(None)
            except Exception:
                pass
    
    def close(self):
        '''
        关闭连接
        '''
        if self.running.is_set():
            self.running.clear()
            try:
                self.conn.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            self.conn.close()
            debug_print("BiSocket","Connection closed.", "INFO")