import select
import socket
import threading
import traceback
from .robot_client_server_utils import *
import pickle

# Fire-and-forget commands that carry the full, absolute state each time and can
# therefore be coalesced: if newer ones are already buffered, older ones are stale.
_COALESCE_CMDS = {"move"}
class ModelServer:
    def __init__(self, model, host="0.0.0.0", port=None):
        self.model = model
        self.host = host
        self.port = port
        self.server_socket = None
        self.running = False
        self.wait_interval = 10
        self.client_threads = []

    def start(self):
        """Start the model server and listen for incoming client connections"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.settimeout(self.wait_interval)
        self.server_socket.listen(5)
        self.running = True

        print(f"🚀 Model server started on {self.host}:{self.port}")
        print("🔄 Server is waiting for client connections...")

        self._accept_connections()

    def stop(self):
        """Stop the server and clean up resources gracefully"""
        self.running = False
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        for t in self.client_threads:
            t.join(timeout=1)
        print("🛑 Server has been stopped")

    def _accept_connections(self):
        """Accept and handle new client connections"""
        while self.running:
            try:
                client_socket, addr = self.server_socket.accept()
                # Match the client: no Nagle batching on the teleop command stream.
                try:
                    client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                except OSError:
                    pass
                print(f"✅ Client connected from {addr}")
                # Handle each client in a separate thread
                t = threading.Thread(target=self._handle_client, args=(client_socket,), daemon=True)
                t.start()
                self.client_threads.append(t)
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"⚠️ Error accepting connection: {e}")
                break

    @staticmethod
    def _recv_exact(sock, n):
        """Read exactly n bytes, or return None if the peer closed the stream."""
        chunks = []
        remaining = n
        while remaining > 0:
            chunk = sock.recv(min(remaining, 4096))
            if not chunk:
                return None
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _read_frame(self, sock):
        """Read one length-prefixed message and return the parsed dict (or None)."""
        len_bytes = self._recv_exact(sock, 4)
        if not len_bytes:
            return None
        msg_length = int.from_bytes(len_bytes, "big")
        raw = self._recv_exact(sock, msg_length)
        if raw is None:
            return None
        return json_to_numpy(raw.decode("utf-8"))

    def _coalesce_latest(self, sock, data):
        """Drop stale buffered coalescable commands, returning (latest, pending).

        While more bytes are already buffered, keep reading frames. Consecutive
        coalescable commands (e.g. ``move``) replace the previous one (latest-wins).
        The first non-coalescable frame is returned as ``pending`` to be handled
        next, preserving ordering for events like reset/finish/halt.
        """
        pending = None
        while select.select([sock], [], [], 0)[0]:
            nxt = self._read_frame(sock)
            if nxt is None:
                break
            if bool(nxt.get("no_reply")) and nxt.get("cmd") in _COALESCE_CMDS:
                data = nxt
                continue
            pending = nxt
            break
        return data, pending

    def _handle_client(self, client_socket):
        """Process requests from a single client"""
        with client_socket:
            pending = None
            while self.running:
                # When the payload carries no_reply=True (high-rate fire-and-forget
                # commands such as move/halt) the client never reads a reply. Sending
                # one anyway would accumulate unread bytes in the socket buffer and
                # desync every later request/response call, so track it per message.
                no_reply = False
                try:
                    data = pending if pending is not None else self._read_frame(client_socket)
                    pending = None
                    if data is None:
                        print("🔌 Client disconnected")
                        break

                    # Latest-wins: if newer moves are already queued behind this one,
                    # the robot should act on the freshest intent, not replay a stale
                    # backlog. This keeps teleop latency bounded to ~one processing
                    # period even when the consumer is slower than the 100Hz producer
                    # (e.g. both grips engaged -> dual-arm apply_teleop).
                    if bool(data.get("no_reply")) and data.get("cmd") in _COALESCE_CMDS:
                        data, pending = self._coalesce_latest(client_socket, data)

                    # Extract command and observation
                    cmd = data.get("cmd")
                    obs = data.get("obs")  # None if not provided
                    # move is fire-and-forget on the master side; do not reply or the
                    # TCP stream desyncs and later RPCs (start/finish) fail JSON decode.
                    no_reply = data.get("no_reply") is True or cmd == "move"

                    # Find corresponding model method
                    method = getattr(self.model, cmd, None)
                    if not callable(method):
                        raise AttributeError(f"No model method named '{cmd}'")

                    try:
                        result = method(obs) if obs is not None else method()
                    except Exception as e:
                        if no_reply:
                            print(f"⚠️ Error handling one-way request '{cmd}': {e}")
                            traceback.print_exc()
                            continue
                        raise

                    if no_reply:
                        continue

                    response = {"res": result}

                    # Serialize response and send back with length header
                    resp_bytes = numpy_to_json(response).encode("utf-8")
                    client_socket.sendall(len(resp_bytes).to_bytes(4, "big"))
                    client_socket.sendall(resp_bytes)

                except (ConnectionResetError, BrokenPipeError):
                    print("🔌 Client connection lost")
                    break
                except Exception as e:
                    err = f"Error handling request: {e}"
                    print(f"⚠️ {err}")
                    tb = traceback.format_exc()
                    if not no_reply:
                        error_resp = numpy_to_json({"error": err, "traceback": tb}).encode("utf-8")
                        client_socket.sendall(len(error_resp).to_bytes(4, "big"))
                        client_socket.sendall(error_resp)
                    break
