"""Cancellation with a recorded first source; never logs terminal input text."""
import threading


class StopSignal(threading.Event):
    def __init__(self):
        super().__init__()
        self.reason = None
        self._reason_lock = threading.Lock()

    def request(self, reason):
        with self._reason_lock:
            if not self.is_set():
                self.reason = reason
            self.set()


def watch_terminal(stream, stop):
    try:
        line = stream.readline()
    except (OSError, ValueError):
        stop.request('stdin_error')
    else:
        stop.request('stdin_eof' if line == '' else 'enter_pressed')
