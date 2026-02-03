from threading import Event, Thread, Lock
from typing import List, Optional
import time
import random

# ==========================================================
# Base Node
# ==========================================================

class Node:
    def __init__(self, name: str):
        self.name = name

        # fan-in
        self.start_events: List[Event] = []

        # trigger
        self.forward_event = Event()

        # done (broadcast)
        self.end_event = Event()

        # fan-out
        self.next_nodes: List["Node"] = []

        self._thread: Optional[Thread] = None
        self._stop_event = Event()
        self._lock = Lock()

        self._has_run = False  # 单次 episode 执行保证

    # ========= Topology =========
    def next_to(self, next_node: "Node"):
        self.next_nodes.append(next_node)
        next_node.add_start_event(self.end_event)

    def add_start_event(self, event: Event):
        self.start_events.append(event)

    # ========= Lifecycle =========
    def start(self):
        self._thread = Thread(
            target=self._run,
            name=f"Node-{self.name}",
            daemon=True
        )
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self.forward_event.set()

    # ========= Episode control =========
    def reset(self):
        with self._lock:
            self._has_run = False
            self.end_event.clear()
            self.forward_event.clear()

    # ========= Execution =========
    def _ready(self):
        return all(ev.is_set() for ev in self.start_events)

    def _run(self):
        while not self._stop_event.is_set():
            self.forward_event.wait()
            self.forward_event.clear()

            if self._stop_event.is_set():
                break

            with self._lock:
                if self._has_run:
                    continue

                if not self._ready():
                    continue

                self._has_run = True

            # === Execute ===
            self.handler()

            # === Done ===
            self.end_event.set()

            # === Fan-out trigger ===
            for n in self.next_nodes:
                n.forward_event.set()

    # ========= User override =========
    def handler(self):
        raise NotImplementedError

class TaskNode(Node):
    def __init__(self, name: str, **task_kwargs):
        super().__init__(name)
        self.task_kwargs = task_kwargs
        self._inited = False

    def task_init(self, **kwargs):
        pass

    def task_step(self):
        pass

    def handler(self):
        if not self._inited:
            self.task_init(**self.task_kwargs)
            self._inited = True
        self.task_step()