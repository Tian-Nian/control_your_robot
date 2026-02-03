from robot.utils.node.node import Node
from threading import Event, Thread
from typing import List, Optional
import time


class Scheduler:
    def __init__(
        self,
        entry_nodes: List[Node],
        all_nodes: List[Node],
        final_nodes: List[Node],   # ⭐ 改成 list
        hz: float = 5.0,
    ):
        self.entry_nodes = entry_nodes
        self.all_nodes = all_nodes
        self.final_nodes = final_nodes

        self.period = 1.0 / hz
        self._stop_event = Event()
        self._thread: Optional[Thread] = None

        self._running_episode = False

    def start(self):
        self._thread = Thread(
            target=self._run,
            name="Scheduler",
            daemon=True
        )
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        for node in self.all_nodes:
            node.stop()

    def _run(self):
        print(f"[SCHED] start @ {1/self.period:.1f} Hz")

        next_tick = time.monotonic()

        while not self._stop_event.is_set():
            now = time.monotonic()

            # ===== episode 正在运行 =====
            if self._running_episode:
                if self._all_final_nodes_done():
                    # print("[SCHED] episode finished\n")
                    self._running_episode = False
                    self._reset_all_nodes()
                time.sleep(0.001)
                continue

            # ===== idle，等 tick =====
            if now >= next_tick:
                # print("[SCHED] start new episode")
                self._trigger_entry_nodes()
                self._running_episode = True
                next_tick += self.period

            time.sleep(0.00001)

        print("[SCHED] stopped")

    # ======================================================
    # Helpers
    # ======================================================

    def _all_final_nodes_done(self) -> bool:
        return all(n.end_event.is_set() for n in self.final_nodes)

    def _trigger_entry_nodes(self):
        for n in self.entry_nodes:
            n.forward_event.set()

    def _reset_all_nodes(self):
        for n in self.all_nodes:
            n.reset()
