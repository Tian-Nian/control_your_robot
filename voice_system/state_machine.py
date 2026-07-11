"""Voice interaction state machine for battle, trash-talk, and chat modes."""

from __future__ import annotations

import logging
import random
import threading
import time
from enum import Enum
from typing import Callable, List, Optional

logger = logging.getLogger(__name__)


class VoiceMode(str, Enum):
    """Voice system modes."""

    BATTLE = "MODE_BATTLE"
    TRASH = "MODE_TRASH"
    CHAT = "MODE_CHAT"


MODE_BATTLE = VoiceMode.BATTLE
MODE_TRASH = VoiceMode.TRASH
MODE_CHAT = VoiceMode.CHAT


class VoiceStateMachine:
    """Manages mode transitions and timing rules for voice behavior."""

    def __init__(
        self,
        on_trash_event: Callable[[str], None],
        on_chat_enter: Callable[[bool], None],
        on_chat_exit: Callable[[], None],
        trash_cooldown_range: tuple[float, float] = (5.0, 8.0),
        chat_idle_timeout: float = 30.0,
        proactive_idle_seconds: float = 10.0,
        proactive_probability: float = 0.7,
    ) -> None:
        self.mode = MODE_BATTLE
        self.on_trash_event = on_trash_event
        self.on_chat_enter = on_chat_enter
        self.on_chat_exit = on_chat_exit
        self.trash_cooldown_range = trash_cooldown_range
        self.chat_idle_timeout = chat_idle_timeout
        self.proactive_idle_seconds = proactive_idle_seconds
        self.proactive_probability = proactive_probability

        self._lock = threading.RLock()
        self._trash_until = 0.0
        self._last_activity = time.monotonic()
        self._last_user_speech = time.monotonic()
        self._queued_trash_events: List[str] = []

    def trigger_event(self, event_type: str) -> None:
        """Handle a game event such as goal/miss/defense/idle."""
        with self._lock:
            self._last_activity = time.monotonic()
            if self.mode == MODE_CHAT:
                self._queued_trash_events.append(event_type)
                logger.info("Queue trash event during CHAT: %s", event_type)
                return

            if self.mode == MODE_TRASH:
                logger.debug("Ignore trash event during cooldown: %s", event_type)
                return

            self._enter_trash(event_type)

    def user_wake_up(self) -> None:
        """Enter chat when the user says a wake word such as '喂' or '哎'."""
        with self._lock:
            if self.mode == MODE_CHAT:
                self._last_user_speech = time.monotonic()
                return
            self._enter_chat(user_triggered=True)

    def user_say(self, text: str) -> None:
        """Update user activity and exit chat on the stop phrase."""
        with self._lock:
            self._last_user_speech = time.monotonic()
            self._last_activity = time.monotonic()
            if "别聊了" in text:
                self.exit_chat("user_stop_phrase")

    def puck_hit(self) -> None:
        """Return to battle when the puck is hit."""
        with self._lock:
            self._last_activity = time.monotonic()
            if self.mode == MODE_TRASH:
                self._set_mode(MODE_BATTLE, "puck_hit")
            elif self.mode == MODE_CHAT:
                self.exit_chat("puck_hit")

    def tick(self) -> None:
        """Advance time-based transitions; call once per game loop iteration."""
        with self._lock:
            now = time.monotonic()
            if self.mode == MODE_TRASH and now >= self._trash_until:
                self._set_mode(MODE_BATTLE, "trash_cooldown_done")
                return

            if self.mode == MODE_CHAT:
                if now - self._last_user_speech >= self.chat_idle_timeout:
                    self.exit_chat("chat_idle_timeout")
                return

            if self.mode == MODE_BATTLE and now - self._last_activity >= self.proactive_idle_seconds:
                self._last_activity = now
                if random.random() < self.proactive_probability:
                    self._enter_chat(user_triggered=False)

    def exit_chat(self, reason: str = "manual") -> None:
        """Exit chat and clear queued trash events without playing them."""
        if self.mode != MODE_CHAT:
            return
        queued_count = len(self._queued_trash_events)
        self._queued_trash_events.clear()
        self.on_chat_exit()
        logger.info("Clear %d queued trash events after CHAT", queued_count)
        self._set_mode(MODE_BATTLE, reason)

    def _enter_trash(self, event_type: str) -> None:
        self._set_mode(MODE_TRASH, f"event:{event_type}")
        self._trash_until = time.monotonic() + random.uniform(*self.trash_cooldown_range)
        self.on_trash_event(event_type)

    def _enter_chat(self, user_triggered: bool) -> None:
        self._queued_trash_events.clear()
        self._last_user_speech = time.monotonic()
        self.on_chat_enter(user_triggered)
        self._set_mode(MODE_CHAT, "user_wake_up" if user_triggered else "proactive_idle")

    def _set_mode(self, new_mode: VoiceMode, reason: str) -> None:
        old_mode = self.mode
        if old_mode == new_mode:
            logger.debug("Mode stay %s, reason=%s", new_mode.value, reason)
            return
        self.mode = new_mode
        logger.info("Voice mode %s -> %s, reason=%s", old_mode.value, new_mode.value, reason)
