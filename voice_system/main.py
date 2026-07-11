"""Main entrypoint and public facade for the desktop air-hockey voice system."""

from __future__ import annotations

import logging
import select
import sys
import threading
import time
from pathlib import Path
from typing import Optional

try:
    import termios
    import tty
    HAS_TTY = True
except ImportError:
    HAS_TTY = False

try:
    from dotenv import load_dotenv
except ImportError:  # python-dotenv is optional at runtime.
    load_dotenv = None

try:
    from .chat_engine import ChatEngine
    from .state_machine import MODE_CHAT, VoiceStateMachine
    from .tts_manager import TTSManager
    from .asr_service import AsrConfig, AsrService
    from .mic_capture import MicCapture
except ImportError:  # Allows `python main.py` inside voice_system/.
    from chat_engine import ChatEngine
    from state_machine import MODE_CHAT, VoiceStateMachine
    from tts_manager import TTSManager
    from asr_service import AsrConfig, AsrService
    from mic_capture import MicCapture


logger = logging.getLogger(__name__)

EVENT_KEY_MAP = {
    "g": "human_goal",
    "G": "ai_goal",
    "m": "human_miss",
    "d": "defense_success",
    "i": "idle",
    "s": "start",
    "e": "end",
}


class VoiceSystem:
    """Public API used by the game loop."""

    def __init__(
        self,
        tts_manager: Optional[TTSManager] = None,
        chat_engine: Optional[ChatEngine] = None,
    ) -> None:
        if load_dotenv:
            load_dotenv()

        self.tts = tts_manager or TTSManager()
        self.chat = chat_engine or ChatEngine()
        self._chat_lock = threading.RLock()
        self._speaking = False  # True while AI is outputting audio
        self._speaking_ended_at = 0.0  # timestamp when speaking stopped

        self.state_machine = VoiceStateMachine(
            on_trash_event=self._on_trash_event,
            on_chat_enter=self._on_chat_enter,
            on_chat_exit=self._on_chat_exit,
        )

    @property
    def is_speaking(self) -> bool:
        """True when AI is playing audio or TTS queue has pending items."""
        return self._speaking

    @property
    def mode(self):
        """Return current voice mode."""
        return self.state_machine.mode

    def trigger_event(self, event_type: str) -> None:
        """Trigger a game event: human_goal/ai_goal/human_miss/defense_success/idle/start/end."""
        self.state_machine.trigger_event(event_type)

    def user_wake_up(self) -> None:
        """Enter chat mode after wake word detection."""
        self.state_machine.user_wake_up()

    def user_say(self, text: str) -> None:
        """Handle user text from ASR or keyboard simulation."""
        self.state_machine.user_say(text)
        if self.mode != MODE_CHAT or "别聊了" in text:
            return

        threading.Thread(
            target=self._reply_to_user_streaming,
            args=(text,),
            name="voice-chat-reply",
            daemon=True,
        ).start()

    def puck_hit(self) -> None:
        """Notify the voice system that the puck was hit."""
        self.state_machine.puck_hit()

    def tick(self) -> None:
        """Advance internal timers; call in the main game loop."""
        self.state_machine.tick()
        # Reset speaking flag only when TTS is fully idle
        if self._speaking:
            if not self.tts.is_busy and not self.tts.player.is_playing():
                self._speaking = False
                self._speaking_ended_at = time.monotonic()

    def shutdown(self) -> None:
        """Release resources."""
        self.tts.shutdown()

    def _on_trash_event(self, event_type: str) -> None:
        logger.info("Play trash talk for event=%s", event_type)
        self._speaking = True
        self.tts.speak_trash(event_type)

    def _on_chat_enter(self, user_triggered: bool) -> None:
        logger.info("Enter CHAT, user_triggered=%s", user_triggered)
        self.tts.stop()
        self._speaking = True
        if not user_triggered:
            opening = self.chat.proactive_opening()
            self.tts.speak_chat(opening)
        else:
            self.tts.speak_transition("来了，想聊啥")

    def _on_chat_exit(self) -> None:
        logger.info("Exit CHAT")
        self.tts.stop()
        self.chat.reset_history()

    def _reply_to_user_streaming(self, text: str) -> None:
        """Stream LLM reply → sentence-by-sentence TTS with minimal latency.

        Flow: DeepSeek stream tokens → split on punctuation → each sentence
        immediately sent to edge-tts streaming playback.
        """
        with self._chat_lock:
            if self.mode != MODE_CHAT:
                return

            self._speaking = True
            try:
                for sentence in self.chat.reply_stream(text):
                    # Check if we're still in chat mode (user might have said "别聊了")
                    if self.mode != MODE_CHAT:
                        logger.info("Chat mode exited during streaming reply, stopping")
                        break
                    logger.info("Streaming sentence to TTS: %s", sentence)
                    self.tts.speak_chat_streaming(sentence)
            except Exception:
                logger.exception("Streaming reply failed")


def configure_logging() -> None:
    """Configure readable console logs."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
    )


def _read_line(prompt: str) -> str:
    sys.stdout.write(prompt)
    sys.stdout.flush()
    return sys.stdin.readline().strip()


def _read_key(timeout: float = 0.05) -> Optional[str]:
    """Read one key in cbreak mode, returning None on timeout."""
    readable, _, _ = select.select([sys.stdin], [], [], timeout)
    if not readable:
        return None
    return sys.stdin.read(1)


def run_keyboard_test() -> None:
    """Keyboard simulation mode for development without real game hardware."""
    if not HAS_TTY:
        logger.error("Keyboard test requires a Unix TTY (termios). Use --mode voice on Windows.")
        return

    configure_logging()
    system = VoiceSystem()
    logger.info("VoiceSystem keyboard test started")
    logger.info("Keys: g human goal, G AI goal, m miss, d defense, i idle, c wake, q stop chat, h puck hit, t text, x exit")
    logger.info("Press keys directly. For t, type text then Enter.")

    old_tty = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            system.tick()
            key = _read_key(timeout=0.05)
            if key is None:
                continue

            if key == "x":
                break
            if key in EVENT_KEY_MAP:
                system.trigger_event(EVENT_KEY_MAP[key])
            elif key == "c":
                system.user_wake_up()
            elif key == "q":
                system.user_say("别聊了")
            elif key == "h":
                system.puck_hit()
            elif key == "t":
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_tty)
                text = _read_line("你说：")
                tty.setcbreak(sys.stdin.fileno())
                system.user_say(text)
            else:
                logger.info("Unknown key: %s", key)

            system.tick()
    except KeyboardInterrupt:
        logger.info("Keyboard test interrupted")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_tty)
        system.shutdown()


# ---------------------------------------------------------------------------
# Voice mode: microphone → ASR → VoiceSystem (replaces keyboard simulation)
# ---------------------------------------------------------------------------

WAKE_WORDS = {"喂", "哎", "嘿", "你好"}

# Debounce: wait this long after last ASR final before triggering reply
DEBOUNCE_SECONDS = 1.0


def run_voice_mode() -> None:
    """Production voice mode: mic capture → real-time ASR → VoiceSystem.

    This replaces the keyboard test for actual hardware deployment.
    Game events (goals, misses, etc.) are still triggered via the public
    API (trigger_event / puck_hit) by the game controller externally.
    """
    configure_logging()
    system = VoiceSystem()

    # --- ASR setup ---
    asr_config = AsrConfig()
    asr_service = AsrService(config=asr_config)

    # --- Debounce state ---
    _debounce_lock = threading.Lock()
    _debounce_text = [""]        # accumulates the latest final text
    _debounce_timer: list = [None]  # holds the Timer object

    def _debounce_fire() -> None:
        """Called after DEBOUNCE_SECONDS of silence. Process the accumulated text."""
        with _debounce_lock:
            text = _debounce_text[0].strip()
            _debounce_text[0] = ""
            _debounce_timer[0] = None
        if text:
            logger.info("Debounced ASR final: %s", text)
            _handle_recognized_text(system, text)

    def on_sentence(text: str, is_final: bool) -> None:
        """Called by ASR when a sentence (interim or final) is recognized."""
        text = text.strip()
        if not text:
            return

        # --- Echo suppression: ignore ASR while AI is speaking ---
        if system.is_speaking:
            logger.debug("ASR suppressed (AI speaking): %s", text)
            return

        # --- Ignore residual echo for 2s after AI stops speaking ---
        if time.monotonic() - system._speaking_ended_at < 2.0:
            logger.debug("ASR suppressed (post-speech buffer): %s", text)
            return

        if is_final:
            # --- Debounce: reset timer on each final, only fire after silence ---
            with _debounce_lock:
                _debounce_text[0] = text  # always use latest complete text
                # Cancel previous timer
                if _debounce_timer[0] is not None:
                    _debounce_timer[0].cancel()
                # Start new timer
                _debounce_timer[0] = threading.Timer(DEBOUNCE_SECONDS, _debounce_fire)
                _debounce_timer[0].daemon = True
                _debounce_timer[0].start()
        else:
            logger.debug("ASR interim: %s", text)

    def on_error(msg: str) -> None:
        logger.error("ASR error: %s", msg)

    asr_session = asr_service.create_session(on_sentence=on_sentence, on_error=on_error)

    # --- Microphone setup ---
    mic = MicCapture(on_audio=asr_session.send_audio)

    # --- Start ---
    logger.info("Starting voice mode (mic → ASR → VoiceSystem)")
    logger.info("Speak to interact. Ctrl+C to exit.")
    asr_session.start()
    mic.start()

    try:
        while True:
            system.tick()
            time.sleep(0.05)
    except KeyboardInterrupt:
        logger.info("Voice mode interrupted")
    finally:
        # Cancel any pending debounce timer
        with _debounce_lock:
            if _debounce_timer[0] is not None:
                _debounce_timer[0].cancel()
        mic.stop()
        asr_session.stop()
        system.shutdown()
        logger.info("Voice mode shutdown complete")


def _handle_recognized_text(system: "VoiceSystem", text: str) -> None:
    """Route recognized text to VoiceSystem based on current mode."""
    # Check for wake words when not in chat mode
    if system.mode != MODE_CHAT:
        for word in WAKE_WORDS:
            if word in text:
                system.user_wake_up()
                # If there's more content beyond the wake word, treat it as speech
                remainder = text
                for w in WAKE_WORDS:
                    remainder = remainder.replace(w, "").strip()
                if remainder:
                    system.user_say(remainder)
                return
        # Not in chat mode and no wake word → still enter chat (any speech activates)
        system.user_wake_up()
        system.user_say(text)
        return

    # In chat mode, forward text directly
    system.user_say(text)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Air-hockey voice system")
    parser.add_argument(
        "--mode",
        choices=["voice", "keyboard"],
        default="voice",
        help="Run mode: 'voice' for mic+ASR (default), 'keyboard' for dev simulation",
    )
    args = parser.parse_args()

    if args.mode == "keyboard":
        run_keyboard_test()
    else:
        run_voice_mode()
