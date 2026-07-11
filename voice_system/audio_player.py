"""Thread-safe, non-blocking MP3 playback.

The preferred backend is pygame.mixer. On macOS, if pygame cannot access
CoreAudio from the current terminal session, the player falls back to `afplay`
through subprocess.Popen, which is still non-blocking and avoids os.system.
"""

from __future__ import annotations

import logging
import platform
import subprocess
import threading
from pathlib import Path
from typing import Optional

import pygame

logger = logging.getLogger(__name__)


class AudioPlayer:
    """Small non-blocking audio player wrapper around pygame.mixer.music."""

    def __init__(self, frequency: int = 24_000, buffer: int = 512) -> None:
        self._lock = threading.RLock()
        self._initialized = False
        self._backend = "pygame"
        self._current_file: Optional[Path] = None
        self._process: Optional[subprocess.Popen[bytes]] = None
        self._frequency = frequency
        self._buffer = buffer
        self._init_mixer()

    def _init_mixer(self) -> None:
        with self._lock:
            if self._initialized:
                return
            try:
                pygame.mixer.init(frequency=self._frequency, buffer=self._buffer)
                self._initialized = True
                self._backend = "pygame"
                logger.info("Audio mixer initialized")
            except Exception:
                if platform.system() == "Darwin":
                    self._backend = "afplay"
                    self._initialized = True
                    logger.warning("pygame mixer unavailable; falling back to non-blocking afplay")
                else:
                    logger.exception("Failed to initialize pygame mixer")
                    self._initialized = False

    def play(self, file_path: str | Path, *, interrupt: bool = True) -> bool:
        """Play an audio file asynchronously.

        Args:
            file_path: MP3/wav file to play.
            interrupt: If true, stop current audio before starting this one.

        Returns:
            True if playback was handed to the mixer, false otherwise.
        """
        path = Path(file_path)
        if not path.exists():
            logger.warning("Audio file not found: %s", path)
            return False

        with self._lock:
            if not self._initialized:
                self._init_mixer()
            if not self._initialized:
                return False

            try:
                if self._backend == "afplay":
                    if interrupt:
                        self._stop_process_locked()
                    elif self._process and self._process.poll() is None:
                        logger.debug("Audio busy, skip non-interrupting play: %s", path)
                        return False
                    self._process = subprocess.Popen(
                        ["afplay", str(path)],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                    self._current_file = path
                    logger.debug("Playing audio with afplay: %s", path)
                    return True

                if interrupt and pygame.mixer.music.get_busy():
                    pygame.mixer.music.stop()
                elif not interrupt and pygame.mixer.music.get_busy():
                    logger.debug("Audio busy, skip non-interrupting play: %s", path)
                    return False

                pygame.mixer.music.load(str(path))
                pygame.mixer.music.play()
                self._current_file = path
                logger.debug("Playing audio: %s", path)
                return True
            except Exception:
                logger.exception("Failed to play audio: %s", path)
                return False

    def is_playing(self) -> bool:
        """Return whether the mixer is currently playing audio."""
        with self._lock:
            if not self._initialized:
                return False
            try:
                if self._backend == "afplay":
                    return bool(self._process and self._process.poll() is None)
                return bool(pygame.mixer.music.get_busy())
            except Exception:
                logger.exception("Failed to query audio state")
                return False

    def stop(self) -> None:
        """Stop current playback immediately."""
        with self._lock:
            if not self._initialized:
                return
            try:
                if self._backend == "afplay":
                    self._stop_process_locked()
                    logger.debug("Audio stopped")
                    return
                pygame.mixer.music.stop()
                logger.debug("Audio stopped")
            except Exception:
                logger.exception("Failed to stop audio")

    def shutdown(self) -> None:
        """Release pygame mixer resources."""
        with self._lock:
            if not self._initialized:
                return
            try:
                if self._backend == "afplay":
                    self._stop_process_locked()
                else:
                    pygame.mixer.music.stop()
                    pygame.mixer.quit()
            except Exception:
                logger.exception("Failed to shutdown audio mixer")
            finally:
                self._initialized = False

    def _stop_process_locked(self) -> None:
        if not self._process or self._process.poll() is not None:
            return
        self._process.terminate()
        try:
            self._process.wait(timeout=0.2)
        except subprocess.TimeoutExpired:
            self._process.kill()
            self._process.wait(timeout=0.2)
