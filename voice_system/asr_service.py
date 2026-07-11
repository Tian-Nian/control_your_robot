"""ASR service using DashScope Recognition SDK (fun-asr-realtime).

This is the Python equivalent of the Java AsrService, directly using
the DashScope Python SDK for real-time streaming speech recognition.
Model and parameters are kept identical to the original Java project.
"""

from __future__ import annotations

import logging
import os
import threading
from typing import Callable, Optional

import dashscope
from dashscope.audio.asr import (
    Recognition,
    RecognitionCallback,
    RecognitionResult,
)

logger = logging.getLogger(__name__)


class AsrConfig:
    """ASR configuration mirroring the Java application.yml."""

    def __init__(
        self,
        api_key: Optional[str] = None,
        workspace_id: Optional[str] = None,
        model: str = "paraformer-realtime-v2",
        format: str = "pcm",
        sample_rate: int = 16000,
    ) -> None:
        self.api_key = api_key or os.getenv("DASHSCOPE_API_KEY", "")
        self.workspace_id = workspace_id or os.getenv("DASHSCOPE_WORKSPACE_ID", "")
        self.model = model
        self.format = format
        self.sample_rate = sample_rate


class AsrService:
    """Manages real-time ASR sessions using DashScope Recognition.

    Mirrors the Java AsrService: creates a Recognition instance,
    streams PCM audio frames, and delivers transcription results
    via callbacks.
    """

    def __init__(self, config: Optional[AsrConfig] = None) -> None:
        self.config = config or AsrConfig()
        if self.config.api_key:
            dashscope.api_key = self.config.api_key

    def create_session(
        self,
        on_sentence: Callable[[str, bool], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> "AsrSession":
        """Create a new ASR session.

        Args:
            on_sentence: callback(text, is_final) called on each recognition result.
            on_error: optional callback(error_message) on failure.
        """
        return AsrSession(self.config, on_sentence, on_error)


class _RecognitionCallback(RecognitionCallback):
    """Internal callback adapter that forwards results to user-provided callables."""

    def __init__(
        self,
        on_sentence: Callable[[str, bool], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._on_sentence = on_sentence
        self._on_error = on_error

    def on_open(self) -> None:
        logger.info("ASR connection opened")

    def on_close(self) -> None:
        logger.info("ASR connection closed")

    def on_event(self, result: RecognitionResult) -> None:
        sentence = result.get_sentence()
        if sentence is None:
            return
        text = sentence.get("text", "")
        # In the Java version: stash_result → interim, non-stash → final
        is_final = "end_time" in sentence
        if text:
            self._on_sentence(text, is_final)

    def on_error(self, result: RecognitionResult) -> None:
        msg = str(result)
        logger.error("ASR error: %s", msg)
        if self._on_error:
            self._on_error(msg)

    def on_complete(self) -> None:
        logger.info("ASR recognition complete")


class AsrSession:
    """A single ASR recognition session that accepts audio frames."""

    def __init__(
        self,
        config: AsrConfig,
        on_sentence: Callable[[str, bool], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._config = config
        self._callback = _RecognitionCallback(on_sentence, on_error)
        self._recognition: Optional[Recognition] = None
        self._started = False
        self._lock = threading.Lock()

    def start(self) -> None:
        """Start the recognition session (connects to DashScope)."""
        with self._lock:
            if self._started:
                return

            self._recognition = Recognition(
                model=self._config.model,
                format=self._config.format,
                sample_rate=self._config.sample_rate,
                callback=self._callback,
            )
            self._recognition.start()
            self._started = True
            logger.info(
                "ASR session started (model=%s, format=%s, sample_rate=%d)",
                self._config.model,
                self._config.format,
                self._config.sample_rate,
            )

    def send_audio(self, audio_data: bytes) -> None:
        """Send a chunk of PCM audio data to the recognizer."""
        with self._lock:
            if self._started and self._recognition:
                self._recognition.send_audio_frame(audio_data)

    def stop(self) -> None:
        """Stop the recognition session gracefully."""
        with self._lock:
            if not self._started:
                return
            self._started = False
            if self._recognition:
                try:
                    self._recognition.stop()
                except Exception:
                    logger.exception("Error stopping ASR session")
                self._recognition = None
            logger.info("ASR session stopped")
