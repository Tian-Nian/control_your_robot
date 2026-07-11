"""Microphone audio capture using sounddevice.

Provides a simple threaded microphone capture that reads PCM frames
and delivers them to a callback. Parameters match the ASR config:
16kHz, mono, 16-bit PCM.
"""

from __future__ import annotations

import logging
import numpy as np
import threading
from typing import Callable, Optional

import sounddevice as sd

logger = logging.getLogger( __name__ )

# Audio capture parameters (matching ASR requirements)
SAMPLE_RATE = 16000
CHANNELS = 1
DTYPE = "int16"
# ~100ms chunks (1600 samples * 2 bytes = 3200 bytes)
CHUNK_SIZE = 1600


class MicCapture:
    """Threaded microphone capture that streams PCM audio to a callback."""

    def __init__ (
        self,
        on_audio: Callable[[bytes], None],
        sample_rate: int = SAMPLE_RATE,
        channels: int = CHANNELS,
        chunk_size: int = CHUNK_SIZE,
        device_index: Optional[int] = None,
    ) -> None:
        self._on_audio = on_audio
        self._sample_rate = sample_rate
        self._channels = channels
        self._chunk_size = chunk_size
        self._device_index = device_index

        self._stream: Optional[sd.InputStream] = None
        self._running = False

    def start(self) -> None:
        """Start capturing audio from the default microphone."""
        if self._running:
            return

        self._running = True
        self._stream = sd.InputStream(
            samplerate=self._sample_rate,
            channels=self._channels,
            dtype=DTYPE,
            blocksize=self._chunk_size,
            device=self._device_index,
            callback=self._audio_callback,
        )
        self._stream.start()
        logger.info(
            "Microphone capture started (rate=%d, channels=%d, chunk=%d)",
            self._sample_rate,
            self._channels,
            self._chunk_size,
        )

    def stop(self) -> None:
        """Stop capturing and release resources."""
        self._running = False
        if self._stream:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                logger.exception("Error closing audio stream")
            self._stream = None
        logger.info("Microphone capture stopped")

    @property
    def is_running(self) -> bool:
        return self._running

    def _audio_callback(self, indata: np.ndarray, frames: int, time_info, status) -> None:
        """sounddevice callback, called from audio thread."""
        if status:
            logger.warning("Mic capture status: %s", status)
        if self._running:
            # indata is int16 numpy array, convert to bytes for ASR
            self._on_audio(indata.tobytes())