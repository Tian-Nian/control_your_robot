"""Voice interaction core for the desktop air-hockey AI opponent."""

from .main import VoiceSystem
from .asr_service import AsrConfig, AsrService
from .mic_capture import MicCapture

__all__ = ["VoiceSystem", "AsrConfig", "AsrService", "MicCapture"]
