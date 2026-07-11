"""Trash-talk text catalog and pre-generated MP3 selection logic."""

from __future__ import annotations

import logging
import random
from pathlib import Path
from typing import Dict, List, Optional

logger = logging.getLogger(__name__)


TRASH_TALK_LINES: Dict[str, List[str]] = {
    "human_goal": [
        "你蒙的吧",
        "这球送你了",
        "我要睁眼跟你打了",
        "这是你最后一次得分了",
    ],
    "ai_goal": [
        "太慢了",
        "换人换人",
        "放轻松，头晕是正常的",
        "人类进化的时候你躲起来了吗",
    ],
    "human_miss": [
        "球呢",
        "建议挂个眼科",
        "人机合一你只有机",
        "我奶奶都比你眼神好",
    ],
    "defense_success": [
        "就这？",
        "跟你打就当休息了",
        "跟你打一天我都不带出汗的",
    ],
    "idle": [
        "你想啥呢bro",
        "我CPU都等降温了",
        "要不你去忙别的，再买个机械臂陪我打吧",
    ],
    "start": [
        "输了别哭",
        "准备好被上课了吗",
    ],
    "end": [
        "就这？",
        "回去再练两年",
    ],
}


class TrashTalkManager:
    """Maps game events to local pre-generated MP3 files."""

    def __init__(self, asset_dir: str | Path | None = None) -> None:
        self.asset_dir = Path(asset_dir) if asset_dir else Path(__file__).parent / "assets" / "trash_talk"
        self.asset_dir.mkdir(parents=True, exist_ok=True)

    @property
    def event_types(self) -> List[str]:
        """Return supported trash-talk event types."""
        return list(TRASH_TALK_LINES.keys())

    def get_random_audio(self, event_type: str) -> Optional[Path]:
        """Return a random MP3 path for an event, or None if unavailable."""
        if event_type not in TRASH_TALK_LINES:
            logger.warning("Unknown trash-talk event: %s", event_type)
            return None

        candidates = [
            self.asset_dir / f"{event_type}_{i}.mp3"
            for i in range(len(TRASH_TALK_LINES[event_type]))
        ]
        existing = [path for path in candidates if path.exists()]
        if not existing:
            logger.warning(
                "No pre-generated MP3 found for event=%s in %s. Run batch_generate.py first.",
                event_type,
                self.asset_dir,
            )
            return None
        return random.choice(existing)

    def get_text(self, event_type: str, index: int) -> str:
        """Return source text for a generated asset."""
        return TRASH_TALK_LINES[event_type][index]

    def expected_files(self) -> Dict[str, List[Path]]:
        """Return all MP3 paths expected from the batch generator."""
        return {
            event: [self.asset_dir / f"{event}_{i}.mp3" for i, _ in enumerate(lines)]
            for event, lines in TRASH_TALK_LINES.items()
        }
