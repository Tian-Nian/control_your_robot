"""Batch-generate all trash-talk MP3 assets with Edge-TTS.

Run from this directory:
    python batch_generate.py

Or from the project root:
    python -m voice_system.batch_generate
"""

from __future__ import annotations

import asyncio
import argparse
import logging
from pathlib import Path

import edge_tts

try:
    from .trash_talk import TRASH_TALK_LINES, TrashTalkManager
except ImportError:  # Allows `python batch_generate.py` inside voice_system/.
    from trash_talk import TRASH_TALK_LINES, TrashTalkManager


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
)
logger = logging.getLogger(__name__)

VOICE = "zh-TW-HsiaoChenNeural"
RATE = "-5%"


async def generate_one(
    text: str,
    output_path: Path,
    overwrite: bool = False,
    voice: str = VOICE,
    rate: str = RATE,
) -> None:
    """Generate one MP3 file."""
    if output_path.exists() and not overwrite:
        logger.info("Skip existing: %s", output_path.name)
        return

    output_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        communicate = edge_tts.Communicate(text=text, voice=voice, rate=rate)
        await communicate.save(str(output_path))
        logger.info("Generated: %s <- %s", output_path.name, text)
    except Exception:
        logger.exception("Failed to generate %s <- %s", output_path.name, text)


async def generate_all(overwrite: bool = False, voice: str = VOICE, rate: str = RATE) -> None:
    """Generate all configured trash-talk assets."""
    manager = TrashTalkManager()
    tasks = []
    for event_type, lines in TRASH_TALK_LINES.items():
        for index, text in enumerate(lines):
            output_path = manager.asset_dir / f"{event_type}_{index}.mp3"
            tasks.append(generate_one(text, output_path, overwrite=overwrite, voice=voice, rate=rate))
    await asyncio.gather(*tasks, return_exceptions=True)


def main() -> None:
    """CLI entrypoint."""
    parser = argparse.ArgumentParser(description="Generate trash-talk MP3 assets.")
    parser.add_argument("--overwrite", action="store_true", help="Regenerate existing MP3 files.")
    parser.add_argument("--voice", default=VOICE, help="Edge-TTS voice name.")
    parser.add_argument("--rate", default=RATE, help='Speech rate, for example "-5%" or "+0%".')
    args = parser.parse_args()

    asyncio.run(generate_all(overwrite=args.overwrite, voice=args.voice, rate=args.rate))
    logger.info("All trash-talk assets are ready.")


if __name__ == "__main__":
    main()
