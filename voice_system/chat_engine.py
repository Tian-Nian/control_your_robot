"""DeepSeek-backed chat engine and proactive topic picker."""

from __future__ import annotations

import logging
import os
import random
from collections import deque
from typing import Deque, Dict, Generator, List, Optional

import requests

logger = logging.getLogger(__name__)


SYSTEM_PROMPT = (
    "你是一个桌面冰球AI对手，user's 的损友 roommate。"
    "说话简短（<30字）、口语化、轻松但不过界。每次发言请先适当回应一下用户刚说的话，不要完全自说自话"
    "当前是聊天模式，像朋友一样接梗吐槽。"
)

PROACTIVE_TOPICS: Dict[str, List[str]] = {
    "work": [
        "你上个月绩效拿了个啥",
        "今天你又被老板骂了是吧",
        "按照你现在这个情况，打算啥时候拿n+1啊",
    ],
    "home": [
        "你今天除了拿外卖还出过卧室门吗",
        "你屋是不是该收拾收拾了",
    ],
    "health": [
        "你刚才弯腰我听到腰椎响了",
        "你眼睛红得像兔子，是哭了还是累了",
        "你脖子前倾快成企鹅了，抬抬头。",
    ],
    "nonsense": [
        "你觉得蝙蝠侠打得过孙悟空吗",
        "你打球的样子像我二舅",
        "你父母不催你结婚吗，咋还有心情在这打球",
    ],
}

# 用于分句的标点符号
_SENTENCE_DELIMITERS = {"。", "！", "？", "，", "；", "、", "\n", "!", "?", ",", ";"}


class ChatEngine:
    """Maintains short chat history and generates compact DeepSeek replies."""

    def __init__(
        self,
        api_key: Optional[str] = "sk-8da4c3a2810b42328fc4594aca7e5937",
        api_base: str = "https://api.deepseek.com",
        model: str = "deepseek-chat",
        timeout: float = 8.0,
    ) -> None:
        self.api_key = api_key or os.getenv("DEEPSEEK_API_KEY")
        self.api_base = api_base.rstrip("/")
        self.model = model
        self.timeout = timeout
        self._history: Deque[Dict[str, str]] = deque(maxlen=10)

    def proactive_opening(self) -> str:
        """Pick a proactive opening line when CHAT starts without user wake-up."""
        topic_lines = random.choice(list(PROACTIVE_TOPICS.values()))
        return random.choice(topic_lines)

    def reset_history(self) -> None:
        """Clear recent conversation history."""
        self._history.clear()

    def reply(self, user_text: str) -> str:
        """Generate a reply from DeepSeek (non-streaming); return a fallback on any failure."""
        user_text = user_text.strip()
        if not user_text:
            return "你说啥？我没听清"

        if not self.api_key:
            logger.warning("DEEPSEEK_API_KEY is not set; using fallback reply")
            return "你说啥？我没听清"

        messages = [{"role": "system", "content": SYSTEM_PROMPT}]
        messages.extend(list(self._history))
        messages.append({"role": "user", "content": user_text})

        try:
            response = requests.post(
                f"{self.api_base}/chat/completions",
                headers={
                    "Authorization": f"Bearer {self.api_key}",
                    "Content-Type": "application/json",
                },
                json={
                    "model": self.model,
                    "messages": messages,
                    "temperature": 0.8,
                    "max_tokens": 60,
                    "stream": False,
                },
                timeout=self.timeout,
            )
            response.raise_for_status()
            payload = response.json()
            reply_text = payload["choices"][0]["message"]["content"].strip()
            if not reply_text:
                raise ValueError("empty DeepSeek reply")
        except Exception:
            logger.exception("DeepSeek chat request failed")
            return "你说啥？我没听清"

        self._history.append({"role": "user", "content": user_text})
        self._history.append({"role": "assistant", "content": reply_text})
        return reply_text

    def reply_stream(self, user_text: str) -> Generator[str, None, None]:
        """Stream reply from DeepSeek, yielding sentence fragments as they form.

        Each yielded string is a complete clause/sentence (split on punctuation),
        ready to be sent to TTS immediately. If anything fails, yields a single
        fallback string.
        """
        user_text = user_text.strip()
        if not user_text:
            yield "你说啥？我没听清"
            return

        if not self.api_key:
            logger.warning("DEEPSEEK_API_KEY is not set; using fallback reply")
            yield "你说啥？我没听清"
            return

        messages = [{"role": "system", "content": SYSTEM_PROMPT}]
        messages.extend(list(self._history))
        messages.append({"role": "user", "content": user_text})

        full_reply = ""
        sentence_buf = ""

        try:
            response = requests.post(
                f"{self.api_base}/chat/completions",
                headers={
                    "Authorization": f"Bearer {self.api_key}",
                    "Content-Type": "application/json",
                },
                json={
                    "model": self.model,
                    "messages": messages,
                    "temperature": 0.8,
                    "max_tokens": 60,
                    "stream": True,
                },
                timeout=self.timeout,
                stream=True,
            )
            response.raise_for_status()

            for line in response.iter_lines(decode_unicode=True):
                if not line:
                    continue
                # SSE format: "data: {...}" or "data: [DONE]"
                if not line.startswith("data: "):
                    continue
                data_str = line[6:]
                if data_str.strip() == "[DONE]":
                    break

                try:
                    import json
                    chunk = json.loads(data_str)
                    delta = chunk.get("choices", [{}])[0].get("delta", {})
                    token = delta.get("content", "")
                except (ValueError, KeyError, IndexError):
                    continue

                if not token:
                    continue

                sentence_buf += token
                full_reply += token

                # Check if we hit a sentence delimiter → flush
                if sentence_buf and sentence_buf[-1] in _SENTENCE_DELIMITERS:
                    flushed = sentence_buf.strip()
                    if flushed:
                        yield flushed
                    sentence_buf = ""

            # Flush remaining buffer
            if sentence_buf.strip():
                yield sentence_buf.strip()

        except Exception:
            logger.exception("DeepSeek streaming request failed")
            if not full_reply:
                yield "你说啥？我没听清"
            return

        # Update history with the complete reply
        if full_reply.strip():
            self._history.append({"role": "user", "content": user_text})
            self._history.append({"role": "assistant", "content": full_reply.strip()})
