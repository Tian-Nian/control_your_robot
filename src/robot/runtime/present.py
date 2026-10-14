"""把观测格式化成可读文本。图像不展开像素。"""

from __future__ import annotations

import numpy as np


def format_value(value: object) -> str:
    if isinstance(value, np.ndarray):
        if value.ndim >= 2 or value.size > 8:
            return f"ndarray shape={tuple(int(dim) for dim in value.shape)} dtype={value.dtype}"
        return np.array2string(value, precision=3, suppress_small=True, separator=" ")
    if isinstance(value, (bytes, bytearray)):
        return f"bytes len={len(value)}"
    if isinstance(value, dict):
        return "{...}"
    text = str(value)
    if len(text) > 72:
        return text[:69] + "..."
    return text


def format_data(data: object) -> str:
    if not isinstance(data, dict):
        return format_value(data)
    lines = [f"{key}: {format_value(value)}" for key, value in data.items()]
    return "\n".join(lines)


def detail_lines(data: object) -> list[str]:
    if not isinstance(data, dict):
        return [format_value(data)]
    return [f"{key}  {format_value(value)}" for key, value in data.items()]
