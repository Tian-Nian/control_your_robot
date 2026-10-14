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


def summarize_value(value: object) -> object:
    """给 agent 的值。图像和长数组只留形状，不带像素。"""
    if isinstance(value, np.ndarray):
        if value.ndim >= 2 or value.size > 8:
            return {
                "kind": "ndarray",
                "shape": [int(dim) for dim in value.shape],
                "dtype": str(value.dtype),
                "nbytes": int(value.nbytes),
            }
        return [_json_number(item) for item in value.reshape(-1).tolist()]
    if isinstance(value, (bytes, bytearray, memoryview)):
        return {"kind": "bytes", "nbytes": len(value)}
    if isinstance(value, dict):
        return {str(key): summarize_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [summarize_value(item) for item in value]
    if isinstance(value, (np.floating, float)):
        return float(value)
    if isinstance(value, (np.integer, int)) and not isinstance(value, bool):
        return int(value)
    return value


def _json_number(value: object) -> object:
    if isinstance(value, bool) or value is None:
        return value
    if isinstance(value, (int, np.integer)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        return float(value)
    return value


def detail_lines(data: object) -> list[str]:
    if not isinstance(data, dict):
        return [format_value(data)]
    return [f"{key}  {format_value(value)}" for key, value in data.items()]
