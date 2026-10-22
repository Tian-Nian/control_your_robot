"""和策略服务之间的 JSON。numpy 数组按字节放进去。"""

from __future__ import annotations

import base64
import json
from typing import Any, Mapping

import numpy as np


def numpy_to_json(data: Any) -> str:
    return json.dumps(_clean(data), cls=_Encoder, ensure_ascii=False)


def json_to_numpy(payload: str) -> Any:
    def hook(item):
        if "__numpy_array__" in item:
            raw = base64.b64decode(item["data"])
            return np.frombuffer(raw, dtype=item["dtype"]).reshape(item["shape"])
        if "__bytes__" in item:
            return base64.b64decode(item["data"])
        return item

    return json.loads(payload, object_hook=hook)


def _clean(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, (bytes, bytearray)):
        return {"__bytes__": True, "data": base64.b64encode(bytes(value)).decode("ascii")}
    if isinstance(value, Mapping):
        return {str(key): _clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(item) for item in value]
    try:
        json.dumps(value)
    except TypeError:
        return str(value)
    return value


class _Encoder(json.JSONEncoder):
    def default(self, value):
        if isinstance(value, np.ndarray):
            return {
                "__numpy_array__": True,
                "data": base64.b64encode(value.tobytes()).decode("ascii"),
                "dtype": str(value.dtype),
                "shape": value.shape,
            }
        if isinstance(value, np.integer):
            return int(value)
        if isinstance(value, np.floating):
            return float(value)
        if isinstance(value, np.bool_):
            return bool(value)
        return super().default(value)
