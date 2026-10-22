"""标准 RGB JPEG。内存里的图是 RGB，文件带 XPL-RGB1，和 XPolicyLab 的 encode_image_bit 一致。"""

from __future__ import annotations

import numpy as np

_MARKER = b"XPL-RGB1"
_PAD = 0xFF
_SOI = 0xD8
_APP0 = 0xE0
_COM = 0xFE
_SEGMENT_LENGTH = len(_MARKER) + 2


def encode_rgb_jpeg(image, quality: int | None = None) -> bytes:
    """把一张 uint8 RGB 收成带标记的 JPEG。已经是字节的原样返回，不补标记。"""
    if isinstance(image, (bytes, bytearray)):
        return bytes(image)
    array = np.asarray(image)
    if array.dtype == np.uint8 and array.ndim == 1:
        return array.tobytes()
    if array.dtype != np.uint8 or array.ndim != 3 or array.shape[-1] != 3:
        raise ValueError(f"彩色图应为 uint8 的 HxWx3，得到 {array.dtype} {array.shape}")
    import cv2

    bgr = cv2.cvtColor(np.ascontiguousarray(array), cv2.COLOR_RGB2BGR)
    params = [] if quality is None else [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    ok, encoded = cv2.imencode(".jpg", bgr, params)
    if not ok:
        raise RuntimeError("图像不能收成 JPEG")
    return _insert_marker(encoded.tobytes())


def _insert_marker(jpeg: bytes) -> bytes:
    """标记放在 JFIF 的 APP0 后面。APP0 必须紧跟 SOI。"""
    segment = bytes([_PAD, _COM]) + _SEGMENT_LENGTH.to_bytes(2, "big") + _MARKER
    offset = 2
    if len(jpeg) >= 6 and jpeg[2:4] == bytes([_PAD, _APP0]):
        app0_end = 4 + int.from_bytes(jpeg[4:6], "big")
        if app0_end <= len(jpeg):
            offset = app0_end
    return jpeg[:offset] + segment + jpeg[offset:]
