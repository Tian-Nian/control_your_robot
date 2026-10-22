"""标准 JPEG 往返后红蓝不交换，已有字节不加标记。"""

import cv2
import numpy as np

from robot.data.rgb_jpeg import encode_rgb_jpeg


def test_rgb_jpeg_roundtrip_keeps_red_in_the_first_channel():
    image = np.zeros((32, 32, 3), dtype=np.uint8)
    image[:, :] = (255, 0, 0)
    encoded = encode_rgb_jpeg(image)
    assert encoded[:2] == b"\xff\xd8"
    assert b"JFIF" in encoded
    assert encoded.index(b"JFIF") < encoded.index(b"XPL-RGB1")
    decoded = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
    rgb = cv2.cvtColor(decoded, cv2.COLOR_BGR2RGB)
    assert rgb[16, 16, 0] > 250
    assert rgb[16, 16, 1] < 5
    assert rgb[16, 16, 2] < 5


def test_existing_bytes_stay_unmarked():
    raw = b"\xff\xd8legacy"
    assert encode_rgb_jpeg(raw) == raw
