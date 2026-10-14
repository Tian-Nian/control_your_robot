import time

import numpy as np

from robot.runtime.hmi.model import HmiModel
from robot.runtime.hmi.render import frame


def test_frame_lists_components_and_moves_selection():
    now_ns = time.time_ns()
    model = HmiModel(["left_arm", "cam_head"])
    model.set_link(True, 7, "cyr_test")
    model.set_names(["left_arm", "cam_head"])
    model.update_component(
        "left_arm",
        {"hz": 100, "state_seq": 4, "drops": 0, "error": "", "alive": True},
        {"seq": 4, "timestamp_ns": now_ns, "data": {"joint": np.zeros(6)}},
        now_ns,
    )
    model.update_component(
        "cam_head",
        {"hz": 30, "state_seq": 2, "drops": 0, "error": "", "alive": True},
        {"seq": 2, "timestamp_ns": now_ns, "data": {"color": np.zeros((4, 4, 3), dtype=np.uint8)}},
        now_ns,
    )
    text = "\n".join(frame(model, 100, 24))
    assert "left_arm" in text
    assert "cam_head" in text
    assert "joint" in text
    assert "hz" in text
    assert "age" in text
    assert "error" in text or "err" in text
    assert model.apply_key("j") is True
    assert model.selected_name() == "cam_head"
    detail = "\n".join(frame(model, 100, 24))
    assert "shape=" in detail
    assert "关闭服务" in text
    assert model.apply_key("stop") is True
    assert "y 确认" in "\n".join(frame(model, 100, 24))
    assert model.apply_key("no") is True
    assert model.take_action() == ""
    assert model.apply_key("stop") is True
    assert model.apply_key("yes") is True
    assert model.take_action() == "stop"
