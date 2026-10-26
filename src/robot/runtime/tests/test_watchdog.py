import logging
import time
import uuid

import numpy as np
import pytest

from robot.runtime.node import _advance, _init_runtime_state
from robot.runtime.transport import Bus, SlotSpec


class Arm:
    def __init__(self):
        self.q = np.zeros(2)
        self.moves = []

    def get_state(self):
        return dict(joint=self.q.copy(), gripper=0, control_mode="position")

    def move(self, payload, is_delta=False):
        self.q = np.array(payload["joint"])
        self.moves.append(self.q.copy())


@pytest.fixture
def scene():
    bus = Bus.create("wd_" + uuid.uuid4().hex[:12], [SlotSpec("arm", 4096)], cmd_capacity=4)
    arm = Arm()
    spec = dict(name="arm", gate={"max_step_rad": .01})
    _init_runtime_state(arm, spec)
    token = bus.acquire_control("model", 2)
    try:
        yield bus, arm, spec, token
    finally:
        bus.unlink()


def tick(scene):
    bus, arm, spec, _ = scene
    _advance(bus, spec, arm, logging.getLogger("test"))


def test_deadline_expires_while_lease_alive_and_needs_release(scene):
    bus, arm, spec, token = scene
    bus.write_stream("arm", {"joint": [1, 0]}, token, valid_for_s=.02)
    tick(scene)
    time.sleep(.025)
    tick(scene)
    assert bus.read_control()["held"]
    assert arm._cyr_stream_state == "stopped"
    assert arm._cyr_stream_reason == "stream target expired"
    stopped = arm.q.copy()
    bus.write_stream("arm", {"joint": [.5, 0]}, token, valid_for_s=1)
    tick(scene)
    assert np.array_equal(arm.q, stopped)
    bus.write_cmd("arm", {"op": "release"}, token=token)
    tick(scene)
    tick(scene)  # The target written while latched must not replay.
    assert np.array_equal(arm.q, stopped)
    bus.write_stream("arm", {"joint": [.5, 0]}, token, valid_for_s=1)
    tick(scene)
    assert arm.q[0] > stopped[0]


def test_delayed_first_read_does_not_extend_deadline(scene):
    bus, arm, _, token = scene
    bus.write_stream("arm", {"joint": [1, 0]}, token, valid_for_s=.01)
    time.sleep(.015)
    tick(scene)
    assert np.allclose(arm.q, 0)
    assert arm._cyr_stream_state == "stopped"


def test_config_cap_applies_when_client_omits_ttl(scene):
    bus, arm, spec, token = scene
    spec["stream_timeout_s"] = .01
    bus.write_stream("arm", {"joint": [1, 0]}, token)
    time.sleep(.015)
    tick(scene)
    assert arm._cyr_stream_state == "stopped"


def test_explicit_stop_preempts_active_motion(scene):
    bus, arm, _, token = scene
    arm._cyr_motion = object()  # Would fail if the plan advanced before stop.
    bus.write_stream("arm", {"op": "stop"}, token)
    tick(scene)
    assert arm._cyr_motion is None
    assert arm._cyr_stream_state == "stopped"


def test_lost_lease_stops_deadlined_target(scene):
    bus, arm, _, token = scene
    bus.write_stream("arm", {"joint": [1, 0]}, token, valid_for_s=1)
    tick(scene)
    bus.release_control(token)
    tick(scene)
    assert arm._cyr_stream_state == "stopped"
    assert arm._cyr_stream_reason == "control lease lost"


@pytest.mark.parametrize("ttl", [0, -1, float("nan"), float("inf")])
def test_invalid_ttl_rejected(scene, ttl):
    bus, _, _, token = scene
    with pytest.raises(ValueError):
        bus.write_stream("arm", {"joint": [1, 0]}, token, valid_for_s=ttl)
