"""Session 上面的薄封装：对齐观测、按合同动作、记下 episode。"""

from __future__ import annotations

from robot.harness.contract import ActionContract
from robot.harness.episode import Episode
from robot.harness.observe import read_observation


class RobotHarness:
    def __init__(self, session, *, mode: str = "joint", control_hz: float = 15.0, max_skew_ms: float = 80.0):
        self.session = session
        self.contract = ActionContract(mode=mode, control_hz=control_hz)
        self.episode = Episode()
        self.max_skew_ms = float(max_skew_ms)
        self.body = _load_body(session)

    def get_obs(self) -> dict:
        return read_observation(self.session, max_skew_ms=self.max_skew_ms, body=self.body)

    def take_action(self, action, *, timeout: float = 2.0) -> dict:
        return self.contract.take_action(self.session, action, timeout=timeout)

    def reset(self, name: str, start_state=None, timeout: float = 15.0) -> dict:
        return self.session.reset(name, start_state=start_state, timeout=timeout)

    def start_episode(self, path) -> None:
        self.episode.start(path)

    def step(self, action, *, timeout: float = 2.0) -> dict:
        observation = self.get_obs()
        if self.episode.active:
            self.episode.record(observation, action)
        result = self.take_action(action, timeout=timeout)
        return {"observation": observation, "result": result}

    def finish_episode(self, success: bool, reason: str = ""):
        return self.episode.finish(success, reason)


def _load_body(session):
    for item in getattr(session, "_config").components:
        path = getattr(item, "body", "") or ""
        if path:
            from robot.harness.body import BodyModel

            return BodyModel.load(path)
    return None
