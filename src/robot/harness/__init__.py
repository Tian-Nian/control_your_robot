"""策略和代理共用的一层：身体、门控、对齐观测、动作合同、episode。"""

from robot.harness.agent import AgentHarness
from robot.harness.contract import ActionContract, components_from_action
from robot.harness.episode import Episode
from robot.harness.executor import PositionExecutor
from robot.harness.harness import RobotHarness
from robot.harness.observe import read_observation

__all__ = [
    "ActionContract",
    "AgentHarness",
    "Episode",
    "RobotHarness",
    "PositionExecutor",
    "components_from_action",
    "read_observation",
]
