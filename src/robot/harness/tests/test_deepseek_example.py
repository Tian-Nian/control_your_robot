"""Offline boundary checks; normal tests never spend API credits."""
import importlib.util
import json
from pathlib import Path

import pytest

path = Path(__file__).resolve().parents[4] / "examples/deepseek_piper_task.py"
spec = importlib.util.spec_from_file_location("deepseek_example", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def call(name, args):
    return {"type": "function", "function": {"name": name, "arguments": json.dumps(args)}}


def test_accepts_expected_absolute_target():
    goal = [0, 1.35, -1, 0, .5, 0]
    assert module.check_call(call("move_joint_absolute", {"joint": goal}), {"joint": goal}) == {"joint": goal}


@pytest.mark.parametrize("tool,args", [
    ("shell", {"gripper": .5}),
    ("set_gripper", {"gripper": 1}),
    ("set_gripper", {"gripper": float("nan")}),
    ("set_gripper", {"gripper": .5, "extra": 1}),
    ("set_gripper", {"gripper": [.5]}),
])
def test_refuses_out_of_task_calls(tool, args):
    with pytest.raises(ValueError):
        module.check_call(call(tool, args), {"gripper": .5})
