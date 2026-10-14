import ast
from pathlib import Path

from robot.runtime.config import load_config

_ROOT = Path(__file__).resolve().parents[1]
_ALLOWED = {
    "robot.controller.TestArm_controller",
    "robot.controller.TestMobile_controller",
    "robot.sensor.TestVision_sensor",
}
_FORBIDDEN = (
    "TestArm",
    "TestMobile",
    "TestVison",
    "TestVision",
    "Piper",
    "Realman",
    "Realsense",
    "Franka",
    "pyrealsense",
    "Bunker",
    "Tracer",
    "URUrx",
    "drAloha",
    "DexHand",
)


def test_shipped_config_uses_only_test_devices():
    config = load_config(Path(__file__).resolve().parents[4] / "config" / "runtime" / "test_robot.yml")
    assert config.shm_name == "cyr_test_robot"
    assert config.cmd_capacity == 8
    names = [item.name for item in config.components]
    assert names == [
        "left_arm",
        "right_arm",
        "test_mobile",
        "cam_head",
        "cam_left_wrist",
        "cam_right_wrist",
    ]
    assert {item.class_path for item in config.components} == _ALLOWED


def test_runtime_sources_do_not_import_devices():
    for path in _ROOT.rglob("*.py"):
        if "tests" in path.parts:
            continue
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                modules = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                modules = [node.module or ""]
            else:
                continue
            for module in modules:
                assert not any(token in module for token in _FORBIDDEN), f"{path} imports {module}"
