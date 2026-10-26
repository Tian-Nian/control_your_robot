import builtins
from pathlib import Path
import tomllib

import pytest


def test_mujoco_is_optional_and_setup_error_is_actionable(monkeypatch):
    root = Path(__file__).resolve().parents[4]
    project = tomllib.loads((root / "pyproject.toml").read_text())["project"]
    assert all("mujoco" not in item.lower() for item in project["dependencies"])
    assert any(item.startswith("mujoco") for item in project["optional-dependencies"]["harness-mujoco"])
    from robot.controller.PiperMuJoCo_controller import PiperMuJoCoController
    original = builtins.__import__
    def blocked(name, *args, **kwargs):
        if name == "mujoco":
            raise ImportError("intentionally absent")
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, "__import__", blocked)
    with pytest.raises(RuntimeError, match="harness-mujoco"):
        PiperMuJoCoController(cameras=False).set_up()


def test_optional_scene_uses_standard_runtime_config():
    from robot.runtime.config import load_config
    root = Path(__file__).resolve().parents[4]
    cfg = load_config(root / "config/runtime/piper_mujoco.yml")
    assert cfg.shm_name == "cyr_piper_mujoco"
    assert len(cfg.components) == 1
    assert "cameras" in cfg.components[0].publish
    assert cfg.components[0].max_bytes >= 2 * 320 * 240 * 7
    old = load_config(root / "config/runtime/piper_sim.yml")
    assert "MuJoCo" not in old.components[0].class_name
