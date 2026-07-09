ROBOT_REGISTRY = {
    "replay_robot": ("my_robot.replay_robot", "ReplayRobot"),
}


def _load_robot_class(robot_type):
    if robot_type not in ROBOT_REGISTRY:
        raise ValueError(
            f"Unknown robot type {robot_type!r}. Supported: {list(ROBOT_REGISTRY)}"
        )
    module_path, class_name = ROBOT_REGISTRY[robot_type]
    import importlib

    module = importlib.import_module(module_path)
    return getattr(module, class_name)


def get_robot(base_cfg):
    robot_type = base_cfg["robot"].get("type")
    if not robot_type:
        raise KeyError("Missing ['robot']['type'] in config")
    robot_cls = _load_robot_class(robot_type)
    return robot_cls(base_config=base_cfg)


def __getattr__(name):
    if name in {"ReplayRobot", "build_replay_robot_config", "create_replay_robot"}:
        from my_robot import replay_robot as module

        return getattr(module, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = ["ReplayRobot", "build_replay_robot_config", "create_replay_robot", "get_robot"]
