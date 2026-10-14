"""设备安全停止。运行时只调用组件上的 safe_stop，不导入具体驱动。"""

from __future__ import annotations


def engage_safe_stop(component) -> str:
    """让组件进入安全状态。没有 safe_stop 时只锁住后续命令。"""
    if component is None:
        return "no device"
    method = getattr(component, "safe_stop", None)
    if not callable(method):
        setattr(component, "_cyr_safety_stopped", True)
        return "hold"
    try:
        result = method()
    except Exception:
        setattr(component, "_cyr_safety_stopped", True)
        raise
    if isinstance(result, str) and result:
        return result
    return "safe_stop"
