"""奥比中光彩色相机。只开彩色流，不改机器人。"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np

from robot.sensor.sensor import Sensor


class OrbbecSensor(Sensor):
    def __init__(self, name: str = "orbbec"):
        super().__init__()
        self.name = name
        self.type = "vision"
        self._pipeline = None
        self._decode = None
        self._color: np.ndarray | None = None

    def set_up(self, serial: str, width: int = 640, height: int = 480, fps: int = 30):
        sdk = _load_sdk()
        self._decode = sdk["decode"]
        context = sdk["Context"]()
        device = _device_by_serial(context, str(serial))
        pipeline = sdk["Pipeline"](device)
        config = sdk["Config"]()
        profiles = pipeline.get_stream_profile_list(sdk["OBSensorType"].COLOR_SENSOR)
        config.enable_stream(_pick_color_profile(profiles, int(width), int(height), int(fps)))
        pipeline.start(config)
        self._pipeline = pipeline
        self._color = self._wait_color()

    def get_information(self):
        color = self._wait_color()
        self._color = color
        return {"color": color}

    def safe_stop(self) -> str:
        pipeline = self._pipeline
        self._pipeline = None
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass
        return "hold"

    def _wait_color(self) -> np.ndarray:
        pipeline = self._pipeline
        if pipeline is None or self._decode is None:
            raise RuntimeError(f"{self.name} 尚未打开")
        frames = pipeline.wait_for_frames(1000)
        if frames is None or frames.get_color_frame() is None:
            if self._color is not None:
                return self._color
            raise RuntimeError(f"{self.name} 没有彩色帧")
        image = self._decode(frames.get_color_frame())
        if image is None:
            raise RuntimeError(f"{self.name} 彩色帧解码失败")
        rgb = np.asanyarray(image)
        if rgb.ndim == 3 and rgb.shape[2] == 3:
            rgb = rgb[:, :, ::-1]
        return np.ascontiguousarray(rgb)


def _load_sdk() -> dict:
    import pyorbbecsdk
    from pyorbbecsdk import Config, Context, OBFormat, OBSensorType, Pipeline

    utils_path = Path(pyorbbecsdk.__file__).resolve().parent / "pyorbbecsdk" / "examples" / "utils.py"
    spec = importlib.util.spec_from_file_location("pyorbbecsdk_examples_utils", utils_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"找不到奥比解码工具 {utils_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return {
        "Config": Config,
        "Context": Context,
        "Pipeline": Pipeline,
        "OBFormat": OBFormat,
        "OBSensorType": OBSensorType,
        "decode": module.frame_to_bgr_image,
    }


def _device_by_serial(context, serial: str):
    device_list = context.query_devices()
    count = int(device_list.get_count())
    found = []
    for index in range(count):
        current = device_list.get_device_serial_number_by_index(index)
        found.append(str(current))
        if str(current) == serial:
            return device_list.get_device_by_index(index)
    raise RuntimeError(f"找不到奥比相机 {serial}，当前有 {found or '无'}")


def _pick_color_profile(profile_list, width: int, height: int, fps: int):
    from pyorbbecsdk import OBFormat

    exact = None
    mjpg = None
    fallback = None
    for index in range(int(profile_list.get_count())):
        profile = profile_list.get_stream_profile_by_index(index)
        video = profile.as_video_stream_profile()
        fallback = fallback or video
        if video.get_format() == OBFormat.MJPG and mjpg is None:
            mjpg = video
        if (
            video.get_width() == width
            and video.get_height() == height
            and video.get_fps() == fps
            and video.get_format() == OBFormat.MJPG
        ):
            exact = video
            break
    chosen = exact or mjpg or fallback
    if chosen is None:
        raise RuntimeError("奥比相机没有彩色流")
    return chosen
