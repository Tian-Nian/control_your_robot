import logging
import os
import threading
import time
import types
from pathlib import Path

import pytest

import numpy as np

from robot.data.hdf5_episode import Hdf5Episode, collect_until
from robot.data.xspark import load_actions, to_xspark
from robot.harness.runtime_env import RuntimeEnv, replay, run_policy
from robot.runtime.discover import write_record
from robot.runtime.lease import ControlLease
from robot.runtime.node import run_component
from robot.runtime.session import Session
from robot.runtime.transport import Bus, ControlBusyError, SafetyStopError, SlotSpec
from robot.teleop.joint_follow import follow


def _shm() -> str:
    return f"cyr_{time.time_ns()}"


class _Arm:
    def __init__(self, joint):
        self.joint = np.asarray(joint, dtype=float).copy()
        self.gripper = 0.0
        self.hold = False

    def set_up(self):
        return None

    def set_collect_info(self, info):
        self.info = list(info)

    def get_state(self):
        return {"joint": self.joint.copy(), "gripper": self.gripper}

    def get(self):
        return {"joint": self.joint.copy(), "gripper": self.gripper, "timestamp": time.time_ns()}

    def move(self, data, is_delta=False):
        if "joint" in data:
            self.joint = np.asarray(data["joint"], dtype=float).reshape(-1)
        if "gripper" in data:
            self.gripper = float(np.asarray(data["gripper"], dtype=float).reshape(-1)[0])

    def reset(self, start_state=None):
        self.joint = np.zeros_like(self.joint) if start_state is None else np.asarray(start_state, dtype=float)


def _config(path: Path, shm: str, names: list[str]) -> None:
    blocks = []
    for name in names:
        blocks.append(
            f"""
  - name: {name}
    class_path: robot.controller.TestArm_controller
    class_name: TestArmController
    args: {{name: {name}, DoFs: 6, INFO: ERROR}}
    publish: [joint, gripper]
    rate_hz: 100
    max_bytes: 65536
"""
        )
    path.write_text(
        f"""
name: loops
shm_name: {shm}
cmd_capacity: 8
components:
{''.join(blocks)}
""",
        encoding="utf-8",
    )


def _start(shm: str, specs: list[dict], arms: list[_Arm]):
    stop = threading.Event()
    threads = []
    for spec, arm in zip(specs, arms):
        thread = threading.Thread(
            target=run_component,
            args=(shm, spec, stop),
            kwargs={"factory": lambda arm=arm: arm},
            daemon=True,
        )
        thread.start()
        threads.append(thread)
    return stop, threads


def test_control_lock_rejects_other_writes_and_keeps_reads():
    shm = _shm()
    bus = Bus.create(shm, [SlotSpec("left_arm", 4096)], cmd_capacity=4)
    try:
        assert bus.write_state("left_arm", {"timestamp_ns": 1, "data": {"joint": [0.1]}})
        bus.write_cmd("left_arm", {"joint": [0.2]})
        bus.write_cmd("left_arm", {"joint": [0.3]})
        bus.write_stream("left_arm", {"joint": [0.8]})
        token = bus.acquire_control("teleop", lease_s=1.0)
        assert bus.pop_cmd("left_arm") is None
        assert bus.read_stream("left_arm") is None
        assert bus.read_state("left_arm")["data"]["joint"] == [0.1]
        assert bus.read_control()["label"] == "遥操"
        try:
            bus.write_cmd("left_arm", {"joint": [0.4]})
        except ControlBusyError as exc:
            assert "遥操" in str(exc)
        else:
            raise AssertionError("没有令牌的写入应该被拒绝")
        assert bus.write_cmd("left_arm", {"joint": [0.5]}, token=token) == 3
        try:
            bus.write_stream("left_arm", {"joint": [0.7]})
        except ControlBusyError as exc:
            assert "遥操" in str(exc)
        else:
            raise AssertionError("没有令牌的流式目标应该被拒绝")
        written = bus.write_stream("left_arm", {"joint": [0.7]}, token=token)
        assert written > 1
        assert bus.read_stream("left_arm")["data"]["joint"] == [0.7]
        assert bus.renew_control(token, 1.0)
        assert bus.release_control(token)
        assert bus.read_control()["held"] is False
        bus.write_cmd("left_arm", {"joint": [0.6]})

        expired = bus.acquire_control("replay", lease_s=0.05)
        time.sleep(0.12)
        assert bus.read_control()["held"] is False
        bus.write_cmd("left_arm", {"joint": [0.2]})
        assert bus.release_control(expired) is True

        holder = bus.acquire_control("model", lease_s=1.0)
        bus.latch_safety("停")
        with pytest.raises(SafetyStopError):
            bus.write_stream("left_arm", {"joint": [0.1]}, token=holder)
    finally:
        bus.unlink()


def test_dead_holder_releases_before_the_lease_ends():
    shm = _shm()
    bus = Bus.create(shm, [SlotSpec("left_arm", 4096)], cmd_capacity=2)
    try:
        from robot.runtime.transport import _CONTROL, _control_offset

        token = bus.acquire_control("model", lease_s=30.0)
        stored = _CONTROL.unpack_from(bus._shm.buf, _control_offset())
        _CONTROL.pack_into(bus._shm.buf, _control_offset(), stored[0], stored[1], 999999, stored[3], stored[4])
        assert bus.read_control()["held"] is False
        bus.write_cmd("left_arm", {"joint": [0.2]})
        assert bus.release_control(token) is True
    finally:
        bus.unlink()


def test_stopped_heartbeat_drops_the_lock():
    shm = _shm()
    bus = Bus.create(shm, [SlotSpec("left_arm", 4096)], cmd_capacity=2)
    config = Path(f"/tmp/{shm}.yml")
    _config(config, shm, ["left_arm"])
    write_record({"pid": os.getpid(), "config_path": str(config), "shm_name": shm, "components": ["left_arm"]})
    session = Session.attach(shm)
    lease = ControlLease(session, "teleop", lease_s=0.3)
    try:
        lease._stop.set()
        lease._thread.join(timeout=1.0)
        time.sleep(0.45)
        assert session._client.read_control()["held"] is False
        session._client.write_cmd("left_arm", {"joint": [0.1]})
    finally:
        lease.close()
        session.close()
        bus.unlink()


def test_lease_heartbeat_holds_without_new_targets():
    shm = _shm()
    bus = Bus.create(shm, [SlotSpec("left_arm", 4096)], cmd_capacity=2)
    config = Path(f"/tmp/{shm}.yml")
    _config(config, shm, ["left_arm"])
    write_record({"pid": os.getpid(), "config_path": str(config), "shm_name": shm, "components": ["left_arm"]})
    session = Session.attach(shm)
    lease = ControlLease(session, "model", lease_s=0.3)
    try:
        time.sleep(0.5)
        assert lease.alive
        assert session._client.read_control()["held"] is True
    finally:
        lease.close()
        session.close()
        bus.unlink()


def test_stream_slew_does_not_hold_the_component():
    shm = _shm()
    bus = Bus.create(shm, [SlotSpec("left_arm", 4096)], cmd_capacity=2)
    arm = _Arm([0.0, 0.0, 0.0])
    steps = []

    def move(data, is_delta=False):
        if "joint" in data:
            joint = np.asarray(data["joint"], dtype=float).reshape(-1)
            steps.append(float(joint[0]))
            arm.joint = joint

    arm.move = move
    spec = {
        "name": "left_arm",
        "publish": ["joint", "gripper"],
        "rate_hz": 50,
        "setup": {},
        "gate": {"max_step_rad": 0.1, "lower": [-2, -2, -2], "upper": [2, 2, 2]},
    }
    stop, threads = _start(shm, [spec], [arm])
    try:
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and bus.read_state("left_arm") is None:
            time.sleep(0.01)
        token = bus.acquire_control("teleop", lease_s=2.0)
        bus.write_stream("left_arm", {"joint": [0.4, 0.0, 0.0]}, token=token)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and (not steps or steps[-1] < 0.39):
            time.sleep(0.02)
        previous = 0.0
        assert steps
        for value in steps:
            assert value - previous <= 0.1 + 1e-9
            previous = value
        assert steps[-1] == pytest.approx(0.4)
        assert arm._cyr_motion_hold is False
        assert bus.read_safety()["latched"] is False
        bus.release_control(token)
        frozen = float(arm.joint[0])
        time.sleep(0.3)
        assert arm.joint[0] <= frozen + 0.11
    finally:
        stop.set()
        for thread in threads:
            thread.join(timeout=2)
        bus.unlink()


def test_joint_follow_holds_the_lock_and_copies_gripper():
    shm = _shm()
    names = ["master", "left_arm"]
    bus = Bus.create(shm, [SlotSpec(name, 4096) for name in names], cmd_capacity=4)
    config = Path(f"/tmp/{shm}.yml")
    _config(config, shm, names)
    write_record({"pid": os.getpid(), "config_path": str(config), "shm_name": shm, "components": names})
    master = _Arm([0.3, 0.1, 0.0, 0.0, 0.0, 0.0])
    master.gripper = 0.4
    follower = _Arm(np.zeros(6))
    specs = [
        {"name": "master", "publish": ["joint", "gripper"], "rate_hz": 100, "setup": {}},
        {"name": "left_arm", "publish": ["joint", "gripper"], "rate_hz": 100, "setup": {}},
    ]
    stop, threads = _start(shm, specs, [master, follower])
    session = Session.attach(shm)
    other = Session.attach(shm)
    teleop_stop = threading.Event()
    worker = threading.Thread(
        target=follow, args=(session, "master", "left_arm", 50.0, teleop_stop, 1.0), daemon=True
    )
    try:
        worker.start()
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and not np.allclose(follower.joint, master.joint):
            time.sleep(0.02)
        assert np.allclose(follower.joint, master.joint)
        assert follower.gripper == 0.4
        warned = []
        handler = logging.Handler()
        handler.setLevel(logging.WARNING)
        handler.emit = warned.append
        logger = logging.getLogger("cyr")
        logger.addHandler(handler)
        try:
            refused = other.call("left_arm", {"joint": [0.0] * 6}, timeout=0.2)
            streamed = other.write_target("left_arm", {"joint": [0.0] * 6})
        finally:
            logger.removeHandler(handler)
        assert refused["accepted"] is False
        assert streamed["accepted"] is False
        assert "遥操" in refused["error"]
        assert any("遥操" in record.getMessage() for record in warned)
        assert other.read_raw("left_arm") is not None
    finally:
        teleop_stop.set()
        worker.join(timeout=2)
        stop.set()
        for thread in threads:
            thread.join(timeout=2)
        other.close()
        session.close()
        bus.unlink()


def test_hdf5_matches_collectany_groups_and_xspark_shifts_action(tmp_path):
    episode = Hdf5Episode(tmp_path, "pick", 30.0)
    episode.record({"left_arm": {"joint": [0.0, 1.0], "gripper": 0.0, "timestamp": 1}}, 1.0)
    episode.record({"left_arm": {"joint": [0.2, 1.1], "gripper": 0.5, "timestamp": 2}}, 4.0)
    episode.record(
        {
            "left_arm": {"joint": [0.4, 1.2], "gripper": 1.0, "timestamp": 3},
            "cam_head": {"color": b"jpeg", "timestamp": 3},
        },
        2.0,
    )
    path = episode.finish()
    import h5py

    with h5py.File(path, "r") as handle:
        assert handle.attrs["skew_ms_max"] == 4.0
        assert np.allclose(handle["left_arm"]["joint"][:, 0], [0.0, 0.2, 0.4])
        assert np.asarray(handle["cam_head"]["color"][0]).tobytes().startswith(b"jpeg")
    assert (tmp_path / "pick" / "config.json").is_file()
    xspark = to_xspark(path, tmp_path / "pick_xspark.hdf5", instruction="拿起", frequency=30.0)
    with h5py.File(xspark, "r") as handle:
        assert handle.attrs["instruction"] == "拿起"
        assert np.allclose(handle["action"]["left_arm_joint_state"][:, 0], [0.2, 0.4])
        assert np.allclose(handle["state"]["left_arm_joint_state"][:, 0], [0.0, 0.2])
    actions = load_actions(path)
    assert actions[2]["left_arm_joint"][0] == 0.4


def test_policy_and_replay_share_take_action(tmp_path):
    shm = _shm()
    bus = Bus.create(shm, [SlotSpec("left_arm", 4096)], cmd_capacity=4)
    config = Path(f"/tmp/{shm}.yml")
    _config(config, shm, ["left_arm"])
    write_record({"pid": os.getpid(), "config_path": str(config), "shm_name": shm, "components": ["left_arm"]})
    arm = _Arm(np.zeros(6))
    spec = {"name": "left_arm", "publish": ["joint", "gripper"], "rate_hz": 100, "setup": {}}
    stop, threads = _start(shm, [spec], [arm])
    session = Session.attach(shm)
    module = types.ModuleType("XPolicyLab.policy.fake_policy.deploy")

    def eval_one_episode(env, model_client):
        model_client.call(func_name="reset")
        while not env.is_episode_end():
            obs = env.get_obs()
            assert "left_arm_joint_state" in obs["state"]
            model_client.call(func_name="update_obs", obs=obs)
            actions = model_client.call(func_name="get_action")
            for index, action in enumerate(actions):
                env.take_action(action)
                if env.is_episode_end() or index + 1 == len(actions):
                    break
                env.get_obs()

    module.eval_one_episode = eval_one_episode
    import sys

    sys.modules.setdefault("XPolicyLab", types.ModuleType("XPolicyLab"))
    sys.modules.setdefault("XPolicyLab.policy", types.ModuleType("XPolicyLab.policy"))
    sys.modules.setdefault("XPolicyLab.policy.fake_policy", types.ModuleType("XPolicyLab.policy.fake_policy"))
    sys.modules["XPolicyLab.policy.fake_policy.deploy"] = module

    class _Model:
        def call(self, func_name=None, obs=None):
            if func_name == "get_action":
                return [{"left_arm_joint_state": [0.25, 0.0, 0.0, 0.0, 0.0, 0.0]}]
            return None

    env = RuntimeEnv(session, control_hz=20.0, instruction="拿起", max_steps=1, lease_s=2.0)
    try:
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and session.read_raw("left_arm") is None:
            time.sleep(0.01)
        run_policy(env, _Model(), "fake_policy")
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline and not np.allclose(arm.joint[0], 0.25):
            time.sleep(0.02)
        assert arm.joint[0] == 0.25
        other = Session.attach(shm)
        try:
            refused = other.call("left_arm", {"joint": [0.0] * 6}, timeout=0.2)
            assert refused["accepted"] is False
            assert "部署" in refused["error"]
        finally:
            other.close()
    finally:
        env.close()

    episode = Hdf5Episode(tmp_path, "replay", 20.0)
    episode.record({"left_arm": {"joint": [0.0] * 6, "gripper": 0.2, "timestamp": 1}}, 0.0)
    episode.record({"left_arm": {"joint": [0.1, 0, 0, 0, 0, 0], "gripper": 0.7, "timestamp": 2}}, 0.0)
    path = episode.finish()
    replay_env = RuntimeEnv(session, control_hz=20.0, role="replay", max_steps=0, lease_s=2.0)
    try:
        assert replay(replay_env, load_actions(path)) == 2
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline and not np.allclose(arm.joint[0], 0.1):
            time.sleep(0.02)
        assert np.allclose(arm.joint[0], 0.1)
        assert arm.gripper == pytest.approx(0.7)
    finally:
        replay_env.close()
        session.close()
        stop.set()
        for thread in threads:
            thread.join(timeout=2)
        bus.unlink()


def test_observation_contract_stores_camera_jpeg():
    shm = _shm()
    names = ["left_arm", "cam_head"]
    bus = Bus.create(shm, [SlotSpec(name, 65536) for name in names], cmd_capacity=2)
    config = Path(f"/tmp/{shm}.yml")
    _config(config, shm, ["left_arm"])
    text = config.read_text(encoding="utf-8")
    text += """
  - name: cam_head
    class_path: robot.sensor.Test_sensor
    class_name: TestSensor
    args: {}
    publish: [color]
    rate_hz: 30
    max_bytes: 65536
"""
    config.write_text(text, encoding="utf-8")
    write_record({"pid": os.getpid(), "config_path": str(config), "shm_name": shm, "components": names})
    image = np.zeros((4, 4, 3), dtype=np.uint8)
    image[0, 0] = (12, 34, 56)

    class _Cam:
        def set_up(self):
            return None

        def set_collect_info(self, info):
            return None

        def get(self):
            return {"color": image, "timestamp": time.time_ns()}

    arm = _Arm(np.zeros(6))
    specs = [
        {"name": "left_arm", "publish": ["joint", "gripper"], "rate_hz": 50, "setup": {}},
        {"name": "cam_head", "publish": ["color"], "rate_hz": 30, "setup": {}},
    ]
    stop, threads = _start(shm, specs, [arm, _Cam()])
    session = Session.attach(shm)
    env = None
    try:
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            arm_state = session.read_raw("left_arm")
            cam_state = session.read_raw("cam_head")
            if arm_state is not None and cam_state is not None:
                break
            time.sleep(0.01)
        env = RuntimeEnv(session, control_hz=20.0, instruction="看一眼", max_steps=0, lease_s=1.0)
        obs = env.get_obs()
        assert obs["instruction"] == "看一眼"
        assert obs["data_format_version"] == "v1.0"
        assert obs["additional_info"]["frequency"] == 20.0
        assert len(obs["state"]["left_arm_joint_state"]) == 6
        color = obs["vision"]["cam_head"]["color"]
        assert color[:2] == b"\xff\xd8"
    finally:
        if env is not None:
            env.close()
        session.close()
        stop.set()
        for thread in threads:
            thread.join(timeout=2)
        bus.unlink()


def test_collect_reads_without_writing(tmp_path):
    shm = _shm()
    bus = Bus.create(shm, [SlotSpec("left_arm", 4096)], cmd_capacity=2)
    config = Path(f"/tmp/{shm}.yml")
    _config(config, shm, ["left_arm"])
    write_record({"pid": os.getpid(), "config_path": str(config), "shm_name": shm, "components": ["left_arm"]})
    arm = _Arm([0.3, 0, 0, 0, 0, 0])
    spec = {"name": "left_arm", "publish": ["joint", "gripper"], "rate_hz": 50, "setup": {}}
    stop, threads = _start(shm, [spec], [arm])
    session = Session.attach(shm)
    collect_stop = threading.Event()
    try:
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and session.read_raw("left_arm") is None:
            time.sleep(0.01)
        holder = {}

        def _run():
            holder["path"] = collect_until(session, "task", tmp_path, 30.0, collect_stop)

        worker = threading.Thread(target=_run)
        worker.start()
        time.sleep(0.15)
        collect_stop.set()
        worker.join(timeout=2)
        import h5py

        with h5py.File(holder["path"], "r") as handle:
            assert handle["left_arm"]["joint"].shape[0] >= 1
            assert handle["left_arm"]["joint"][0, 0] == 0.3
        assert np.allclose(arm.joint, [0.3, 0, 0, 0, 0, 0])
    finally:
        collect_stop.set()
        stop.set()
        for thread in threads:
            thread.join(timeout=2)
        session.close()
        bus.unlink()


def test_piper_sim_follow_replay_and_fake_policy(tmp_path):
    pytest.importorskip("pinocchio")
    pytest.importorskip("rerun")
    from robot.controller.PiperSim_controller import PiperSimController

    probe = PiperSimController("probe", spawn=False, INFO="ERROR")
    probe.set_up(control_mode="position")
    before = probe.flange_pose()
    probe.move_tool_delta([0.02, 0.0, 0.0])
    after = probe.flange_pose()
    tool_shift = before[:3, :3].T @ (after[:3, 3] - before[:3, 3])
    assert tool_shift == pytest.approx([0.02, 0.0, 0.0], abs=1e-3)
    assert np.allclose(after[:3, :3], before[:3, :3], atol=1e-3)

    shm = _shm()
    names = ["master", "left_arm"]
    bus = Bus.create(shm, [SlotSpec(name, 65536) for name in names], cmd_capacity=4)
    config = Path(f"/tmp/{shm}.yml")
    _config(config, shm, names)
    write_record({"pid": os.getpid(), "config_path": str(config), "shm_name": shm, "components": names})
    target = np.array([0.2, 1.0, -0.8, 0.0, 0.4, 0.1])
    master = _Arm(target)
    follower = PiperSimController("left_arm", spawn=False, INFO="ERROR")
    specs = [
        {"name": "master", "publish": ["joint", "gripper"], "rate_hz": 50, "setup": {}},
        {"name": "left_arm", "publish": ["joint", "eef", "gripper"], "rate_hz": 50, "setup": {}},
    ]
    stop, threads = _start(shm, specs, [master, follower])
    session = Session.attach(shm)
    teleop_stop = threading.Event()
    worker = threading.Thread(
        target=follow, args=(session, "master", "left_arm", 30.0, teleop_stop, 2.0), daemon=True
    )
    env = None
    try:
        worker.start()
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and (follower.q is None or not np.allclose(follower.q[:6], target, atol=1e-4)):
            time.sleep(0.02)
        assert follower.q is not None
        assert np.allclose(follower.q[:6], target, atol=1e-4)
        other = Session.attach(shm)
        try:
            refused = other.call("left_arm", {"joint": [0.0] * 6}, timeout=0.2)
            assert refused["accepted"] is False
            assert "遥操" in refused["error"]
        finally:
            other.close()
        teleop_stop.set()
        worker.join(timeout=2)

        played = [0.0, 0.9, -0.7, 0.1, 0.2, 0.0]
        episode = Hdf5Episode(tmp_path, "piper", 20.0)
        episode.record({"left_arm": {"joint": [0.0] * 6, "timestamp": 1}}, 0.0)
        episode.record({"left_arm": {"joint": played, "timestamp": 2}}, 0.0)
        path = episode.finish()

        class _Model:
            def call(self, func_name=None, obs=None):
                if func_name == "get_action":
                    return [{"left_arm_joint_state": played}]
                return None

        module = types.ModuleType("XPolicyLab.policy.piper_fake.deploy")

        def eval_one_episode(task_env, model_client):
            model_client.call(func_name="reset")
            while not task_env.is_episode_end():
                obs = task_env.get_obs()
                assert len(obs["state"]["left_ee_pose"]) == 7
                assert np.allclose(obs["state"]["left_arm_joint_state"], 0.0, atol=1e-4)
                for action in model_client.call(func_name="get_action"):
                    task_env.take_action(action)

        module.eval_one_episode = eval_one_episode
        import sys

        sys.modules.setdefault("XPolicyLab", types.ModuleType("XPolicyLab"))
        sys.modules.setdefault("XPolicyLab.policy", types.ModuleType("XPolicyLab.policy"))
        sys.modules.setdefault("XPolicyLab.policy.piper_fake", types.ModuleType("XPolicyLab.policy.piper_fake"))
        sys.modules["XPolicyLab.policy.piper_fake.deploy"] = module

        replay_env = RuntimeEnv(session, control_hz=20.0, role="replay", max_steps=0, lease_s=2.0)
        try:
            assert replay(replay_env, load_actions(path)) == 2
        finally:
            replay_env.close()
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline and not np.allclose(follower.q[:6], played, atol=1e-4):
            time.sleep(0.02)
        assert np.allclose(follower.q[:6], played, atol=1e-4)

        env = RuntimeEnv(session, control_hz=20.0, instruction="仿真", max_steps=1, lease_s=2.0)
        run_policy(env, _Model(), "piper_fake")
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline and not np.allclose(follower.q[:6], played, atol=1e-4):
            time.sleep(0.02)
        assert np.allclose(follower.q[:6], played, atol=1e-4)
        assert bus.read_safety()["latched"] is False
    finally:
        teleop_stop.set()
        worker.join(timeout=2)
        if env is not None:
            env.close()
        session.close()
        stop.set()
        for thread in threads:
            thread.join(timeout=2)
        bus.unlink()
