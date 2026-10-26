"""Optional physics tests; rendering additionally requires explicit opt-in."""
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

pytest.importorskip("mujoco", reason="explicit harness-mujoco extra not installed")
from robot.controller.PiperMuJoCo_controller import PiperMuJoCoController
from robot.harness.stacking import StackVerifier, grasp_evidence


@pytest.fixture
def scene():
    device = PiperMuJoCoController(cameras=False)
    device.set_up()
    try:
        yield device
    finally:
        device.stop()


def advance(scene, seconds):
    for _ in range(round(seconds / .02)):
        scene.servo_step(.02)
    assert not scene.fault
    return scene.get_state()


def test_free_bowls_fall_and_settle_with_hollow_collision(scene):
    before = scene.get_state()
    after = advance(scene, 1.)
    for name, item in after["objects"].items():
        assert item["pose"][2] < before["objects"][name]["pose"][2] - .05
        assert abs(item["pose"][2]) < .001
        assert np.linalg.norm(item["velocity"]) < .01
    assert after["contacts"]
    assert not StackVerifier().update(after)["stacked"]
    # There must be multiple collision pieces, not one solid convex bowl.
    assert sum(scene.model.geom(i).name.startswith("bowl_0_collision_") for i in range(scene.model.ngeom)) == 24


def test_joint_commands_drive_actuators_without_teleporting(scene):
    before = scene.get_state()["joint"]
    goal = before.copy(); goal[1] += .05
    scene.set_joint(goal)
    np.testing.assert_allclose(scene.get_state()["joint"], before)
    after = advance(scene, 1)
    np.testing.assert_allclose(after["joint"], goal, atol=.005)
    with pytest.raises(ValueError): scene.set_joint([0] * 5)
    with pytest.raises(ValueError): scene.set_control_mode("mit")


def test_robotwin_camera_mount_and_optical_axes_at_multiple_poses(scene):
    # Independently reproduced from RoboTwin piper.urdf fixed-joint origins,
    # not from MJCF's camera settings or visual mesh origins.
    r1 = Rotation.from_euler("xyz", [0, 0, 1.5708]).as_matrix()
    r2 = Rotation.from_euler("xyz", [0, -1.1116926535897932, -np.pi / 2]).as_matrix()
    offset = np.array([0, 0, -.077]) + r1 @ [.032, .055, .115]
    gl_from_axes = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
    cid = scene.model.camera("wrist").id
    link = scene.model.body("link6").id
    for q in ([0, 1.3, -1, 0, .5, 0], [.2, 1., -.7, .1, .3, -.2]):
        scene.data.qpos[scene._qadr[:6]] = q
        scene.mj.mj_forward(scene.model, scene.data)
        rotation = scene.data.xmat[link].reshape(3, 3)
        np.testing.assert_allclose(scene.data.cam_xpos[cid], scene.data.xpos[link] + rotation @ offset, atol=1e-9)
        np.testing.assert_allclose(scene.data.cam_xmat[cid].reshape(3, 3), rotation @ r1 @ r2 @ gl_from_axes, atol=1e-9)
    assert scene.model.cam_fovy[cid] == 37
    head = scene.model.camera("head").id
    np.testing.assert_allclose(scene.data.cam_xpos[head], [0, .032, .6], atol=1e-9)
    assert scene.model.cam_fovy[head] == 37


def test_pinocchio_reference_pose_and_local_ik(scene):
    # Reference produced by the original PiperSimController at READY_JOINT.
    reference = [.2704102192955, .00141521125664, .3129706029072,
                 .4159810225025, .000001836697575, .9093732945892, -.000001516837446]
    np.testing.assert_allclose(scene.get_state()["eef"], reference, atol=1e-9)
    before = scene.data.qpos.copy()
    np.testing.assert_allclose(scene.solve_ik(reference), before[scene._qadr[:6]], atol=1e-6)
    np.testing.assert_array_equal(scene.data.qpos, before)


def test_pinocchio_tool_and_flange_match_physics(scene):
    pytest.importorskip("pinocchio")
    from robot.harness.kinematics import PiperKinematics
    kin = PiperKinematics()
    for q in ([0, 1.3, -1, 0, .5, 0], [.2, 1., -.7, .1, .3, -.2]):
        scene.data.qpos[scene._qadr[:6]] = q
        scene.mj.mj_forward(scene.model, scene.data)
        for point, site in (("tool", scene._tool), ("flange", scene._flange)):
            pose = np.asarray(kin.pose(q, point))
            np.testing.assert_allclose(pose[:3], scene.data.site_xpos[site], atol=1e-8)
            rotation = Rotation.from_quat(pose[[4, 5, 6, 3]]).as_matrix()
            np.testing.assert_allclose(rotation, scene.data.site_xmat[site].reshape(3, 3), atol=1e-8)


def test_readonly_viewer_replays_current_state_without_physics_steps(scene, monkeypatch):
    import importlib.util
    path = Path(__file__).resolve().parents[4] / "examples/piper_mujoco_viewer.py"
    spec = importlib.util.spec_from_file_location("readonly_viewer", path)
    viewer = importlib.util.module_from_spec(spec); spec.loader.exec_module(viewer)
    scene.set_joint([.1, 1.3, -1, 0, .5, 0]); state = advance(scene, 1.)
    original = scene.data.qpos.copy()
    local = PiperMuJoCoController(cameras=False); local.set_up()
    try:
        monkeypatch.setattr(local.mj, "mj_step", lambda *args: pytest.fail("viewer must never step physics"))
        viewer.sync_view(local, state)
        np.testing.assert_allclose(local.get_state()["joint"], state["joint"])
        np.testing.assert_allclose(local.get_state()["eef"], state["eef"], atol=1e-8)
        for name, item in state["objects"].items():
            np.testing.assert_allclose(local.get_state()["objects"][name]["pose"], item["pose"], atol=1e-8)
        np.testing.assert_array_equal(scene.data.qpos, original)
        assert local.data.time == state["sim_time"]
    finally:
        local.stop()


def test_viewer_uses_matching_bundled_binary_not_conda_path(scene, monkeypatch):
    import rerun as rr
    import rerun_cli
    spawned = []
    monkeypatch.setattr(rr, "spawn", lambda **kwargs: spawned.append(kwargs))
    scene.spawn = True
    scene._start_rerun(port=9879)
    executable = Path(spawned[0]["executable_path"])
    assert executable.is_file() and Path(rerun_cli.__file__).parent in executable.parents
    assert spawned[0]["port"] == 9879


def test_fixed_head_image_directions_match_agent_coordinate_guide(scene):
    cid=scene.model.camera("head").id
    rotation=scene.data.cam_xmat[cid].reshape(3,3) @ np.diag([1,-1,-1])
    eye=scene.data.cam_xpos[cid]
    def project(point):
        p=rotation.T @ (np.asarray(point)-eye)
        assert p[2]>0
        return p[:2]/p[2]  # normalized image coordinates, +v downward
    point=np.asarray(scene.get_state()["eef"][:3])
    initial=project(point)
    assert (project(point+[0,.01,0])-initial)[0]<0  # +Y appears left
    assert (project(point+[.01,0,0])-initial)[1]<0  # +X appears up
    assert (project(point+[0,0,.01])-initial)[1]<0  # +Z can also appear up


def test_actual_contact_pick_lift_and_release(scene):
    advance(scene, 1)
    start_z = scene.get_state()["objects"]["bowl_0"]["pose"][2]
    def move(xyz, angle):
        a = np.deg2rad(angle) / 2
        scene.set_joint(scene.solve_ik([*xyz, np.cos(a), 0, np.sin(a), 0], "tool"))
        return advance(scene, 3)
    scene.set_gripper(1); advance(scene, .6)
    move([.42, -.04, .17], 145)
    move([.42, -.04, .035], 180)
    scene.set_gripper(0); advance(scene, 2)
    lifted = move([.42, -.04, .17], 145)
    assert grasp_evidence(lifted, "bowl_0", start_z)["grasped_and_lifted"]
    scene.set_gripper(1)
    released = advance(scene, 3)
    assert not grasp_evidence(released, "bowl_0", start_z)["bilateral_contact"]
    assert abs(released["objects"]["bowl_0"]["pose"][2] - start_z) < .002


def test_three_bowl_support_and_stability_fixture(scene):
    # This initializes a stacking fixture; it does NOT claim a robot stacked them.
    verifier = StackVerifier()
    for i in range(3):
        address = scene.model.joint(f"bowl_{i}_free").qposadr[0]
        scene.data.qpos[address:address+3] = [.5, 0, .08 + i * .08]
    scene.mj.mj_forward(scene.model, scene.data)
    for _ in range(200):
        scene.servo_step(.02)
        result = verifier.update(scene.get_state())
    assert result["stacked"]
    state = scene.get_state()
    # Removing support evidence must invalidate even geometrically aligned bowls.
    state["contacts"] = []
    assert not verifier.update(state)["stacked"]


def test_cyr_start_attach_control_and_stop(tmp_path):
    root = Path(__file__).resolve().parents[4]
    completed = subprocess.run([sys.executable, str(root / "examples/piper_mujoco_harness.py"),
                                "--no-cameras", "--out", str(tmp_path / "events.jsonl")],
                               cwd=root, capture_output=True, text=True, timeout=60)
    assert completed.returncode == 0, completed.stdout + completed.stderr
    events = [json.loads(line) for line in (tmp_path / "events.jsonl").read_text().splitlines()]
    assert [e["result"]["status"] for e in events if e["event"] == "finished"] == ["succeeded"] * 2


@pytest.mark.skipif(os.environ.get("CYR_TEST_MUJOCO_RENDER") != "1", reason="explicit rendering opt-in required")
def test_rgbd_timestamp_intrinsics_and_rerun_recording(tmp_path):
    device = PiperMuJoCoController(cameras=True, record_path=str(tmp_path / "scene.rrd"))
    try:
        device.set_up()
        first = device.get_state()["cameras"]["wrist"]["frame_id"]
        device.servo_step(.1)
        for camera in device.get_state()["cameras"].values():
            assert camera["rgb"].shape == (240, 320, 3)
            assert camera["depth_m"].shape == (240, 320)
            assert camera["rgb"].std() > 5
            assert np.isfinite(camera["depth_m"]).all()
            assert np.min(camera["depth_m"]) > 0
            assert camera["frame_id"] > first
            assert camera["sim_time"] == pytest.approx(.1)
            assert camera["intrinsics"][0, 0] == pytest.approx(120 / np.tan(np.deg2rad(37) / 2))
    finally:
        device.stop()
    assert (tmp_path / "scene.rrd").stat().st_size > 10000
