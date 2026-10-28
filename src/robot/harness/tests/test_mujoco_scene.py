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
    move([.32, -.04, .17], 145)
    move([.32, -.04, .035], 180)
    scene.set_gripper(0); advance(scene, 2)
    lifted = move([.32, -.04, .17], 145)
    assert grasp_evidence(lifted, "bowl_0", start_z)["grasped_and_lifted"]
    # A short lift alone missed slow soft-contact creep. Verify sustained hold.
    assert scene.model.opt.noslip_iterations == 0
    assert scene.model.opt.impratio == 100
    for _ in range(15):
        held = advance(scene, 1)
        assert grasp_evidence(held, "bowl_0", start_z)["grasped_and_lifted"]
        assert abs(held["objects"]["bowl_0"]["pose"][2] - lifted["objects"]["bowl_0"]["pose"][2]) < .003
        assert held["gripper_command"] == 0 and held["gripper_preload_n"] == 2
        assert np.max(np.abs(held["gripper_actuator_force_n"])) <= 10.00001
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


@pytest.mark.parametrize("dx", [-.01, 0., .01])
def test_near_orange_rim_allows_six_vertical_agent_steps_and_retreat(scene, dx):
    """The default rim and a 1 cm lateral margin must allow the real bounded tool."""
    pytest.importorskip("pinocchio")
    from robot.harness.kinematics import PiperKinematics
    from robot.harness.policy import interpret
    kin = PiperKinematics()
    advance(scene, 1.)
    # This is a fixed fixture preposition, not an agent supplied object pose.
    bowl_xy = scene.model.body("bowl_0").pos[:2]
    start = [bowl_xy[0] + dx, bowl_xy[1] + .08, .095, 0., 0., 1., 0.]
    scene.set_joint(scene.solve_ik(start, "tool"))
    scene.set_gripper(1.)
    state = advance(scene, 2.)
    assert kin.error(state["joint"], start, "tool")[0] < .001
    for direction in (["z+"] * 6 + ["z-"] * 6):
        q = scene.get_state()["joint"].tolist()
        target, info = interpret({"action": "tool_move", "direction": direction,
                                  "reason": "验证近处碗沿的1cm垂直进退"}, q, kin)
        assert np.max(np.abs(np.asarray(target["joint"]) - q)) <= .08
        scene.set_joint(target["joint"])
        state = advance(scene, .6)
        pe, re = kin.error(state["joint"], info["target_pose"], "tool")
        assert pe < .001 and re < .01
    assert abs(kin.pose(state["joint"], "tool")[2] - .095) < .001


def test_logged_wait_pose_has_feasible_directions_and_small_joint_limit_relief(scene):
    """Replay the logged robot posture only; no automatic grasp/escape assertion."""
    pytest.importorskip("pinocchio")
    from robot.harness.kinematics import PiperKinematics
    from robot.harness.policy import interpret
    from robot.harness.recovery import recovery_options
    from robot.harness.vision import one_cm_options
    q = [-.3718572040898985, 1.6477429036329063, -1.5816930383846342,
         .1598376707855643, 1.2015320066944835, -.3994725660921034]
    scene.set_joint(q); scene.set_gripper(0.)
    state = advance(scene, 2.)
    kin = PiperKinematics()
    fk = {"tool": kin.pose(state["joint"], "tool"),
          "one_cm_translation_options": one_cm_options(state["joint"], kin),
          "tool_translation_options": one_cm_options(state["joint"], kin, frame="tool")}
    options = recovery_options(state["joint"], kin, fk)
    base_moves = {item["direction"] for item in options["feasible_base_moves"]}
    assert "up" not in base_moves and {"back", "down"} <= base_moves
    relief = next(item for item in options["joint_steps"] if item["action"] == {
        "action": "joint_step", "joint_index": 5, "sign": -1})
    assert relief["relieves_near_joint_limit"]
    assert relief["delta_xyz_base_m"][2] > .002
    target, _ = interpret({**relief["action"], "reason": "固定姿态复现：缓解第5轴上限"}, state["joint"].tolist(), kin)
    scene.set_joint(target["joint"])
    after = advance(scene, .6)
    assert after["joint"][4] < state["joint"][4] - .035
    assert kin.pose(after["joint"], "tool")[2] > fk["tool"][2] + .002


def test_oblique_head_profile_keeps_wrist_mount_and_published_calibration_consistent():
    if os.environ.get('CYR_TEST_MUJOCO_RENDER') != '1':
        pytest.skip('rendering requires CYR_TEST_MUJOCO_RENDER=1')
    device = PiperMuJoCoController(cameras=True, head_camera_profile='oblique')
    device.set_up()
    try:
        advance(device, 1.)
        # Recorded posture where the original head view hid the orange bowl.
        device.data.qpos[device._qadr[:6]] = [-.297495862152, 1.824204305, -1.332733256,
                                              .048476410, 1.006526595, -.320344908]
        device.mj.mj_forward(device.model, device.data)
        device._capture()
        camera = device.get_state()['cameras']['head']
        np.testing.assert_allclose(camera['world_from_camera'][:3], [.75, -.65, .65])
        assert camera['intrinsics'][0, 0] == pytest.approx(120 / np.tan(np.deg2rad(25)))
        assert device.model.cam_fovy[device.model.camera('wrist').id] == 37
        renderer = device.renderer
        renderer.enable_segmentation_rendering()
        renderer.update_scene(device.data, camera='head')
        segmentation = renderer.render()
        bowl = device.model.body('bowl_0').id
        geoms = np.where(device.model.geom_bodyid == bowl)[0]
        visible = np.isin(segmentation[:, :, 0], geoms) & (segmentation[:, :, 1] == int(device.mj.mjtObj.mjOBJ_GEOM))
        assert visible.sum() > 30
    finally:
        device.stop()


def test_closing_preload_persists_at_thin_gap_and_is_force_limited(scene):
    scene.set_gripper(0)
    ids=scene._act[6:]
    for _ in range(3):
        # Frozen joint probes isolate the actuator law from contact geometry.
        scene.data.qpos[scene._qadr[6:]]=.001
        scene.data.qvel[scene._vadr[6:]]=0
        scene.data.ctrl[ids]=0
        scene.mj.mj_forward(scene.model,scene.data)
        np.testing.assert_allclose(scene.data.actuator_force[ids],[-2.5,-2.5],atol=1e-8)
        scene.set_joint(scene.get_state()['joint'])
    scene.data.qpos[scene._qadr[6:]]=.04
    scene.mj.mj_forward(scene.model,scene.data)
    np.testing.assert_allclose(scene.data.actuator_force[ids],[-10,-10])
    scene.set_gripper(1)
    np.testing.assert_allclose(scene.model.actuator_biasprm[ids,0],0)
    assert scene.get_state()['gripper_preload_n']==0


def test_runtime_hold_clears_preload_even_if_measured_width_is_zero(scene):
    from robot.runtime.node import _stop_stream
    scene.set_gripper(0)
    scene.data.qpos[scene._qadr[6:]]=0
    _stop_stream(scene,'stop requested')
    assert not scene._gripper_preload_active
    np.testing.assert_allclose(scene.model.actuator_biasprm[scene._act[6:],0],0)


@pytest.mark.parametrize('force',[-1,11,float('nan')])
def test_invalid_gripper_preload_is_rejected(force):
    with pytest.raises(ValueError,match='gripper_preload_n'):
        PiperMuJoCoController(cameras=False,gripper_preload_n=force)
