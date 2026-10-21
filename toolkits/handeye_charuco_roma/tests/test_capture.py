from pathlib import Path

import numpy as np

from handeye_charuco.session import Session
from handeye_charuco_roma.capture import (
    annotate_overlay,
    capture_sample,
    drop_samples,
    record_joint,
    replay_waypoints,
)
from handeye_charuco_roma.fk import FakeFk
from handeye_charuco_roma.joints import JointLog
from handeye_charuco_roma.mover import FakeMover
from handeye_charuco_roma.observe import FakeObserve, grab_pair, rgb_from_sample

from handeye_charuco_test_helpers import SPEC, cam_board_views, render_view


def test_fake_observe_pair():
    rgb0 = np.zeros((32, 40, 3), dtype=np.uint8)
    rgb0[3, 4] = (9, 10, 11)
    obs = FakeObserve(q=np.array([0.1, 0, 0, 0, 0, 0, 0]), rgb_fn=lambda: rgb0)
    rgb, q, cam, arm = grab_pair(obs, arm="right_arm", cam="cam_head")
    np.testing.assert_array_equal(rgb[3, 4], [9, 10, 11])
    np.testing.assert_allclose(q[0], 0.1)
    assert cam["format"] == 1
    assert arm["kind"] == "joint"
    back = rgb_from_sample(cam)
    np.testing.assert_array_equal(back[3, 4], [9, 10, 11])


def test_capture_writes_session(tmp_path: Path):
    view = cam_board_views(1)[0]
    obs = FakeObserve(rgb_fn=lambda: render_view(view), q=np.array([0.2, 0, 0, 0, 0, 0, 0]))
    session = Session(
        root=tmp_path,
        setup="eye_to_hand",
        board=SPEC,
        arm_side="right",
        camera_role="head",
        meta={"source": "test"},
    )
    sample, det, rgb = capture_sample(
        session,
        obs,
        FakeFk(),
        SPEC,
        arm="right_arm",
        cam="cam_head",
        min_corners=8,
        require_detection=False,
    )
    assert rgb.ndim == 3
    assert det.n_corners >= 0
    assert sample.T_base_ee.shape == (4, 4)
    np.testing.assert_allclose(sample.joint[0], 0.2)
    session.save()
    loaded = Session.load(tmp_path)
    assert len(loaded.samples) == 1
    np.testing.assert_allclose(loaded.samples[0].T_base_ee[:3, 3], FakeFk().fk_T([0.2, 0, 0, 0, 0, 0, 0])[:3, 3])


def test_record_joint_then_replay(tmp_path: Path):
    q0 = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    q1 = np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    obs = FakeObserve(rgb_fn=lambda: render_view(cam_board_views(1)[0]), q=q0)
    log = JointLog(tmp_path / "joints.json", arm="right_arm")
    i0, rec0 = record_joint(log, obs, "right_arm")
    obs.q = q1
    i1, rec1 = record_joint(log, obs, "right_arm")
    assert (i0, i1) == (0, 1)
    np.testing.assert_allclose(rec0[0], 0.1)
    np.testing.assert_allclose(rec1[0], 0.3)
    loaded = JointLog.load_or_new(tmp_path / "joints.json", arm="right_arm")
    assert len(loaded.q) == 2

    session = Session(root=tmp_path, setup="eye_in_hand", board=SPEC, arm_side="left", camera_role="left_wrist")
    slept = []
    pairs = replay_waypoints(
        session,
        loaded,
        obs,
        FakeFk(),
        SPEC,
        FakeMover(obs),
        arm="right_arm",
        cam="cam_head",
        settle_s=1.0,
        gap_s=1.0,
        sleep=slept.append,
    )
    assert len(pairs) == 2
    assert slept == [1.0, 1.0, 1.0, 1.0]
    assert len(session.samples) == 2
    np.testing.assert_allclose(session.samples[0].joint[0], 0.1)
    np.testing.assert_allclose(session.samples[1].joint[0], 0.3)


def test_annotate_and_drop_sample(tmp_path: Path):
    session = Session(root=tmp_path, setup="eye_to_hand", board=SPEC, arm_side="left", camera_role="head")
    rgb = render_view(cam_board_views(1)[0])
    rel = session.save_image(rgb, 37)
    from handeye_charuco.session import Sample

    s = Sample(index=37, joint=np.zeros(7), T_base_ee=np.eye(4), image_file=rel, n_corners=4)
    s.T_base_ee[:3, 3] = [0.231, 0.725, -0.012]
    session.samples.append(s)
    vis = annotate_overlay(np.zeros((80, 400, 3), dtype=np.uint8), s)
    assert vis.shape == (80, 400, 3)
    session.save()
    (session.overlays_dir).mkdir(parents=True, exist_ok=True)
    (session.overlays_dir / "0037.png").write_bytes(b"x")
    assert drop_samples(session, [37]) == [37]
    assert session.samples == []
    assert not (tmp_path / "images" / "0037.png").exists()
    assert not (session.overlays_dir / "0037.png").exists()
