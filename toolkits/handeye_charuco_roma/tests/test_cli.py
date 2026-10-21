from pathlib import Path

import numpy as np

from handeye_charuco.session import Sample, Session
from handeye_charuco_roma.cli import main

from handeye_charuco_roma.config import load_config, resolve_camera_device, resolve_camera_role, resolve_setup
from handeye_charuco_test_helpers import (
    K,
    SPEC,
    T_EE_CAM,
    cam_board_views,
    eih_ee_from_view,
    eth_ee_from_view,
    render_view,
)


def test_check_config_offline():
    assert main(["check-config", "--offline"]) == 0


def test_eih_binds_left_wrist():
    cfg = load_config(overlay={"setup": "eye_in_hand", "arm_side": "left", "camera_role": "auto", "camera_device": None})
    assert resolve_setup(cfg) == "eye_in_hand"
    assert resolve_camera_role(cfg) == "left_wrist"
    assert resolve_camera_device(cfg) == "cam_left"
    assert main(["--setup", "eye_in_hand", "--side", "left", "check-config", "--offline"]) == 0


def test_eth_still_binds_head():
    cfg = load_config()
    assert resolve_setup(cfg) == "eye_to_hand"
    assert resolve_camera_role(cfg) == "head"
    assert resolve_camera_device(cfg) == "cam_head"


def test_check_board():
    assert main(["check-board"]) == 0


def test_print_board(tmp_path: Path):
    out = tmp_path / "board.png"
    assert main(["print-board", "--out", str(out)]) == 0
    assert out.is_file()


def test_check_config_fake_observe():
    assert main(["--fake", "check-config"]) == 0


def test_preview_fake_no_display():
    assert main(["--fake", "preview", "--no-display"]) == 0


def test_auto_mode_rejected():
    try:
        main(["--fake", "capture", "--mode", "auto", "--out", "/tmp/nope"])
    except SystemExit as exc:
        assert "teach" in str(exc) or "claim" in str(exc).lower() or "Pipeline" in str(exc)
    else:
        raise AssertionError("auto mode should exit")


def test_calibrate_verify_from_session(tmp_path: Path):
    session = Session(
        root=tmp_path,
        setup="eye_to_hand",
        board=SPEC,
        arm_side="right",
        camera_role="head",
    )
    session.intrinsics = K
    for i, view in enumerate(cam_board_views(12)):
        rgb = render_view(view)
        rel = session.save_image(rgb, i)
        session.samples.append(
            Sample(index=i, joint=np.zeros(7), T_base_ee=eth_ee_from_view(view), image_file=rel)
        )
    session.save()
    result_path = tmp_path / "result.json"
    code = main(["calibrate", str(tmp_path), "--out", str(result_path)])
    assert result_path.is_file()
    assert code in (0, 1)
    v = main(
        [
            "verify",
            str(tmp_path),
            "--result",
            str(result_path),
            "--max-rms-trans",
            "0.02",
            "--max-rms-rot",
            "4",
        ]
    )
    assert v == 0


def test_calibrate_verify_eye_in_hand(tmp_path: Path):
    session = Session(
        root=tmp_path,
        setup="eye_in_hand",
        board=SPEC,
        arm_side="left",
        camera_role="left_wrist",
    )
    session.intrinsics = K
    for i, view in enumerate(cam_board_views(12)):
        rgb = render_view(view)
        rel = session.save_image(rgb, i)
        session.samples.append(
            Sample(index=i, joint=np.zeros(7), T_base_ee=eih_ee_from_view(view), image_file=rel)
        )
    session.save()
    result_path = tmp_path / "result.json"
    code = main(["calibrate", str(tmp_path), "--out", str(result_path), "--force-intrinsics"])
    assert result_path.is_file()
    assert code in (0, 1)
    payload = __import__("json").loads(result_path.read_text())
    assert payload["setup"] == "eye_in_hand"
    assert payload["T_ee_cam"] is not None
    got = np.asarray(payload["T_ee_cam"], dtype=float)
    np.testing.assert_allclose(got[:3, 3], T_EE_CAM[:3, 3], atol=0.02)
    v = main(
        [
            "verify",
            str(tmp_path),
            "--result",
            str(result_path),
            "--max-rms-trans",
            "0.02",
            "--max-rms-rot",
            "4",
        ]
    )
    assert v == 0
