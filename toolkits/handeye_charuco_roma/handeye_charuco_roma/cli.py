"""roma Observe capture + the existing handeye_charuco solver."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from handeye_charuco import se3
from handeye_charuco.calibrate import calibrate_session, quality_gate, save_result
from handeye_charuco.charuco import (
    BoardSpec,
    detect_charuco,
    draw_detection,
    generate_board_image,
)
from handeye_charuco.session import Session
from handeye_charuco.verify import (
    board_spec_questionnaire,
    draw_verify_overlay,
    evaluate_frame,
    load_result,
    verify_quality_gate,
    verify_session,
)

from .capture import capture_sample, drop_samples, label_session_overlays, record_joint, replay_waypoints
from .joints import JointLog
from .mover import FakeMover, RomaSessionMover
from .config import (
    DEFAULT_CONFIG_PATH,
    PACKAGE_ROOT,
    load_config,
    resolve_arm_device,
    resolve_camera_device,
    resolve_camera_role,
    resolve_kine_cfg,
    resolve_roma_root,
    resolve_sdk_root,
    resolve_setup,
)
from .fk import MarvinOfflineFk
from .observe import FakeObserve, ObserveLike, connect_observe, grab_pair


def _load(args) -> dict:
    overlay = {}
    if args.setup:
        overlay["setup"] = args.setup
    if getattr(args, "side", None):
        overlay["arm_side"] = args.side
    if getattr(args, "camera_role", None):
        overlay["camera_role"] = args.camera_role
    if getattr(args, "arm_device", None):
        overlay["arm_device"] = args.arm_device
    if getattr(args, "camera_device", None):
        overlay["camera_device"] = args.camera_device
    if getattr(args, "roma_root", None):
        overlay["roma_root"] = args.roma_root
    if getattr(args, "sdk_root", None):
        overlay["sdk_root"] = args.sdk_root
    cfg = load_config(args.config, overlay=overlay or None)
    cfg["setup"] = resolve_setup(cfg)
    # default.yml 是眼在手外/头相机；--setup eye_in_hand 时改绑左腕，除非用户显式指定相机。
    if cfg["setup"] == "eye_in_hand":
        if not getattr(args, "camera_role", None) and cfg.get("camera_role") in (None, "", "auto", "head"):
            cfg["camera_role"] = "auto"
        if not getattr(args, "camera_device", None) and cfg.get("camera_device") in (None, "", "auto", "cam_head"):
            cfg["camera_device"] = None
    cfg["camera_role"] = resolve_camera_role(cfg)
    cfg["arm_device"] = resolve_arm_device(cfg)
    cfg["camera_device"] = resolve_camera_device(cfg)
    return cfg


def _board(cfg: dict) -> BoardSpec:
    return BoardSpec.from_dict(cfg.get("board") or {})


def _session_dir(args, cfg: dict) -> Path:
    if getattr(args, "out", None):
        return Path(args.out).expanduser().resolve()
    setup = cfg["setup"]
    side = cfg.get("arm_side", "left")
    role = cfg["camera_role"]
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return (PACKAGE_ROOT / "runs" / f"{setup}_{side}_{role}_{stamp}").resolve()


def _fk(cfg: dict, args) -> MarvinOfflineFk:
    if getattr(args, "fake", False):
        from .fk import FakeFk

        return FakeFk(side=str(cfg.get("arm_side") or "left"))
    sdk = resolve_sdk_root(cfg)
    return MarvinOfflineFk(
        side=str(cfg.get("arm_side") or "left"),
        sdk_root=sdk,
        kine_cfg=resolve_kine_cfg(cfg, sdk),
    )


def _open_obs(cfg: dict, args) -> ObserveLike:
    if getattr(args, "fake", False):
        return FakeObserve(arm=cfg["arm_device"], cam=cfg["camera_device"])
    timeout = float((cfg.get("capture") or {}).get("ready_timeout_s", 15.0))
    return connect_observe(timeout_s=timeout)


def summarize(cfg: dict, obs: Optional[ObserveLike] = None) -> str:
    lines = [
        f"roma={resolve_roma_root(cfg)}",
        f"sdk={resolve_sdk_root(cfg)}",
        f"kine={resolve_kine_cfg(cfg)}",
        f"setup={cfg['setup']} side={cfg.get('arm_side')} role={cfg['camera_role']}",
        f"arm_device={cfg['arm_device']}  camera_device={cfg['camera_device']}",
        "HAL: Observe only (no claim). Do not open Pipeline MarvinArmController on 192.168.1.190.",
        "FK: Marvin_Kine offline → unified flange TCP (X fwd / Y left / Z up).",
        (
            "EIH: board fixed in world, wrist cam on flange. Solve T_ee_cam."
            if cfg["setup"] == "eye_in_hand"
            else "ETH: board on flange, head cam fixed. Solve T_base_cam."
        ),
    ]
    if obs is not None:
        lines.append(f"hardware_phase={obs.hardware_phase} names={list(obs.names)}")
        for name in (cfg["arm_device"], cfg["camera_device"]):
            try:
                st = obs.status(name)
                lines.append(f"  {name}: phase={st.get('phase')} mode={st.get('mode')} owner={st.get('owner')!r}")
            except Exception as exc:
                lines.append(f"  {name}: status failed: {exc}")
    return "\n".join(lines)


def _teach_banner(cfg: dict) -> str:
    if cfg["setup"] == "eye_in_hand":
        return (
            "拖拽只记关节: 板固定，腕相机能看到板即可。回车记当前 q，q/done 结束并回放。"
            "回放时每点停 1s 再采图和 eef。"
        )
    return (
        "拖拽只记关节: 板固连法兰。回车记当前 q，q/done 结束并回放。"
        "回放时每点停 1s 再采图和 eef。"
    )


def cmd_check_config(args) -> int:
    cfg = _load(args)
    spec = _board(cfg)
    print(summarize(cfg))
    print(
        f"board {spec.squares_x}x{spec.squares_y} square={spec.square_m*1000:.2f}mm "
        f"marker={spec.marker_m*1000:.2f}mm dict={spec.dictionary} print_scale={spec.print_scale}"
    )
    print()
    print(board_spec_questionnaire(spec))
    if args.offline:
        return 0
    try:
        obs = _open_obs(cfg, args)
    except Exception as exc:
        print(f"Observe skipped: {exc}")
        return 0
    print()
    print(summarize(cfg, obs))
    try:
        rgb, q, cam_s, arm_s = grab_pair(obs, arm=cfg["arm_device"], cam=cfg["camera_device"])
        T = _fk(cfg, args).fk_T(q)
        xyz = se3.to_xyz_rpy(T, degrees=True)
        print(
            f"sample {cfg['camera_device']} {rgb.shape[1]}x{rgb.shape[0]}  "
            f"{cfg['arm_device']} q={np.array2string(q[:7], precision=3)}"
        )
        print(f"FK EE xyz={xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f}  seq_cam={cam_s.get('seq')} seq_arm={arm_s.get('seq')}")
    except Exception as exc:
        print(f"sample/FK failed: {exc}")
        return 1
    return 0


def cmd_check_board(args) -> int:
    cfg = _load(args)
    print(board_spec_questionnaire(_board(cfg)))
    return 0


def cmd_print_board(args) -> int:
    cfg = _load(args)
    spec = _board(cfg)
    dpi = int(args.dpi)
    px_per_m = dpi / 0.0254
    margin_px = int(round(12.0 / 1000.0 * px_per_m))
    board_img = generate_board_image(spec, px_per_m=px_per_m, margin_px=margin_px)
    caption = (
        f"Charuco {spec.squares_x}x{spec.squares_y}  "
        f"square={spec.square_length_m*1000:.1f}mm  marker={spec.marker_length_m*1000:.1f}mm  "
        f"{spec.dictionary}"
    )
    pad = 80
    canvas = 255 * np.ones((board_img.shape[0] + pad, board_img.shape[1]), dtype=np.uint8)
    canvas[pad:] = board_img
    cv2.putText(canvas, caption[:110], (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, 0, 1, cv2.LINE_AA)
    out = Path(args.out).expanduser().resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(out), canvas):
        raise RuntimeError(f"failed to write {out}")
    print(f"wrote {out}")
    return 0


def cmd_preview(args) -> int:
    cfg = _load(args)
    spec = _board(cfg)
    obs = _open_obs(cfg, args)
    fk = _fk(cfg, args)
    print(summarize(cfg, obs))
    print("preview: q/ESC quit. Charuco must light up green before capture.")
    use_gui = bool(args.display if args.display is not None else (cfg.get("capture") or {}).get("display", False))
    try:
        while True:
            rgb, q, _, _ = grab_pair(obs, arm=cfg["arm_device"], cam=cfg["camera_device"])
            det = detect_charuco(rgb, spec, rgb=True)
            T_board = None
            bgr = draw_detection(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), det, T_board, None)
            try:
                ee = se3.to_xyz_rpy(fk.fk_T(q), degrees=True)
                cv2.putText(
                    bgr,
                    f"EE xyz={ee[0]:.3f},{ee[1]:.3f},{ee[2]:.3f}",
                    (12, 56),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1,
                )
            except Exception as exc:
                cv2.putText(bgr, f"FK failed: {exc}", (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            if not use_gui:
                print(f"corners={det.n_corners}  {rgb.shape[1]}x{rgb.shape[0]}")
                return 0
            cv2.imshow("handeye_roma_preview", bgr)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    finally:
        if use_gui:
            cv2.destroyAllWindows()
    return 0


def _new_session(args, cfg: dict) -> Session:
    root = _session_dir(args, cfg)
    existing = root / "session.json"
    if existing.is_file():
        session = Session.load(root)
        nxt = 0 if not session.samples else session.samples[-1].index + 1
        accepted = sum(1 for s in session.samples if s.accepted)
        print(f"resume {session.root}  samples={len(session.samples)} accepted={accepted}  next=#{nxt}")
        return session
    session = Session(
        root=root,
        setup=cfg["setup"],
        board=_board(cfg),
        arm_side=cfg.get("arm_side", "left"),
        camera_role=cfg["camera_role"],
        meta={
            "source": "roma_observe",
            "arm_device": cfg["arm_device"],
            "camera_device": cfg["camera_device"],
            "roma_root": str(resolve_roma_root(cfg)),
            "kine_cfg": str(resolve_kine_cfg(cfg)),
        },
    )
    session.ensure_dirs()
    return session


def cmd_capture(args) -> int:
    cfg = _load(args)
    if args.mode == "auto":
        raise SystemExit("自动走位请用 teach 记关节再 replay，不要 --mode auto。")
    cap = dict(cfg.get("capture") or {})
    obs = _open_obs(cfg, args)
    fk = _fk(cfg, args)
    session = _new_session(args, cfg)
    spec = session.board
    min_corners = int(cap.get("min_corners", 8))
    max_reproj = float(cap.get("max_reproj_px", 3.0))
    grab = int(cap.get("grab_frames", 4))
    settle_s = float(getattr(args, "settle_s", None) if getattr(args, "settle_s", None) is not None else cap.get("settle_s", 1.0))
    gap_s = float(getattr(args, "gap_s", None) if getattr(args, "gap_s", None) is not None else cap.get("gap_s", 1.0))
    max_deg_s = float(cap.get("replay_max_deg_s", 20.0))
    log = JointLog.load_or_new(session.root / "joints.json", arm=cfg["arm_device"])
    print(summarize(cfg, obs))
    print(f"session -> {session.root}")
    print(_teach_banner(cfg))
    use_gui = bool(args.display if args.display is not None else cap.get("display", False))
    if not args.replay_only:
        if use_gui:
            _teach_gui(cfg, obs, log)
        else:
            _teach_stdio(cfg, obs, log)
        print(f"joints -> {log.path}  n={len(log.q)}")
    if args.no_replay:
        return 0 if log.q else 2
    if not log.q:
        print("没有记录到关节，先拖拽回车记点。")
        return 2
    mover = FakeMover(obs) if getattr(args, "fake", False) else RomaSessionMover(obs, cfg["arm_device"])
    try:
        print(f"REPLAY {len(log.q)} waypoints  settle={settle_s}s gap={gap_s}s  already={len(session.samples)}")
        pairs = replay_waypoints(
            session,
            log,
            obs,
            fk,
            spec,
            mover,
            arm=cfg["arm_device"],
            cam=cfg["camera_device"],
            min_corners=min_corners,
            max_reproj_px=max_reproj,
            grab_frames=grab,
            settle_s=0.0 if getattr(args, "fake", False) else settle_s,
            gap_s=0.0 if getattr(args, "fake", False) else gap_s,
            max_deg_s=max_deg_s,
        )
        for sample, det in pairs:
            print(f"  replay #{sample.index} corners={det.n_corners} accepted={sample.accepted}")
    finally:
        mover.close()
    session.save()
    accepted = sum(1 for s in session.samples if s.accepted)
    print(f"saved {session.root / 'session.json'}  samples={len(session.samples)} accepted={accepted}")
    if accepted < int(cap.get("min_samples", 12)):
        print(f"WARNING: accepted {accepted} < min_samples {cap.get('min_samples', 12)}")
        return 2
    return 0


def _capture_one(cfg, obs, fk, session, spec, min_corners, max_reproj, grab):
    return capture_sample(
        session,
        obs,
        fk,
        spec,
        arm=cfg["arm_device"],
        cam=cfg["camera_device"],
        min_corners=min_corners,
        max_reproj_px=max_reproj,
        grab_frames=grab,
        require_detection=False,
    )


def _teach_gui(cfg, obs, log: JointLog):
    spec = _board(cfg)
    while True:
        rgb, q, _, _ = grab_pair(obs, arm=cfg["arm_device"], cam=cfg["camera_device"])
        det = detect_charuco(rgb, spec, rgb=True)
        bgr = draw_detection(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), det, None, None)
        cv2.putText(
            bgr,
            f"joints={len(log.q)}  SPACE=record q  q=replay",
            (12, 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
        cv2.imshow("handeye_roma_teach", bgr)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        if key in (32, 13):
            idx, rec = record_joint(log, obs, cfg["arm_device"])
            print(f"  joint #{idx} q={np.array2string(rec, precision=3)}")
    cv2.destroyAllWindows()


def _teach_stdio(cfg, obs, log: JointLog):
    print(f"no GUI: Enter 只记关节（已有 {len(log.q)}），q/done 开始回放")
    while True:
        line = sys.stdin.readline()
        if not line:
            break
        if line.strip().lower() in {"q", "quit", "done"}:
            break
        idx, rec = record_joint(log, obs, cfg["arm_device"])
        print(f"  joint #{idx} q={np.array2string(rec, precision=3)}")


def cmd_label_overlays(args) -> int:
    session = Session.load(args.session)
    n = label_session_overlays(session)
    print(f"labeled {n} overlays in {session.overlays_dir}")
    return 0


def cmd_drop_frames(args) -> int:
    session = Session.load(args.session)
    dropped = drop_samples(session, args.indices)
    missing = sorted(set(int(i) for i in args.indices) - set(dropped))
    print(f"dropped {dropped}  left={len(session.samples)}")
    if missing:
        print(f"not in session: {missing}")
        return 2
    return 0


def cmd_calibrate(args) -> int:
    session = Session.load(args.session)
    result, _intr = calibrate_session(
        session,
        min_corners=int(args.min_corners),
        max_reproj_px=float(args.max_reproj),
        refine=not args.no_refine,
        reject=not args.no_reject,
        force_intrinsics=args.force_intrinsics,
        refine_intrinsics=not args.no_refine_intrinsics,
    )
    session.save()
    out = Path(args.out) if args.out else session.root / "result.json"
    save_result(out, result, session)
    m = result.metrics
    print(f"setup={result.setup} method={result.method}")
    print(
        f"inliers={m.get('n')} accepted={m.get('n_accepted')} captured={m.get('n_captured')} "
        f"rejected={m.get('n_rejected')}"
    )
    print(
        f"rms trans={m['rms_trans_m']*1000:.2f} mm  rot={m['rms_rot_deg']:.3f} deg  "
        f"max trans={m['max_trans_m']*1000:.2f} mm  max rot={m['max_rot_deg']:.3f} deg"
    )
    if result.T_base_cam is not None:
        print("T_base_cam (arm base → camera optical):\n", np.array2string(result.T_base_cam, precision=5))
    if result.T_ee_cam is not None:
        print("T_ee_cam (flange TCP → camera optical):\n", np.array2string(result.T_ee_cam, precision=5))
    warnings = quality_gate(result)
    for w in warnings:
        print("WARNING:", w)
    print(f"wrote {out}")
    return 1 if warnings else 0


def _resolve_result(args, session: Session):
    result_path = Path(args.result) if args.result else session.root / "result.json"
    result, result_intr, _payload = load_result(result_path)
    return result, result_intr or session.intrinsics, result_path


def cmd_verify(args) -> int:
    session = Session.load(args.holdout_session) if args.holdout_session else Session.load(args.session)
    result, intrinsics, result_path = _resolve_result(args, Session.load(args.session))
    if session.intrinsics is None and intrinsics is not None:
        session.intrinsics = intrinsics
    report = verify_session(
        session,
        result,
        intrinsics=intrinsics,
        min_corners=int(args.min_corners),
        max_reproj_px=float(args.max_reproj),
        write_overlays=not args.no_overlays,
    )
    out = Path(args.out) if args.out else session.root / "verify_report.json"
    out.write_text(json.dumps(report.to_dict(), indent=2), encoding="utf-8")
    print(f"setup={report.setup}  result={result_path}  frames={report.n}")
    print(
        f"pose rms {report.rms_trans_m*1000:.2f} mm / {report.rms_rot_deg:.3f} deg  "
        f"max {report.max_trans_m*1000:.2f} mm / {report.max_rot_deg:.3f} deg"
    )
    print(
        f"reproj rms {report.rms_reproj_px:.2f} px  max {report.max_reproj_px:.2f} px  "
        f"3D origin rms {report.rms_origin_m*1000:.2f} mm"
    )
    warnings = verify_quality_gate(
        report,
        max_rms_trans_m=float(args.max_rms_trans),
        max_rms_rot_deg=float(args.max_rms_rot),
        max_rms_reproj_px=float(args.max_rms_reproj),
        max_rms_origin_m=float(args.max_rms_origin),
    )
    for w in warnings:
        print("WARNING:", w)
    print(f"wrote {out}")
    return 0 if not warnings else 1


def cmd_verify_live(args) -> int:
    session = Session.load(args.session)
    result, intrinsics, result_path = _resolve_result(args, session)
    if not args.setup:
        args.setup = result.setup
    if not getattr(args, "side", None):
        args.side = session.arm_side
    if not getattr(args, "camera_role", None):
        args.camera_role = session.camera_role
    cfg = _load(args)
    spec = session.board
    K = intrinsics or session.intrinsics
    if K is None:
        raise SystemExit("result.json / session 里没有内参，先 calibrate")
    obs = _open_obs(cfg, args)
    fk = _fk(cfg, args)
    print(summarize(cfg, obs))
    print(f"live verify  result={result_path}")
    snap_dir = Path(args.out) if args.out else session.root / "verify_live"
    snap_dir.mkdir(parents=True, exist_ok=True)
    snaps = []
    use_gui = bool(args.display if args.display is not None else False)
    try:
        while True:
            rgb, q, _, _ = grab_pair(obs, arm=cfg["arm_device"], cam=cfg["camera_device"])
            det = detect_charuco(rgb, spec, rgb=True)
            T_obs = T_pred = sample = None
            try:
                T_ee = fk.fk_T(q)
                T_obs, T_pred, sample = evaluate_frame(
                    setup=result.setup,
                    T_base_ee=T_ee,
                    detection=det,
                    result=result,
                    intrinsics=K,
                    min_corners=int(args.min_corners),
                    max_reproj_px=float(args.max_reproj),
                )
            except Exception as exc:
                print(f"frame eval failed: {exc}")
            bgr = draw_verify_overlay(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), det, T_obs, T_pred, K, sample)
            if not use_gui:
                if sample is not None:
                    snaps.append(sample)
                break
            cv2.imshow("handeye_roma_verify_live", bgr)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key in (32, 13) and sample is not None:
                idx = len(snaps)
                cv2.imwrite(str(snap_dir / f"{idx:04d}.png"), bgr)
                snaps.append(sample)
                print(
                    f"  snap #{idx} reproj={sample.reproj_px:.2f}px "
                    f"pose={sample.pose_trans_m*1000:.1f}mm origin={sample.origin_err_m*1000:.1f}mm"
                )
    finally:
        if use_gui:
            cv2.destroyAllWindows()
    if not snaps:
        print("没有保存验证帧。")
        return 2
    reproj = [s.reproj_px for s in snaps]
    origin = [s.origin_err_m for s in snaps]
    print(
        f"live snaps={len(snaps)}  rms reproj={float(np.sqrt(np.mean(np.square(reproj)))):.2f}px  "
        f"rms origin={float(np.sqrt(np.mean(np.square(origin))))*1000:.2f} mm"
    )
    (snap_dir / "live_snaps.json").write_text(
        json.dumps([s.to_dict() for s in snaps], indent=2), encoding="utf-8"
    )
    return 0


def cmd_map_pixel(args) -> int:
    from handeye_charuco.eval_map import map_uv_to_base

    wh = tuple(int(x) for x in args.image_size.lower().split("x"))
    uv = [float(x) for x in args.uv.replace(",", " ").split()]
    out = map_uv_to_base(args.result, uv, float(args.z), wh)
    print(json.dumps(out, indent=2))
    return 0


def cmd_reproject(args) -> int:
    from handeye_charuco.eval_map import draw_reproject_overlay, reproject_eef
    from handeye_charuco.ingest import load_rgb, parse_floats

    eef = parse_floats(args.eef, 7)
    if args.image:
        rgb = load_rgb(args.image)
        wh = (int(rgb.shape[1]), int(rgb.shape[0]))
    else:
        rgb = None
        wh = tuple(int(x) for x in args.image_size.lower().split("x"))
    out = reproject_eef(args.result, eef, wh)
    print(json.dumps(out, indent=2))
    if args.out:
        if rgb is None:
            raise SystemExit("--out needs --image")
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        vis = draw_reproject_overlay(bgr, out["uv"])
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.out), vis)
        print("wrote", args.out)
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="handeye_charuco_roma",
        description="roma Observe Charuco hand-eye (core solver lives in ../handeye_charuco)",
    )
    p.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    p.add_argument("--roma-root", dest="roma_root", default=None)
    p.add_argument("--sdk-root", dest="sdk_root", default=None)
    p.add_argument("--setup", choices=["eye_in_hand", "eye_to_hand"])
    p.add_argument("--side", choices=["left", "right"])
    p.add_argument("--camera-role", dest="camera_role", default=None)
    p.add_argument("--arm-device", dest="arm_device", default=None)
    p.add_argument("--camera-device", dest="camera_device", default=None)
    p.add_argument("--fake", action="store_true", help="in-process Observe (tests)")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("check-config", help="print roma device binding; optionally ping Observe")
    s.add_argument("--offline", action="store_true", help="do not connect Observe")
    s.set_defaults(func=cmd_check_config)

    s = sub.add_parser("check-board")
    s.set_defaults(func=cmd_check_board)

    s = sub.add_parser("print-board")
    s.add_argument("--out", default=str(PACKAGE_ROOT / "runs" / "charuco_board.png"))
    s.add_argument("--dpi", type=int, default=300)
    s.set_defaults(func=cmd_print_board)

    s = sub.add_parser("preview")
    s.add_argument("--display", action=argparse.BooleanOptionalAction, default=None)
    s.set_defaults(func=cmd_preview)

    s = sub.add_parser("capture", help="drag-record joints, then replay to grab eef+image")
    s.add_argument("--out", default=None)
    s.add_argument("--mode", choices=["teach", "auto"], default="teach")
    s.add_argument("--display", action=argparse.BooleanOptionalAction, default=None)
    s.add_argument("--replay-only", action="store_true", help="skip teach; replay joints.json")
    s.add_argument("--no-replay", action="store_true", help="only record joints, do not move")
    s.add_argument("--settle-s", dest="settle_s", type=float, default=None)
    s.add_argument("--gap-s", dest="gap_s", type=float, default=None)
    s.set_defaults(func=cmd_capture)

    s = sub.add_parser("label-overlays", help="rewrite overlays with EE xyz/rpy stamped")
    s.add_argument("session")
    s.set_defaults(func=cmd_label_overlays)

    s = sub.add_parser("drop-frames", help="remove samples by index (images + overlays)")
    s.add_argument("session")
    s.add_argument("indices", nargs="+", type=int)
    s.set_defaults(func=cmd_drop_frames)

    s = sub.add_parser("calibrate")
    s.add_argument("session")
    s.add_argument("--out", default=None)
    s.add_argument("--min-corners", default=8, type=int)
    s.add_argument("--max-reproj", default=3.0, type=float)
    s.add_argument("--no-refine", action="store_true")
    s.add_argument("--no-reject", action="store_true")
    s.add_argument("--force-intrinsics", action="store_true")
    s.add_argument("--no-refine-intrinsics", action="store_true")
    s.set_defaults(func=cmd_calibrate)

    s = sub.add_parser("verify")
    s.add_argument("session")
    s.add_argument("--result", default=None)
    s.add_argument("--holdout-session", default=None)
    s.add_argument("--out", default=None)
    s.add_argument("--min-corners", default=8, type=int)
    s.add_argument("--max-reproj", default=4.0, type=float)
    s.add_argument("--max-rms-trans", default=0.008, type=float)
    s.add_argument("--max-rms-rot", default=1.5, type=float)
    s.add_argument("--max-rms-reproj", default=4.0, type=float)
    s.add_argument("--max-rms-origin", default=0.010, type=float)
    s.add_argument("--no-overlays", action="store_true")
    s.set_defaults(func=cmd_verify)

    s = sub.add_parser("verify-live")
    s.add_argument("session")
    s.add_argument("--result", default=None)
    s.add_argument("--out", default=None)
    s.add_argument("--min-corners", default=8, type=int)
    s.add_argument("--max-reproj", default=4.0, type=float)
    s.add_argument("--display", action=argparse.BooleanOptionalAction, default=None)
    s.add_argument("--allow-eye-in-hand", action="store_true")
    s.set_defaults(func=cmd_verify_live)

    s = sub.add_parser("map-pixel")
    s.add_argument("result")
    s.add_argument("--uv", required=True)
    s.add_argument("--z", required=True, type=float)
    s.add_argument("--image-size", default="1280x800")
    s.set_defaults(func=cmd_map_pixel)

    s = sub.add_parser("reproject")
    s.add_argument("result")
    s.add_argument("--eef", required=True)
    s.add_argument("--image", default=None)
    s.add_argument("--image-size", default="1280x800")
    s.add_argument("--out", default=None)
    s.set_defaults(func=cmd_reproject)

    return p


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return int(args.func(args) or 0)


if __name__ == "__main__":
    raise SystemExit(main())
