"""Write one session sample from a paired Observe grab + offline FK."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from handeye_charuco.charuco import BoardSpec, Detection, Intrinsics, detect_charuco, draw_detection, estimate_board_pose
from handeye_charuco.session import Sample, Session
from handeye_charuco import se3

from .joints import JointLog
from .mover import ArmMover
from .observe import ObserveLike, grab_pair, read_joint


def capture_sample(
    session: Session,
    obs: ObserveLike,
    fk,
    spec: BoardSpec,
    *,
    arm: str,
    cam: str,
    min_corners: int = 8,
    max_reproj_px: float = 3.0,
    grab_frames: int = 3,
    require_detection: bool = True,
) -> tuple[Sample, Detection, np.ndarray]:
    rgb = q = None
    for _ in range(max(int(grab_frames), 1)):
        rgb, q, _cam, _arm = grab_pair(obs, arm=arm, cam=cam)
    assert rgb is not None and q is not None
    T_ee = se3.as_se3(fk.fk_T(q))
    det = detect_charuco(rgb, spec, rgb=True)
    index = 0 if not session.samples else session.samples[-1].index + 1
    rel = session.save_image(rgb, index)
    T_cb = None
    err = float("inf")
    ok = False
    K_src = session.intrinsics
    if K_src is not None and det.n_corners >= 4:
        ok, T_cb, err = estimate_board_pose(det, K_src.camera_matrix, K_src.dist_coeffs)
        accepted = bool(ok and det.n_corners >= min_corners and err <= max_reproj_px)
    else:
        accepted = det.n_corners >= min_corners
    if require_detection and not accepted:
        sample = Sample(
            index=index,
            joint=q[:7],
            T_base_ee=T_ee,
            image_file=rel,
            n_corners=det.n_corners,
            reproj_error_px=err,
            T_cam_board=T_cb,
            accepted=False,
        )
        return sample, det, rgb
    sample = Sample(
        index=index,
        joint=q[:7],
        T_base_ee=T_ee,
        image_file=rel,
        n_corners=det.n_corners,
        reproj_error_px=err,
        T_cam_board=T_cb,
        accepted=accepted,
    )
    session.samples.append(sample)
    write_sample_overlay(session, sample, rgb, det)
    return sample, det, rgb


def annotate_overlay(bgr, sample) -> np.ndarray:
    """Stamp flange TCP xyz/rpy so a human can cull frames from overlays."""
    vis = np.asarray(bgr).copy()
    xyzrpy = se3.to_xyz_rpy(sample.T_base_ee, degrees=True)
    lines = [
        f"#{int(sample.index)}  EE xyz={xyzrpy[0]:.3f} {xyzrpy[1]:.3f} {xyzrpy[2]:.3f}",
        f"EE rpy={xyzrpy[3]:.1f} {xyzrpy[4]:.1f} {xyzrpy[5]:.1f}  corners={int(sample.n_corners)}",
    ]
    x, y0 = 12, 60
    for i, line in enumerate(lines):
        y = y0 + i * 30
        (tw, th), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
        cv2.rectangle(vis, (x - 6, y - th - 8), (x + tw + 6, y + 8), (0, 0, 0), -1)
        cv2.putText(vis, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)
    return vis


def write_sample_overlay(session: Session, sample, rgb, det: Detection) -> Path:
    session.ensure_dirs()
    bgr = cv2.cvtColor(np.asarray(rgb), cv2.COLOR_RGB2BGR)
    overlay = draw_detection(bgr, det, sample.T_cam_board, session.intrinsics)
    overlay = annotate_overlay(overlay, sample)
    path = session.overlays_dir / f"{int(sample.index):04d}.png"
    if not cv2.imwrite(str(path), overlay):
        raise RuntimeError(f"failed to write {path}")
    return path


def label_session_overlays(session: Session) -> int:
    from handeye_charuco.ingest import load_rgb

    n = 0
    spec = session.board
    for sample in session.samples:
        path = session.root / sample.image_file
        if not path.is_file():
            continue
        rgb = load_rgb(path)
        det = detect_charuco(rgb, spec, rgb=True)
        sample.n_corners = det.n_corners
        write_sample_overlay(session, sample, rgb, det)
        n += 1
    return n


def drop_samples(session: Session, indices) -> list[int]:
    want = {int(i) for i in indices}
    dropped = [s.index for s in session.samples if s.index in want]
    session.samples = [s for s in session.samples if s.index not in want]
    for idx in dropped:
        for rel in (session.root / "images" / f"{idx:04d}.png", session.overlays_dir / f"{idx:04d}.png"):
            if rel.is_file():
                rel.unlink()
    session.save()
    return dropped


def record_joint(log: JointLog, obs: ObserveLike, arm: str) -> tuple[int, np.ndarray]:
    q = read_joint(obs, arm)
    idx = log.append(q)
    log.save()
    return idx, q


def replay_waypoints(
    session: Session,
    log: JointLog,
    obs: ObserveLike,
    fk,
    spec: BoardSpec,
    mover: ArmMover,
    *,
    arm: str,
    cam: str,
    min_corners: int = 8,
    max_reproj_px: float = 3.0,
    grab_frames: int = 4,
    settle_s: float = 1.0,
    gap_s: float = 1.0,
    max_deg_s: float = 20.0,
    sleep=None,
) -> list:
    import time

    pause = time.sleep if sleep is None else sleep
    start = len(session.samples)
    out = []
    for i, q in enumerate(log.q):
        if i < start:
            continue
        mover.goto(q, max_deg_s=max_deg_s)
        if settle_s > 0:
            pause(float(settle_s))
        sample, det, _ = capture_sample(
            session,
            obs,
            fk,
            spec,
            arm=arm,
            cam=cam,
            min_corners=min_corners,
            max_reproj_px=max_reproj_px,
            grab_frames=grab_frames,
            require_detection=False,
        )
        session.save()
        out.append((sample, det))
        if gap_s > 0:
            pause(float(gap_s))
    return out
