import numpy as np
import pytest

from handeye_charuco_roma.fk import MarvinOfflineFk, native_mm_to_unified


def test_native_mm_to_unified_right_home_z():
    T = np.eye(4)
    T[2, 3] = 870.5
    out = native_mm_to_unified(T, side="right")
    np.testing.assert_allclose(np.linalg.norm(out[:3, 3]), 0.8705, atol=1e-9)
    # right R_n2u maps native +Z to unified -Y
    np.testing.assert_allclose(out[:3, 3], [0.0, -0.8705, 0.0], atol=1e-9)


def test_native_mm_to_unified_left_home_z():
    T = np.eye(4)
    T[2, 3] = 870.5
    out = native_mm_to_unified(T, side="left")
    np.testing.assert_allclose(out[:3, 3], [0.0, 0.8705, 0.0], atol=1e-9)


def test_marvin_kine_zero_joints_right():
    try:
        fk = MarvinOfflineFk(side="right")
        T = fk.fk_T(np.zeros(7))
    except FileNotFoundError as exc:
        pytest.skip(str(exc))
    assert np.all(np.isfinite(T))
    np.testing.assert_allclose(np.linalg.norm(T[:3, 3]), 0.8705, atol=1e-3)
    eef = fk.fk_eef_wxyz(np.zeros(7))
    assert eef.shape == (7,)
    np.testing.assert_allclose(np.linalg.norm(eef[3:]), 1.0, atol=1e-6)
