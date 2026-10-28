"""RoboHarness-inspired RGB tracking and single-use motion planning contracts."""
import json
import threading
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from robot.harness.policy import parse_action
from robot.harness.progress import EpisodeProgress
from robot.harness.visual_tracking import RGBTracks, image_id
from robot.harness.visual_plans import MotionPlans, observation_id
from robot.harness.vision import observation, build_messages
from robot.harness.tests.test_policy import AffineKinematics, stage, rows
from robot.harness.tests.test_vision import VisualSession, load_example

cv2 = pytest.importorskip('cv2')


def texture():
    rng = np.random.default_rng(24)
    gray = cv2.GaussianBlur(rng.integers(0, 256, (128, 128), dtype=np.uint8), (3, 3), 0)
    return np.repeat(gray[:, :, None], 3, axis=2)


def frame(rgb, stamp):
    return {'wrist': {'rgb': rgb, 'timestamp_ns': stamp, 'depth_m': 'SECRET_DEPTH', 'objects': 'SECRET_OBJECTS'}}


class GroundedSession(VisualSession):
    def __init__(self):
        super().__init__()
        self.rgb = texture()
    def read_raw(self, name):
        value = super().read_raw(name)
        for camera in value['data']['cameras'].values():
            camera.update(rgb=self.rgb.copy(), intrinsics=np.array([[100, 0, 64], [0, 100, 64], [0, 0, 1]]),
                          world_from_camera=np.array([0, 0, 0, 1, 0, 0, 0]))
        return value


def obs_for(session):
    cameras = {}
    obs, stamps = observation(session, local=cameras)
    obs.update(observation_id=observation_id(stamps), image_ids={n: image_id(n, t) for n, t in stamps.items()})
    return obs, cameras


def plan_action(obs, **kw):
    return {'action': 'plan_motion', 'observation_id': obs['observation_id'], 'frame': 'base',
            'delta_cm': [1, 0, 0], 'reason': 'read-only preview', **kw}


def execute_action(obs, identifier):
    return {'action': 'execute_plan', 'observation_id': obs['observation_id'], 'plan_id': identifier, 'reason': 'execute inspected preview'}


def test_rgb_tracks_follow_motion_then_fail_closed_on_occlusion_without_depth():
    tracks = RGBTracks()
    rgb = texture(); tracks.update(frame(rgb, 1))
    first = tracks.register('wrist', 'wrist:1', [64, 64], 'rim')
    shifted = cv2.warpAffine(rgb, np.float32([[1, 0, 3], [0, 1, -2]]), (128, 128))
    tracks.update(frame(shifted, 2))
    current = tracks.get(first['track_id'])
    assert current['status'] == 'observed'
    np.testing.assert_allclose(current['pixel'], [67, 62], atol=.35)
    assert not current['identity_verified'] and 'SECRET' not in json.dumps(tracks.summary())
    tracks.update(frame(np.zeros_like(rgb), 3))
    assert tracks.get(first['track_id'])['pixel'] is None
    assert tracks.get(first['track_id'])['status'] == 'lost'
    tracks.update(frame(rgb, 4))
    assert tracks.get(first['track_id'])['status'] == 'lost'  # Never silently rebind identity.


def test_registration_requires_exact_frame_texture_and_capacity():
    tracks = RGBTracks(capacity=1); tracks.update(frame(texture(), 10))
    with pytest.raises(ValueError, match='image_id'):
        tracks.register('wrist', 'wrist:9', [64, 64], 'rim')
    track = tracks.register('wrist', 'wrist:10', [64, 64], 'rim')
    with pytest.raises(ValueError, match='capacity'):
        tracks.register('wrist', 'wrist:10', [64, 64], 'pad')
    tracks.forget(track['track_id'])
    tracks.update(frame(np.zeros((128, 128, 3), np.uint8), 11))
    with pytest.raises(ValueError, match='texture'):
        tracks.register('wrist', 'wrist:11', [64, 64], 'rim')


def test_pairs_are_same_camera_pixels_not_world_distances():
    tracks = RGBTracks(); tracks.update(frame(texture(), 10))
    tracks.register('wrist', 'wrist:10', [50, 64], 'pad')
    tracks.register('wrist', 'wrist:10', [64, 64], 'rim')
    result = tracks.summary()
    assert result['pairs'][0]['distance_pixels'] == 14
    assert not result['metric_depth_available']


def test_camera_frame_translation_and_rotation_use_captured_calibration():
    session, kin, plans, tracks = GroundedSession(), AffineKinematics(), MotionPlans(), RGBTracks()
    obs, cameras = obs_for(session)
    cameras['wrist']['world_from_camera'] = np.array([0, 0, 0, 2**-.5, 0, 0, 2**-.5])
    preview = plans.create(plan_action(obs, frame='wrist_camera', rot_deg=[2, 0, 0]), obs, cameras, kin, tracks)
    np.testing.assert_allclose(preview['resolved_delta_base_m'], [0, .01, 0], atol=1e-10)
    np.testing.assert_allclose(preview['rotation_base_rad'], [0, np.deg2rad(2), 0], atol=1e-10)
    assert not session.writes and preview['motion_written'] is False
    target, resolution = plans.admit(execute_action(obs, preview['plan_id']), obs,
        {'joint': session.q, 'gripper': 0}, kin, tracks)
    assert resolution['waypoints'] and target['joint'] == resolution['waypoints'][-1]['joint']
    with pytest.raises(ValueError, match='consumed'):
        plans.admit(execute_action(obs, preview['plan_id']), obs, {'joint': session.q, 'gripper': 0}, kin, tracks)


@pytest.mark.parametrize('change', ['time', 'epoch', 'joint', 'gripper', 'observation'])
def test_plan_admission_rejects_stale_or_changed_state(change):
    clock = [0.]
    plans, tracks, session, kin = MotionPlans(clock=lambda: clock[0]), RGBTracks(), GroundedSession(), AffineKinematics()
    obs, cameras = obs_for(session)
    preview = plans.create(plan_action(obs), obs, cameras, kin, tracks)
    measured = {'joint': list(session.q), 'gripper': 0}
    action = execute_action(obs, preview['plan_id'])
    if change == 'time': clock[0] = 121
    if change == 'epoch': plans.invalidate_for_motion()
    if change == 'joint': measured['joint'][0] += .02
    if change == 'gripper': measured['gripper'] = .1
    if change == 'observation': action['observation_id'] = 'old'
    with pytest.raises(ValueError): plans.admit(action, obs, measured, kin, tracks)
    assert not session.writes


def test_anchor_drift_invalidates_plan_without_inventing_depth():
    plans, tracks, session, kin = MotionPlans(), RGBTracks(), GroundedSession(), AffineKinematics()
    obs, cameras = obs_for(session); tracks.update(cameras)
    track = tracks.register('wrist', obs['image_ids']['wrist'], [64, 64], 'rim')
    preview = plans.create(plan_action(obs, anchor_track_id=track['track_id']), obs, cameras, kin, tracks)
    session.rgb = cv2.warpAffine(session.rgb, np.float32([[1, 0, 5], [0, 1, 0]]), (128, 128))
    fresh, new_cameras = obs_for(session); tracks.update(new_cameras)
    with pytest.raises(ValueError, match='3 pixels'):
        plans.admit(execute_action(fresh, preview['plan_id']), fresh, {'joint': session.q, 'gripper': 0}, kin, tracks)


@pytest.mark.parametrize('update', [{'delta_cm': [.1, 0, 0]}, {'delta_cm': [5, 5, 0]}, {'rot_deg': [9, 0, 0]},
                                   {'frame': 'world'}, {'delta_cm': [True, 0, 0]}, {'joint': [0]*6}])
def test_new_protocol_does_not_relax_bounds_or_accept_model_waypoints(update):
    obs, _ = obs_for(GroundedSession())
    with pytest.raises(ValueError): parse_action(plan_action(obs, **update))


def test_loop_preview_is_read_only_then_exact_plan_is_executed(tmp_path):
    example, session, inputs = load_example(), GroundedSession(), []
    def controller(messages):
        data = json.loads(messages[1]['content'][0]['text']); inputs.append(data)
        assert not session.writes
        if len(inputs) == 1: return plan_action(data)
        preview = data['visual_panel']['motion_plans'][0]
        assert data['recent_actions'][0]['tool_result']['motion_written'] is False
        return execute_action(data, preview['plan_id'])
    out = tmp_path / 'planned'
    example.run_loop(session, controller, threading.Event(), out, task='preview then execute',
        planner=lambda _: {'stages': [stage()]}, legacy_actions=False, kinematics=AffineKinematics(),
        interval=.001, max_rounds=2, grounded_tools=True)
    turns = rows(out / 'turns.jsonl')
    assert [r['status'] for r in turns] == ['queried', 'succeeded']
    assert len(session.writes) == 1 and session.q[0] == pytest.approx(.02)
    assert 'SECRET' not in json.dumps(inputs)


def test_anchor_is_rechecked_after_model_reply_before_any_motion(tmp_path):
    example, session, count = load_example(), GroundedSession(), 0
    def controller(messages):
        nonlocal count
        count += 1; data = json.loads(messages[1]['content'][0]['text'])
        if count == 1:
            return {'action': 'track_point', 'camera': 'wrist', 'image_id': data['image_ids']['wrist'],
                    'pixel': [64, 64], 'label': 'rim', 'reason': 'register'}
        if count == 2:
            return plan_action(data, anchor_track_id=data['visual_panel']['rgb_tracking']['tracks'][0]['track_id'])
        # Simulate occlusion AFTER the controller's pictured observation.
        session.rgb[:] = 0
        return execute_action(data, data['visual_panel']['motion_plans'][0]['plan_id'])
    out = tmp_path / 'occluded'
    example.run_loop(session, controller, threading.Event(), out, task='no stale anchor',
        planner=lambda _: {'stages': [stage()]}, legacy_actions=False, kinematics=AffineKinematics(),
        interval=.001, max_rounds=3, grounded_tools=True)
    assert [r['status'] for r in rows(out / 'turns.jsonl')] == ['queried', 'queried', 'rejected']
    assert not session.writes


def test_verifier_never_receives_tracks_or_motion_previews():
    obs, _ = obs_for(GroundedSession())
    obs['visual_panel'] = {'SECRET_TRACK': 1}
    data = json.loads(build_messages(obs, 'task', [], role='verifier', stage=stage())[1]['content'][0]['text'])
    assert set(data) == {'stage'}


def test_readonly_previews_do_not_hide_measured_execution_cycle():
    progress = EpisodeProgress()
    for i, delta in enumerate((.01, -.01, .01, -.01)):
        progress.record({'action': {'action': 'plan_motion'}, 'status': 'queried'}, stage(), 2*i+1)
        progress.record({'action': {'action': 'execute_plan'}, 'status': 'succeeded',
                         'measured_effect': {'delta_base_m': [delta, 0, 0]},
                         'action_resolution': {'delta_rotvec_base_rad': [0, 0, 0]}}, stage(), 2*i+2)
    assert progress.stagnation['kind'] == 'measured_translation_cycle'


def test_read_only_preview_at_review_boundary_does_not_consume_failure_budget():
    from robot.harness.supervision import decide_next
    decision = decide_next(record={"status": "queried", "action": {"action": "plan_motion"}},
        observation={"joint_rad": [0]*6, "gripper": 1}, stagnation=False,
        stage_turns=12, stage_rounds=12, failures=0, waits=0, max_waits=3,
        replans=2, max_replans=2, recovery_plans=0, max_recovery_plans=3)
    assert decision.kind == "stage_review"
