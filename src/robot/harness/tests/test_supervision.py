"""Event-driven image checks, explicit sensor policy and persistent attempts."""
import json
import threading
import pytest

from robot.harness.progress import EpisodeProgress
from robot.harness.supervision import verification_trigger
from robot.harness.tests.test_policy import AffineKinematics, stage, rows
from robot.harness.tests.test_vision import VisualSession, load_example


def payload(messages):
    return json.loads(messages[1]['content'][0]['text'])


def test_close_then_lift_verified_without_controller_done(tmp_path):
    example, session, checks, controls = load_example(), VisualSession(), [], []
    session.gripper = [1.]
    def controller(messages):
        controls.append(payload(messages))
        if len(controls) == 1:
            return {'action': 'set_gripper', 'gripper': 0, 'reason': 'close'}
        assert controls[-1]['stage']['motion'] == 'LIFT'
        return {'action': 'move', 'direction': 'up', 'reason': 'probe'}
    def verifier(messages):
        checks.append(messages)
        assert set(payload(messages)) == {'stage'}
        assert sum(p['type'] == 'image_url' for p in messages[1]['content']) == 4
        return {'complete': True, 'reason': 'scripted image verdict'}
    out = tmp_path / 'auto'
    example.run_loop(session, controller, threading.Event(), out, task='lift',
        planner=lambda _: {'stages': [stage(), stage('lift', 'LIFT')]}, verifier=verifier,
        legacy_actions=False, kinematics=AffineKinematics(), interval=.001, max_rounds=8, verification_interval=6)
    turns = rows(out / 'turns.jsonl')
    assert len(turns) == 2 and len(checks) == 2
    assert [r['verification_trigger'] for r in turns] == ['close_attempt', 'closed_upward_probe']
    assert all(r['status'] == 'succeeded' for r in turns)  # Motion outcome remains separate.
    assert controls[-1]['task_progress']['last_image_check']['complete']


def test_ambiguous_check_is_returned_to_controller_without_false_success(tmp_path):
    example, session, controls = load_example(), VisualSession(), []
    def controller(messages):
        controls.append(payload(messages))
        return ({'action': 'set_gripper', 'gripper': 0, 'reason': 'try'} if len(controls) == 1
                else {'action': 'wait', 'reason': 'inspect'})
    example.run_loop(session, controller, threading.Event(), tmp_path / 'false', task='grasp',
        planner=lambda _: {'stages': [stage()]}, verifier=lambda _: {'complete': False, 'reason': 'rim occluded'},
        legacy_actions=False, interval=.001, max_rounds=2, verification_interval=6)
    progress = controls[-1]['task_progress']
    assert progress['last_image_check']['reason'] == 'rim occluded'
    assert progress['completed_stages'] == []
    assert progress['grasp_attempts'][0]['last_image_check']['complete'] is False
    assert controls[-1]['recent_actions'][0]['verification']['complete'] is False


def test_enter_during_automatic_verifier_prevents_next_motion(tmp_path):
    example, session, stop, calls = load_example(), VisualSession(), threading.Event(), []
    def verifier(messages):
        stop.set()
        return {'complete': True, 'reason': 'late'}
    def controller(messages):
        calls.append(messages)
        return {'action': 'set_gripper', 'gripper': 0, 'reason': 'close'}
    example.run_loop(session, controller, stop, tmp_path / 'stop', task='grasp',
        planner=lambda _: {'stages': [stage()]}, verifier=verifier, legacy_actions=False,
        interval=.001, max_rounds=8, verification_interval=6)
    assert len(calls) == 1 and len(session.writes) == 1 and session.released
    assert rows(tmp_path / "stop" / "turns.jsonl")[0]["status"] == "succeeded"


def test_rgb_only_rejects_depth_before_any_point_resolution(tmp_path, monkeypatch):
    example, session = load_example(), VisualSession()
    def forbidden(*args, **kwargs):
        pytest.fail('depth resolver must not be reached')
    monkeypatch.setattr(example, 'interpret', forbidden)
    out = tmp_path / 'rgb'
    def controller(messages):
        assert payload(messages)['sensor_policy']['single_camera_depth_point_allowed'] is False
        return {'action': 'point', 'camera': 'wrist', 'pixel': [2, 2], 'reason': 'query'}
    example.run_loop(session, controller, threading.Event(), out, task='rgb',
        planner=lambda _: {'stages': [stage()]}, legacy_actions=False,
        interval=.001, max_rounds=1, allow_depth_point=False)
    assert rows(out / 'turns.jsonl')[0]['status'] == 'rejected'
    assert not session.writes


@pytest.mark.parametrize('status,confirmed,recovered,expected', [
    ('succeeded', True, False, 'close_attempt'), ('failed', True, True, 'close_attempt'),
    ('failed', False, True, None), ('unknown', True, True, None), ('cancelled', True, True, None)])
def test_verification_does_not_bypass_unconfirmed_execution(status, confirmed, recovered, expected):
    record = {'status': status, 'stop_confirmed': confirmed, 'action': {'action': 'set_gripper', 'gripper': 0},
              'recovery': {'status': 'ready' if recovered else 'failed'}}
    assert verification_trigger(record, stage(), interval=6, stage_turns=1,
                                grasp_state='not_attempted') == expected


def test_attempt_memory_survives_planning_and_separates_robot_lift_from_object_lift():
    progress = EpisodeProgress()
    progress.enter_stage(stage())
    close = {'action': {'action': 'set_gripper', 'gripper': 0}, 'status': 'succeeded'}
    progress.record(close, stage(), 1)
    progress.record({'action': {'action': 'move', 'direction': 'up'}, 'status': 'succeeded',
                     'measured_effect': {'delta_base_m': [0, 0, .01]}}, stage(), 2)
    progress.begin_plan()
    progress.record(close, stage(), 3)
    assert progress.snapshot()['grasp_attempts'][0]['upward_travel_m'] == .01
    assert progress.attempt_count == 1 and progress.attempts[0]['close_commands'] == 2
    assert progress.grasp_state == 'close_attempted_unverified'
    progress.record({'action': {'action': 'set_gripper', 'gripper': 1}, 'status': 'succeeded'}, stage(), 4)
    progress.record(close, stage(), 5)
    assert progress.attempt_count == 2 and progress.attempts[0]['ended_by_open']
    assert progress.completed == []


def test_diagnostics_distinguishes_closing_execution_from_image_verification(tmp_path):
    from robot.harness.diagnostics import summarize_run
    records = [
        {'round': 1, 'action': {'action': 'set_gripper', 'gripper': 0}, 'status': 'succeeded'},
        {'round': 2, 'action': {'action': 'point', 'camera': 'wrist'}, 'status': 'queried'},
        {'round': 3, 'action': {'action': 'move', 'direction': 'up'}, 'status': 'succeeded',
         'verification': {'complete': False, 'reason': 'occluded'}}]
    (tmp_path / 'turns.jsonl').write_text('\n'.join(map(json.dumps, records)))
    (tmp_path / 'latest_messages.json').write_text('MUST_NOT_READ')
    report = summarize_run(tmp_path)
    assert report['close_rounds'] == [1]
    assert report['single_camera_depth_rounds'] == [2]
    assert len(report['image_checks']) == 1 and not report['image_checks'][0]['complete']
    assert 'MUST_NOT_READ' not in json.dumps(report)


def test_open_command_is_not_a_closed_periodic_check():
    record = {"status": "succeeded", "action": {"action": "set_gripper", "gripper": 1}}
    assert verification_trigger(record, stage('lift', 'LIFT'), interval=6, stage_turns=6,
                                grasp_state='close_attempted_unverified') is None
