"""Tool-driven migration: no task budgets, public RGB-D, latched gripper."""
import json
import threading
import time
import numpy as np
import pytest
from robot.harness.robowork import Workspace, normalized_pixel, run_workspace, load_workspace_resume
from robot.harness.robowork_executor import LatchedExecutor
from robot.harness.vision import LOWER, UPPER, Stopped
from robot.harness.tests.test_grounded_tools import GroundedSession
from robot.harness.tests.test_policy import AffineKinematics


class Scene(GroundedSession):
    blocked_gripper = False
    def read_raw(self, name):
        result=super().read_raw(name)
        for cam in result['data']['cameras'].values():
            cam['depth_m']=np.full((128,128), .5)
        return result
    def write_target(self,name,payload,**kwargs):
        result=super().write_target(name,payload,**kwargs)
        if self.blocked_gripper and payload['gripper']==0:
            self.gripper=[.3]
        return result


def executor(tmp_path,session,stop):
    return LatchedExecutor(session,stop=stop,lower=LOWER,upper=UPPER,journal=tmp_path/'control.jsonl')


def test_blocked_close_remains_latched_through_motion_and_idle(tmp_path):
    scene=Scene();scene.blocked_gripper=True;stop=threading.Event()
    with executor(tmp_path,scene,stop) as ex:
        result=ex.command(scene.q,gripper=0,timeout_s=.08,settle_s=.01,close=True)
        assert result['joint_arrived'] and result['measured_gripper']==.3
        assert result['grasp_detected']=='unknown'
        q=list(scene.q);q[0]+=.02
        assert ex.command(q,timeout_s=.15,settle_s=.01)['joint_arrived']
        time.sleep(.55)
        assert all(row['gripper']==0 for row in scene.writes)
    assert scene.released


def test_stop_during_command_and_no_late_write(tmp_path):
    scene=Scene();stop=threading.Event()
    with executor(tmp_path,scene,stop) as ex:
        timer=threading.Timer(.08,stop.set);timer.start()
        with pytest.raises(Stopped):
            ex.command(scene.q,gripper=0,timeout_s=1,close=True)
        timer.join()
        time.sleep(.15)
        count=len(scene.writes)
        time.sleep(.1)
        assert len(scene.writes)==count


def test_feedback_stale_during_idle_stops_refresh(tmp_path):
    scene=Scene();stop=threading.Event()
    # Mock frozen camera subclass refreshes outer data only; main stamp stays old.
    ex=executor(tmp_path,scene,stop)
    try:
        ex.command(scene.q,timeout_s=.3,settle_s=.01)
        original=scene.read_raw('left_arm')
        scene.read_raw=lambda _:original
        assert stop.wait(1.)
        assert 'stale' in ex.fault
    finally:
        # Restore readable stop telemetry so cleanup can be verified.
        del scene.read_raw
        ex.close()


def test_normalized_pixels_match_endpoints_and_reject_bad_values():
    cam={'rgb_shape':(480,640)}
    assert normalized_pixel(cam,1000,1000)==[639,479]
    assert normalized_pixel(cam,0,0)==[0,0]
    with pytest.raises(ValueError):normalized_pixel(cam,1001,50)
    with pytest.raises(ValueError):normalized_pixel(cam,True,50)


def test_tool_plans_no_centimetre_floor_and_single_use(tmp_path):
    scene=Scene();stop=threading.Event()
    with executor(tmp_path,scene,stop) as ex:
        work=Workspace(scene,ex,AffineKinematics(),tmp_path,stop);work.capture()
        result=work.dispatch(dict(tool='plan_eef',arguments=dict(frame='base',delta_m=[.002,0,0],rot_deg=[0,0,0])))
        assert not scene.writes
        call=dict(tool='exec_plan_pose',arguments=dict(plan_id=result['plan_id']))
        assert work.dispatch(call)['completed_waypoints']==1
        with pytest.raises(ValueError,match='unknown/consumed'):work.dispatch(call)
        work.capture()
        result=work.dispatch(dict(tool='plan_eef',arguments=dict(frame='base',delta_m=[.06,0,0],rot_deg=[0,0,0])))
        assert result['waypoints']>=6
        messages=work.messages('pick',[])
        assert 'SECRET' not in json.dumps(messages)
        assert 'max_replans' not in json.dumps(messages)


def test_depth_bound_to_current_image_and_no_truth(tmp_path):
    scene=Scene();stop=threading.Event()
    with executor(tmp_path,scene,stop) as ex:
        work=Workspace(scene,ex,AffineKinematics(),tmp_path,stop);work.capture()
        args=dict(camera='wrist',image_id=work.obs['image_ids']['wrist'],u=500,v=500)
        result=work.dispatch(dict(tool='read_depth',arguments=args))
        assert result['depth_m']==.5
        assert 'SECRET' not in json.dumps(result)
        work.capture()
        with pytest.raises(ValueError,match='latest image'):work.dispatch(dict(tool='read_depth',arguments=args))


def test_more_than_three_errors_do_not_end_agent_run(tmp_path):
    calls=[]
    def policy(messages):
        calls.append(messages)
        return dict(tool='unknown_tool',arguments={},reason='scripted invalid action')
    out=tmp_path/'run'
    run_workspace(Scene(),policy,threading.Event(),out,task='test',kinematics=AffineKinematics(),max_rounds=7,interval=0)
    assert len(calls)==7
    summary=json.loads((out/'run_summary.json').read_text())
    assert summary['exit_reason']=='max_rounds'
    resumed=load_workspace_resume(out)
    assert resumed['last_round']==7 and resumed['task']=='test'
    assert 'plan' not in resumed and 'gripper_command' not in resumed


def test_finish_is_agent_assessment_not_external_verification(tmp_path):
    out=tmp_path/'run'
    run_workspace(Scene(),lambda _:dict(tool='finish',arguments={'outcome':'completed','evidence':'scripted test, no actual task'}),
        threading.Event(),out,task='test',kinematics=AffineKinematics(),max_rounds=5,interval=0)
    result=json.loads((out/'run_summary.json').read_text())
    assert result['agent_outcome']['agent_outcome']=='completed'
    assert not result['independently_verified'] and not result['task_complete']


def test_skills_are_loaded_explicitly(tmp_path):
    scene=Scene();stop=threading.Event()
    with executor(tmp_path,scene,stop) as ex:
        work=Workspace(scene,ex,AffineKinematics(),tmp_path,stop);work.capture()
        assert 'TASK SKILL:' not in work.messages('pick',[])[0]['content']
        work.dispatch(dict(tool='activate_skill',arguments={'name':'pick-up-object'}))
        assert 'TASK SKILL:' in work.messages('pick',[])[0]['content']
        work.dispatch(dict(tool='deactivate_skill',arguments={}))
        assert 'TASK SKILL:' not in work.messages('pick',[])[0]['content']


def test_permanent_api_error_ends_without_retries_or_motion(tmp_path):
    from robot.harness.vision import ModelAPIError
    calls=[];scene=Scene()
    def policy(_):
        calls.append(1)
        raise ModelAPIError(400)
    out=tmp_path/'rejected'
    run_workspace(scene,policy,threading.Event(),out,task='test',kinematics=AffineKinematics(),max_rounds=0,interval=0)
    assert calls==[1] and not scene.writes
    assert json.loads((out/'run_summary.json').read_text())['exit_reason']=='model_api_rejected'
    assert not ModelAPIError(401).retryable
    assert ModelAPIError(429).retryable and ModelAPIError(503).retryable


def test_ik_runtimeerror_returns_to_agent_without_any_motion(tmp_path):
    class IKFailure(AffineKinematics):
        def solve(self,*args,**kwargs):
            raise RuntimeError('inverse solution did not converge, residual 0.0100 m')
    calls=[];scene=Scene()
    def policy(messages):
        calls.append(messages)
        if len(calls)==1:
            return dict(tool='plan_eef',arguments=dict(frame='base',delta_m=[0,0,.08],rot_deg=[0,0,0]))
        payload=json.loads(messages[1]['content'][0]['text'])
        result=payload['recent_tool_history'][-1]['result']
        assert result['kind']=='ik_not_converged' and result['motion_written'] is False
        return dict(tool='finish',arguments=dict(outcome='abandoned',evidence='scripted recovery check'))
    run_workspace(scene,policy,threading.Event(),tmp_path/'run',task='test',kinematics=IKFailure(),max_rounds=2,interval=0)
    assert len(calls)==2 and scene.writes==[]


def test_close_reports_joint_deflection_without_false_arrival(tmp_path):
    class Deflected(Scene):
        def write_target(self,name,payload,**kwargs):
            result=super().write_target(name,payload,**kwargs)
            if payload['gripper']==0:
                self.q=list(self.q);self.q[0]+=.009
                self.gripper=[.425]
            return result
    scene=Deflected();stop=threading.Event()
    with executor(tmp_path,scene,stop) as ex:
        result=ex.command(list(scene.q),gripper=0,close=True,timeout_s=.08,settle_s=.01)
        assert result['gripper_observation_complete']
        assert not result['joint_arrived'] and 'warning' in result and 'error' not in result
        assert result['gripper_command']==0 and result['measured_gripper']==.425
        assert ex.intent['gripper']==0


def test_cleanup_does_not_mask_original_error(tmp_path,monkeypatch):
    scene=Scene();ex=executor(tmp_path,scene,threading.Event())
    def unconfirmed(*a,**kw):raise RuntimeError('position stop could not be verified')
    monkeypatch.setattr(ex,'_stop_and_confirm',unconfirmed)
    with pytest.raises(ValueError,match='original') as caught:
        with ex:
            raise ValueError('original planning defect')
    assert 'shutdown hold was not confirmed' in caught.value.__notes__[0]
    assert ex.shutdown_error=='position stop could not be verified'
    assert scene.released


def test_cleanup_failure_alone_still_raises(tmp_path,monkeypatch):
    scene=Scene();ex=executor(tmp_path,scene,threading.Event())
    def unconfirmed(*a,**kw):raise RuntimeError('position stop could not be verified')
    monkeypatch.setattr(ex,'_stop_and_confirm',unconfirmed)
    with pytest.raises(RuntimeError,match='stop could not'):
        with ex: pass
    assert scene.released


def test_terminal_stop_sources_are_distinct_and_first_source_preserved():
    import io
    from robot.harness.stop_signal import StopSignal, watch_terminal
    for value,reason in [('\n','enter_pressed'),('','stdin_eof')]:
        stop=StopSignal();watch_terminal(io.StringIO(value),stop)
        assert stop.is_set() and stop.reason==reason
        stop.request('runtime_monitor_fault')
        assert stop.reason==reason


def test_runtime_stop_while_waiting_for_model_is_not_user_stop(tmp_path,monkeypatch):
    from robot.harness.stop_signal import StopSignal
    from robot.harness import robowork
    stop=StopSignal();scene=Scene();real=robowork.LatchedExecutor
    holders=[]
    def factory(*a,**kw):
        ex=real(*a,**kw);holders.append(ex);return ex
    monkeypatch.setattr(robowork,'LatchedExecutor',factory)
    def policy(_):
        holders[0].fault='control lease lost'
        stop.request('runtime_monitor_fault')
        return dict(tool='move_joints',arguments={'joint_rad':[0]*6})
    out=tmp_path/'fault'
    run_workspace(scene,policy,stop,out,task='test',kinematics=AffineKinematics(),max_rounds=1,interval=0)
    summary=json.loads((out/'run_summary.json').read_text())
    assert summary['exit_reason']=='runtime_monitor_fault'
    assert summary['runtime_error']=='control lease lost'
    assert not scene.writes


def test_enter_during_inference_is_reported_and_late_action_discarded(tmp_path):
    from robot.harness.stop_signal import StopSignal
    stop=StopSignal();scene=Scene()
    def policy(_):
        stop.request('enter_pressed')
        return dict(tool='close_gripper',arguments={})
    out=tmp_path/'enter'
    run_workspace(scene,policy,stop,out,task='test',kinematics=AffineKinematics(),max_rounds=1,interval=0)
    summary=json.loads((out/'run_summary.json').read_text())
    assert summary['exit_reason']=='enter_pressed' and summary['runtime_error'] is None
    assert not scene.writes


def test_transient_feedback_gap_waits_for_fresh_frame_without_writes(tmp_path):
    scene=Scene();stop=threading.Event()
    with executor(tmp_path,scene,stop) as ex:
        # Test the read contract without racing the periodic watcher.
        ex.worker_done.set();ex.worker.join()
        original=scene.read_raw
        start=time.monotonic()
        frame=original('left_arm');frame['timestamp_ns']=time.time_ns()-150_000_000
        scene.read_raw=lambda name: frame if time.monotonic()-start<.06 else original(name)
        writes=len(scene.writes)
        fresh=ex.observe()
        assert fresh['age_ms']<=100 and len(scene.writes)==writes
        assert not stop.is_set()
        events=[json.loads(s)['event'] for s in (tmp_path/'control.jsonl').read_text().splitlines()]
        assert 'feedback_gap_wait' in events and 'feedback_gap_recovered' in events


def test_persistent_or_future_feedback_still_fails_without_target_write(tmp_path):
    from robot.harness.executor import FeedbackAgeError
    scene=Scene();stop=threading.Event()
    with executor(tmp_path,scene,stop) as ex:
        ex.worker_done.set();ex.worker.join()
        original=scene.read_raw
        for delta in (-150_000_000,100_000_000):
            frame=original('left_arm');frame['timestamp_ns']=time.time_ns()+delta
            scene.read_raw=lambda _:frame
            writes=len(scene.writes)
            try:
                with pytest.raises(FeedbackAgeError):ex.observe()
                assert len(scene.writes)==writes
            finally:
                scene.read_raw=original
