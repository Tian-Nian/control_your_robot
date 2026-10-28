"""Piper simulation transport for a tool-driven agent with latched gripper intent.

Unlike PositionExecutor, ordinary completion does not issue a runtime stop: doing
so would replace the commanded gripper width with its measured contact width.
The live lease, fresh telemetry and renewable targets remain mandatory.
"""
import threading
import time
import numpy as np
from .executor import PositionExecutor, FeedbackAgeError, _vector
from .vision import Stopped


class LatchedExecutor(PositionExecutor):
    def __init__(self, session, *, stop, **kwargs):
        spec = next((s for s in session.describe() if s['name'] == 'left_arm'), {})
        if spec.get('class_name') != 'PiperMuJoCoController' or spec.get('class_path') != 'robot.controller.PiperMuJoCo_controller':
            raise RuntimeError('latched tool executor currently supports only Piper MuJoCo')
        super().__init__(session, **kwargs)
        self.stop_event = stop
        self.command_lock = threading.RLock()
        self.worker_done = threading.Event()
        self.fault = None
        self.shutdown_error = None
        self.intent = None
        self.gripper_command = self.observe()['gripper']
        self.worker = threading.Thread(target=self._refresh, daemon=True)
        self.worker.start()

    def _read(self):
        # RGB-D rendering shares the simulator publishing loop. A transient
        # publication gap is not permission to use old feedback: wait briefly
        # for a genuinely fresh frame, without writing/renewing any target.
        deadline = time.monotonic() + .4
        first_age = None
        while True:
            try:
                feedback = super()._read()
                if first_age is not None and hasattr(self, '_log'):
                    self._emit('feedback_gap_recovered', first_age_ms=first_age,
                               recovered_age_ms=(time.time_ns()-feedback['timestamp_ns'])/1e6)
                return feedback
            except FeedbackAgeError as exc:
                if exc.age_ms < 0 or exc.age_ms > 500 or time.monotonic() >= deadline:
                    raise
                if first_age is None:
                    first_age = exc.age_ms
                    if hasattr(self, '_log'):
                        self._emit('feedback_gap_wait', age_ms=first_age, max_wait_s=.4,
                                   policy='no target write until a fresh frame arrives')
                time.sleep(.01)

    def _stop_and_confirm(self, timeout_s=3.0):
        # Contact-loaded simulation can take longer to settle after hold. Keep
        # the same feedback/stability criteria, only extend the observation window.
        return super()._stop_and_confirm(timeout_s=timeout_s)

    def _refresh(self):
        next_write = 0.
        try:
            while not self.worker_done.wait(.02):
                with self.command_lock:
                    if self.stop_event.is_set():
                        self.intent = None
                        self._stop_and_confirm()
                        return
                    self._read()
                    if not self.lease.alive:
                        raise RuntimeError('control lease lost')
                    if self.intent is not None and time.monotonic() >= next_write:
                        self._write(self.intent, valid_for_s=2.)
                        next_write = time.monotonic() + .5
        except Exception as exc:
            self.fault = str(exc)
            try:
                self._emit('background_fault', reason=self.fault)
            except Exception:
                pass  # A log failure must not bypass the stop attempt.
            if hasattr(self.stop_event, 'request'):
                self.stop_event.request('runtime_monitor_fault')
            else:
                self.stop_event.set()
            with self.command_lock:
                self.intent = None
                try:
                    self._stop_and_confirm()
                except Exception:
                    pass  # close() retries and reports unconfirmed stop.

    def _check(self):
        if self.fault:
            raise RuntimeError(self.fault)
        if self.stop_event.is_set():
            raise Stopped()

    def command(self, joint, *, gripper=None, timeout_s=5., settle_s=.15, close=False):
        q = _vector(joint, len(self.lower))
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise ValueError('joint target outside robot limits')
        g = self.gripper_command if gripper is None else float(gripper)
        if not np.isfinite(g) or not 0 <= g <= 1:
            raise ValueError('gripper command must be 0..1')
        with self.command_lock:
            self._check()
            before = self._read()
            self._resume_verified_hold()
            self.gripper_command = g
            self.intent = {'joint': q.tolist(), 'gripper': g}
            self._write(self.intent, valid_for_s=2.)
        start = time.monotonic(); stable = None
        last_stamp = before['timestamp_ns']
        while True:
            self._check()
            measured = self._read()
            if (measured['timestamp_ns'] - last_stamp) / 1e6 > self.max_age_ms:
                stable = None
            last_stamp = measured['timestamp_ns']
            error = float(np.max(np.abs(np.asarray(measured['joint']) - q)))
            elapsed = time.monotonic() - start
            if close and elapsed >= timeout_s:
                # Closing is a timed observation, not a position-arrival skill.
                # Contact can deflect arm joints as well as block finger travel.
                result = dict(gripper_observation_complete=True,
                    joint_arrived=error <= .005, joint_error_rad=error,
                    measured_joint=measured['joint'], requested_joint=q.tolist(),
                    measured_gripper=measured['gripper'], gripper_command=g,
                    grasp_detected='unknown', contact_truth_used=False)
                if error > .005:
                    result['warning'] = 'arm deviated from held joint target during closing; inspect current posture before moving'
                break
            # A close can be obstructed. Report measured width, never demand zero
            # width or infer a grasp; keep the close target through later motion.
            grip_arrived = close or abs(measured['gripper'] - g) <= .02 or gripper is None
            if measured['seq'] > before['seq'] and error <= .005 and grip_arrived:
                stable = time.monotonic() if stable is None else stable
                if time.monotonic() - stable >= settle_s and (not close or elapsed >= timeout_s):
                    result = dict(joint_arrived=True, joint_error_rad=error, measured_gripper=measured['gripper'],
                        gripper_command=g, grasp_detected='unknown', contact_truth_used=False)
                    break
            else:
                stable = None
            if elapsed >= timeout_s:
                result = dict(error='joint/gripper arrival deadline exceeded', joint_arrived=False,
                    measured_joint=measured['joint'], requested_joint=q.tolist(),
                    joint_tolerance_rad=.005, gripper_error=abs(measured['gripper']-g),
                    joint_error_rad=error, measured_gripper=measured['gripper'], gripper_command=g,
                    grasp_detected='unknown', contact_truth_used=False)
                with self.command_lock:
                    self._check()
                    self.intent = {'joint': measured['joint'], 'gripper': g}
                    self._write(self.intent, valid_for_s=2.)
                break
            self.stop_event.wait(.01)
        self._emit('tool_outcome', result=result)
        return result

    def close(self):
        if not hasattr(self, 'worker_done'):
            return super().close()
        self.worker_done.set()
        self.worker.join(timeout=2.)
        with self.command_lock:
            self.intent = None
            try:
                super().close()
            except Exception as exc:
                self.shutdown_error = str(exc)
                raise

    def __exit__(self, exc_type, exc, traceback):
        try:
            self.close()
        except Exception as cleanup:
            self.shutdown_error = str(cleanup)
            if exc is None:
                raise
            exc.add_note(f'Additionally, shutdown hold was not confirmed: {cleanup}')
        return False
