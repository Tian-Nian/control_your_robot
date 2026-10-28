"""Exercise the real cyr CLI against an isolated optional physics runtime."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest
pytest.importorskip('mujoco')


def test_call_joint_eef_reset_gripper_and_rejected_ik():
    root = Path(__file__).resolve().parents[4]
    spec = importlib.util.spec_from_file_location('cli_scene_helper', root / 'examples/piper_mujoco_harness.py')
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    with helper.scene_session(SimpleNamespace(attach=False, viewer=False, no_cameras=True)) as session:
        def call(data, delta=False, accepted=True):
            process = subprocess.run([sys.executable, '-m', 'robot.runtime', 'call', 'left_arm',
                                      json.dumps(data), *(['--delta'] if delta else [])],
                                     cwd=root, capture_output=True, text=True, timeout=25)
            reply = json.loads(process.stdout)
            assert reply['accepted'] == accepted, reply
            assert process.returncode == (0 if accepted else 1), process.stderr
            return reply

        def state():
            return session.read_raw('left_arm')['data']

        time.sleep(.3)
        home = [0, 1.3, -1, 0, .5, 0]
        goal = [.12, 1.3, -1, 0, .5, 0]  # Larger than the stream gate.
        call({'joint': goal, 'gripper': 0})
        np.testing.assert_allclose(state()['joint'], goal, atol=.006)
        assert state()['gripper_command'] == 0
        assert state()['gripper_preload_n'] == 2
        call({'joint': [-.06, 0, 0, 0, 0, 0]}, delta=True)
        assert abs(state()['joint'][0] - .06) < .007
        call({'op': 'reset'})
        np.testing.assert_allclose(state()['joint'], home, atol=.006)
        assert state()['gripper_command'] == 0  # Arm reset preserves closing force.
        call({'eef': [.001, 0, 0]}, delta=True)
        target = list(state()['eef'])
        target[2] += .002
        call({'eef': target, 'gripper': 1})
        np.testing.assert_allclose(state()['eef'][:3], target[:3], atol=.002)
        assert state()['gripper_command'] == 1
        assert state()['gripper_preload_n'] == 0
        call({'op': 'reset', 'start_state': goal})
        np.testing.assert_allclose(state()['joint'], goal, atol=.006)
        call({'eef': [10, 10, 10, 1, 0, 0, 0]}, accepted=False)
        call({'op': 'release'})
        call({'op': 'reset', 'start_state': [0]}, accepted=False)
        call({'op': 'release'})
        call({'op': 'reset'})  # Rejection must not kill the component.
        np.testing.assert_allclose(state()['joint'], home, atol=.006)
        assert state()['sim_time'] > 1
        assert all(abs(item['pose'][2]) < .005 for item in state()['objects'].values())
