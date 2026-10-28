"""Read-only motion preview then single-use admission, inspired by RoboHarness V2.

Plans live in this process only. They never restore old commands after resume.
No collision/object pose claims are made; RGB anchors are optional 2D evidence.
"""
import copy
import time
import uuid
from collections import OrderedDict

import numpy as np

from .policy import _camera
from .vision import resolve_action


def observation_id(stamps):
    return 'obs:' + ':'.join(str(int(stamps[name])) for name in ('head', 'wrist'))


class MotionPlans:
    def __init__(self, capacity=4, ttl_s=120., clock=time.monotonic):
        self.capacity, self.ttl_s, self.clock = capacity, ttl_s, clock
        self.epoch = 0
        self._plans = OrderedDict()

    def _anchor(self, identifier, tracks, obs):
        point = tracks.get(identifier)
        if point['status'] != 'observed' or point['image_id'] != obs['image_ids'][point['camera']]:
            raise ValueError('anchor is lost or not observed in the current image; select a fresh track')
        return point

    def create(self, action, obs, cameras, kinematics, tracks):
        if action['observation_id'] != obs['observation_id']:
            raise ValueError('plan_motion observation_id does not match the current images')
        if kinematics is None:
            raise ValueError('motion planning requires Pinocchio')
        frame = action['frame']
        translation = np.asarray(action['delta_cm'], dtype=float) / 100.
        rotation = np.deg2rad(action.get('rot_deg', [0., 0., 0.]))
        low_frame = frame
        if frame in ('head_camera', 'wrist_camera'):
            name = frame.split('_')[0]
            calibration = _camera(cameras, name)
            if cameras[name]['timestamp_ns'] != obs['image_timestamps_ns'][name]:
                raise ValueError('camera calibration is not from the pictured frame')
            translation = calibration['rotation'] @ translation
            rotation = calibration['rotation'] @ rotation
            low_frame = 'base'
        low = {'action': 'move_eef_delta', 'delta_xyz': translation.tolist(),
               'delta_rotvec': rotation.tolist(), 'frame': low_frame, 'point': 'tool'}
        resolved, resolution = resolve_action(low, obs['joint_rad'], kinematics)
        anchor = self._anchor(action['anchor_track_id'], tracks, obs) if action.get('anchor_track_id') else None
        identifier = 'plan_' + uuid.uuid4().hex[:12]
        summary = {'plan_id': identifier, 'status': 'ready', 'observation_id': obs['observation_id'],
                   'requested_frame': frame, 'delta_cm': list(action['delta_cm']),
                   'rot_deg': list(action.get('rot_deg', [0., 0., 0.])),
                   'resolved_delta_base_m': resolution['resolved_delta_xyz_m'],
                   'rotation_base_rad': resolution['delta_rotvec_base_rad'],
                   'target_tool_pose': resolved['pose'], 'waypoint_count': len(resolution['waypoints']),
                   'anchor_track_id': anchor['track_id'] if anchor else None,
                   'validation': 'IK, joint bounds and per-waypoint joint delta; NOT collision clearance',
                   'expires_after_s': self.ttl_s, 'motion_written': False}
        self._plans[identifier] = {'summary': summary, 'joint': list(obs['joint_rad']), 'gripper': obs['gripper'],
                                   'epoch': self.epoch, 'created': self.clock(), 'resolution': copy.deepcopy(resolution),
                                   'anchor': anchor}
        while len(self._plans) > self.capacity:
            self._plans.popitem(last=False)
        return copy.deepcopy(summary)

    def admit(self, action, obs, measured, kinematics, tracks, *, latest_obs=None):
        if action['observation_id'] != obs['observation_id']:
            raise ValueError('execute_plan must reference the current observation_id')
        item = self._plans.get(action['plan_id'])
        if item is None or item['summary']['status'] != 'ready':
            raise ValueError('plan is unknown, consumed or invalidated; plan again')
        if item['epoch'] != self.epoch or self.clock() - item['created'] > self.ttl_s:
            raise ValueError('plan expired or robot command epoch changed; plan again')
        for joint, gripper in ((obs['joint_rad'], obs['gripper']), (measured['joint'], measured['gripper'])):
            if np.max(np.abs(np.asarray(joint) - item['joint'])) > .005 or abs(gripper - item['gripper']) > .02:
                raise ValueError('robot or gripper changed since planning; plan again')
            pe, re = kinematics.error(joint, kinematics.pose(item['joint'], 'tool'), 'tool')
            if pe > .001 or re > .01:
                raise ValueError('tool pose drifted since planning; plan again')
        if item['anchor']:
            anchor = self._anchor(item['anchor']['track_id'], tracks, latest_obs or obs)
            if np.linalg.norm(np.asarray(anchor['pixel']) - item['anchor']['pixel']) > 3.:
                raise ValueError('tracked patch moved more than 3 pixels since planning; plan again')
        result = copy.deepcopy(item['resolution'])
        result.update(plan_id=action['plan_id'], visual_action='execute_plan',
                      planned_frame=item['summary']['requested_frame'])
        # Single use even if the subsequent write is interrupted. No automatic retry.
        item['summary']['status'] = 'consumed'
        return {'joint': result['waypoints'][-1]['joint']}, result

    def invalidate_for_motion(self):
        self.epoch += 1
        for item in self._plans.values():
            if item['summary']['status'] == 'ready':
                item['summary']['status'] = 'invalidated'

    def summaries(self):
        result = []
        for item in self._plans.values():
            summary = copy.deepcopy(item['summary'])
            if summary['status'] == 'ready' and self.clock() - item['created'] > self.ttl_s:
                summary['status'] = 'expired'
            result.append(summary)
        return result
