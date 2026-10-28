"""Bounded RGB point identity, inspired by RoboHarness's observation-only tracks.

This is a fresh 2D-only implementation, not its RGB-D tracker. No depth,
segmentation labels, object transforms or automatic motion are consumed.
"""
import copy
import uuid

import numpy as np


def image_id(camera, timestamp_ns):
    return f"{camera}:{int(timestamp_ns)}"


class RGBTracks:
    def __init__(self, capacity=6):
        self.capacity = capacity
        self._tracks = {}
        self._frames = {}

    @staticmethod
    def _cv():
        try:
            import cv2
            return cv2
        except ImportError as exc:
            raise ValueError("RGB tracking requires the optional harness-vision OpenCV dependency") from exc

    def update(self, cameras):
        for name in ('head', 'wrist'):
            camera = cameras.get(name, {})
            rgb, stamp = camera.get('rgb'), camera.get('timestamp_ns')
            if rgb is None or stamp is None:
                continue
            rgb = np.asarray(rgb)
            if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
                raise ValueError('tracking needs uint8 RGB')
            old = self._frames.get(name)
            if old and stamp == old['stamp']:
                continue
            active = [t for t in self._tracks.values() if t['camera'] == name and t['status'] == 'observed']
            gray = None
            if active:
                cv = self._cv()
                gray = cv.cvtColor(rgb, cv.COLOR_RGB2GRAY)
                for track in active:
                    if not old or stamp < old['stamp'] or stamp - old['stamp'] > 120_000_000_000 or old['rgb'].shape != rgb.shape:
                        self._lose(track, 'frame discontinuity')
                        continue
                    try:
                        self._advance(track, cv.cvtColor(old['rgb'], cv.COLOR_RGB2GRAY), gray)
                    except (ValueError, cv.error):
                        self._lose(track, 'optical flow rejected')
                    if track['status'] == 'observed':
                        track['image_id'] = image_id(name, stamp)
                        track['last_seen_timestamp_ns'] = int(stamp)
            self._frames[name] = {'rgb': rgb.copy(), 'stamp': int(stamp)}

    @staticmethod
    def _lose(track, reason):
        track.update(status='lost', pixel=None, delta_pixels=None, reason=reason, quality=0.)
        track.pop('_features', None)
        # No silent reacquisition: a repeated texture is not verified identity.

    def _advance(self, track, before, after):
        cv = self._cv()
        points = track['_features']
        new, ok, _ = cv.calcOpticalFlowPyrLK(before, after, points, None, winSize=(21, 21), maxLevel=2)
        if new is None:
            raise ValueError('flow unavailable')
        back, reverse_ok, _ = cv.calcOpticalFlowPyrLK(after, before, new, None, winSize=(21, 21), maxLevel=2)
        if back is None:
            raise ValueError('reverse flow unavailable')
        error = np.linalg.norm(back.reshape(-1, 2) - points.reshape(-1, 2), axis=1)
        valid = (ok.ravel() == 1) & (reverse_ok.ravel() == 1) & np.isfinite(error) & (error < .8)
        if valid.sum() < 4 or valid.mean() < .6:
            raise ValueError('insufficient consistent support')
        old_points, new_points = points.reshape(-1, 2)[valid], new.reshape(-1, 2)[valid]
        transform, inliers = cv.estimateAffinePartial2D(old_points, new_points, method=cv.RANSAC, ransacReprojThreshold=1.5)
        if transform is None or inliers is None or inliers.sum() < 4:
            raise ValueError('local geometry inconsistent')
        scale = float(np.linalg.norm(transform[:, 0]))
        pixel = transform @ np.r_[track['pixel'], 1.]
        height, width = after.shape
        if not .7 <= scale <= 1.4 or np.linalg.norm(pixel - track['pixel']) > 64 or not (10 <= pixel[0] < width-10 and 10 <= pixel[1] < height-10):
            raise ValueError('point left supported region')
        a = cv.getRectSubPix(before, (15, 15), tuple(map(float, track['pixel']))).astype(float)
        b = cv.getRectSubPix(after, (15, 15), tuple(map(float, pixel))).astype(float)
        a -= a.mean(); b -= b.mean()
        denom = np.linalg.norm(a) * np.linalg.norm(b)
        ncc = float((a*b).sum()/denom) if denom > 1e-6 else 0.
        if ncc < .6:
            raise ValueError('appearance changed or occluded')
        delta = pixel - track['pixel']
        track.update(pixel=pixel.tolist(), delta_pixels=delta.tolist(), quality=float(min(valid.mean(), ncc)),
                     reason='forward/backward flow and local appearance consistent')
        track['_features'] = new_points[inliers.ravel().astype(bool)].reshape(-1, 1, 2).astype(np.float32)

    def register(self, camera, expected_image_id, pixel, label):
        cv = self._cv()
        frame = self._frames.get(camera)
        if not frame or image_id(camera, frame['stamp']) != expected_image_id:
            raise ValueError('track registration image_id does not match the current captured image')
        if len(self._tracks) >= self.capacity:
            raise ValueError('track capacity reached; forget an old track first')
        gray = cv.cvtColor(frame['rgb'], cv.COLOR_RGB2GRAY)
        x, y = map(float, pixel); h, w = gray.shape
        if not (16 <= x < w-16 and 16 <= y < h-16):
            raise ValueError('track point needs a 16 pixel image margin')
        mask = np.zeros_like(gray); cv.circle(mask, (round(x), round(y)), 14, 255, -1)
        features = cv.goodFeaturesToTrack(gray, maxCorners=24, qualityLevel=.02, minDistance=3, mask=mask)
        patch = cv.getRectSubPix(gray, (15, 15), (x, y))
        if features is None or len(features) < 4 or float(patch.std()) < 8:
            raise ValueError('selected patch has insufficient texture; choose another visible rim patch')
        identifier = 'track_' + uuid.uuid4().hex[:10]
        track = {'track_id': identifier, 'camera': camera, 'label': label, 'pixel': [x, y],
                 'status': 'observed', 'image_id': expected_image_id, 'last_seen_timestamp_ns': frame['stamp'],
                 'delta_pixels': [0., 0.], 'quality': 1., 'reason': 'model-selected textured RGB patch',
                 'identity_verified': False, 'source': 'RGB optical flow; no depth or object truth', '_features': features}
        self._tracks[identifier] = track
        return self.get(identifier)

    def get(self, identifier):
        if identifier not in self._tracks:
            raise ValueError('unknown track_id; old-session tracks cannot be resumed')
        return copy.deepcopy({k: v for k, v in self._tracks[identifier].items() if not k.startswith('_')})

    def forget(self, identifier):
        self.get(identifier)
        del self._tracks[identifier]
        return {'forgotten': identifier}

    def summary(self):
        tracks = [self.get(t) for t in self._tracks]
        pairs = []
        for i, a in enumerate(tracks):
            for b in tracks[i+1:]:
                if a['status'] == b['status'] == 'observed' and a['camera'] == b['camera'] and a['image_id'] == b['image_id']:
                    delta = np.asarray(b['pixel']) - a['pixel']
                    pairs.append({'from': a['track_id'], 'to': b['track_id'], 'camera': a['camera'],
                                  'delta_pixels': delta.tolist(), 'distance_pixels': float(np.linalg.norm(delta))})
        return {'tracks': tracks, 'pairs': pairs, 'metric_depth_available': False,
                'meaning': 'observed 2D patches; no semantic identity, grasp or collision guarantee'}
