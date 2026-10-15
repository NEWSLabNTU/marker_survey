"""Checking a marker map while driving: each fix against the robot's fused pose. No ROS.

The localizer node publishes one record per image with markers (`fix_record`,
JSON on ~/fixes); the web interface's check screen sums them up
(`CheckStats`):

- the fix against the state estimator's pose at the image's stamp (the
  prior), and the distance between them;
- per marker, how often it agreed: it was in a fix that lay within
  `agree_distance` and `agree_angle_deg` of the fused pose. It disagreed when
  it was left out of a fix as an outlier, or when the fix it was in lay
  farther away (gated, or merely off). A marker that disagrees in more than
  `max_disagree` of at least `min_samples` comparisons is flagged: it moved,
  or it was surveyed badly.

Without a prior there is nothing to compare with; the fixes are still counted.
"""

from collections import deque
from dataclasses import dataclass
import math
from typing import Dict, Optional

import numpy as np

from marker_survey.core.se3 import rotation_angle

from marker_localizer.core.localize import Result, rms, wrap, yaw_of


def _pose(T, planar: bool) -> dict:
    out = {'x': round(float(T[0, 3]), 4), 'y': round(float(T[1, 3]), 4),
           'yaw_deg': round(math.degrees(yaw_of(T[:3, :3])), 2)}
    if not planar:
        out['z'] = round(float(T[2, 3]), 4)
    return out


def fix_record(res: Result, detections, T_prior: Optional[np.ndarray], published: bool, stamp: float,
               mode: str) -> dict:
    """One image's outcome as plain data: the fix (or why none), the prior, and every
    marker in view with what became of it."""
    planar = mode == 'planar'
    fix = res.fix
    rec = {'stamp': round(float(stamp), 4), 'published': bool(published), 'reason': res.reason,
           'fix': None, 'prior': None, 'distance_m': None, 'angle_deg': None, 'markers': {}}
    if T_prior is not None:
        rec['prior'] = _pose(T_prior, planar)
    if fix is not None:
        sd = math.sqrt(max(fix.covariance[0, 0], fix.covariance[1, 1]))
        rec['fix'] = dict(_pose(fix.T_map_base, planar), ids=[int(i) for i in fix.ids],
                          sd_cm=round(sd * 100, 2), rms_px=round(float(fix.rms_px), 3))
        if T_prior is not None:
            d = fix.T_map_base[:3, 3] - T_prior[:3, 3]
            if planar:
                rec['distance_m'] = round(float(np.hypot(d[0], d[1])), 4)
                rec['angle_deg'] = round(abs(math.degrees(wrap(fix.yaw - yaw_of(T_prior[:3, :3])))), 3)
            else:
                rec['distance_m'] = round(float(np.linalg.norm(d)), 4)
                rec['angle_deg'] = round(math.degrees(rotation_angle(fix.T_map_base[:3, :3],
                                                                     T_prior[:3, :3])), 3)
    used = set(fix.ids) if fix is not None else set()
    for d in detections:
        mid = int(d.id)
        if mid in used:
            m = {'state': 'used', 'residual_px': round(rms(fix.residuals_px[mid]), 3)}
        elif mid in res.dropped:
            why = res.dropped[mid]
            m = {'state': 'outlier' if why.startswith('outlier') else 'unusable', 'why': why}
        elif mid in res.unknown:
            m = {'state': 'unknown'}
        else:
            m = {'state': 'unused'}
        rec['markers'][str(mid)] = m
    return rec


@dataclass
class CheckSettings:
    agree_distance: float = 0.15     # [m] a fix this close to the fused pose agrees
    agree_angle_deg: float = 5.0
    min_samples: int = 5             # comparisons before a marker can be flagged
    max_disagree: float = 0.2        # flagged above this share of disagreements
    keep: int = 300                  # recent fixes kept for the map


class CheckStats:
    """The check screen's numbers, summed over the records of a drive."""

    def __init__(self, settings: CheckSettings = None):
        self.s = settings or CheckSettings()
        self.reset()

    def reset(self):
        self.images = 0
        self.fixes = 0
        self.published = 0
        self.with_prior = 0
        self.agreed_fixes = 0
        self.distances = []
        self.recent = deque(maxlen=self.s.keep)
        self.markers: Dict[int, dict] = {}
        self.last = None

    def _m(self, mid: int) -> dict:
        return self.markers.setdefault(mid, {'id': mid, 'seen': 0, 'used': 0, 'outlier': 0,
                                             'compared': 0, 'agreed': 0, 'dist_sum': 0.0})

    def agrees(self, rec: dict) -> Optional[bool]:
        if rec.get('fix') is None or rec.get('distance_m') is None:
            return None
        return (rec['distance_m'] <= self.s.agree_distance
                and (rec.get('angle_deg') or 0.0) <= self.s.agree_angle_deg)

    def add(self, rec: dict):
        self.images += 1
        self.last = rec
        ok = self.agrees(rec)
        if rec.get('fix') is not None:
            self.fixes += 1
            self.published += bool(rec.get('published'))
            p = rec.get('prior')
            self.recent.append({'fix': [rec['fix']['x'], rec['fix']['y']],
                                'prior': None if p is None else [p['x'], p['y']],
                                'agrees': ok, 'published': bool(rec.get('published'))})
        if ok is not None:
            self.with_prior += 1
            self.agreed_fixes += bool(ok)
            self.distances.append(rec['distance_m'])
        for key, m in (rec.get('markers') or {}).items():
            st = m.get('state')
            if st in ('unknown', 'unusable'):
                continue
            c = self._m(int(key))
            c['seen'] += 1
            if st == 'used':
                c['used'] += 1
                if ok is not None:
                    c['compared'] += 1
                    c['agreed'] += bool(ok)
                    c['dist_sum'] += rec['distance_m']
            elif st == 'outlier':
                c['outlier'] += 1
                if rec.get('prior') is not None:
                    c['compared'] += 1          # left out while others fitted: a disagreement

    def table(self) -> list:
        out = []
        for mid, c in sorted(self.markers.items()):
            share = c['agreed'] / c['compared'] if c['compared'] else None
            used_cmp = c['compared'] - c['outlier'] if c['compared'] else 0
            flagged = (c['compared'] >= self.s.min_samples and share is not None
                       and 1.0 - share > self.s.max_disagree)
            out.append({'id': mid, 'seen': c['seen'], 'used': c['used'], 'outlier': c['outlier'],
                        'compared': c['compared'], 'agreed': c['agreed'],
                        'agreement': None if share is None else round(share, 3),
                        'mean_distance_m': (round(c['dist_sum'] / used_cmp, 4) if used_cmp > 0
                                            else None),
                        'flagged': bool(flagged)})
        return out

    def summary(self) -> dict:
        d = np.array(self.distances) if self.distances else None
        return {
            'images': self.images, 'fixes': self.fixes, 'published': self.published,
            'with_prior': self.with_prior, 'agreed': self.agreed_fixes,
            'distance_mean_m': None if d is None else round(float(d.mean()), 4),
            'distance_median_m': None if d is None else round(float(np.median(d)), 4),
            'distance_max_m': None if d is None else round(float(d.max()), 4),
            'last': self.last, 'markers': self.table(),
            'flagged': [r['id'] for r in self.table() if r['flagged']],
            'recent': list(self.recent),
            'settings': {'agree_distance': self.s.agree_distance,
                         'agree_angle_deg': self.s.agree_angle_deg,
                         'min_samples': self.s.min_samples, 'max_disagree': self.s.max_disagree},
        }
