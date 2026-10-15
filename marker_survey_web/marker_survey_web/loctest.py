"""The localization test screen's logic: the localizer's fixes as a trajectory, kidnap
events, and test points measured with a tape. No ROS.

A person walks with the camera (or a robot drives) while the localizer runs on
a surveyed map, with no prior: each fix stands on its own. When no marker is in
view the pose freezes; when one comes back into view it jumps to where the
camera now is. This module makes that visible:

- the trajectory: every published fix (x, y, z, yaw, its stamp);
- a kidnap event: two consecutive fixes more than `gap_s` apart in time and
  more than `jump_m` apart in position. The page draws the segment between
  them dashed;
- the time since the last fix (the localizer's own `last_fix_age` when its
  status has one, else the time since the web node last heard a fix);
- test points: the person stands on a spot whose x, y they measured with a
  tape from the origin marker's floor point and records it; the localizer's
  latest pose is stored beside it, and the error between the two.
"""

from collections import deque
from dataclasses import dataclass
import csv
import io
import math
import time
from typing import Optional


@dataclass
class LocTestSettings:
    gap_s: float = 1.0          # [s] fixes farther apart in time: a gap
    jump_m: float = 0.5         # [m] a gap followed by a jump larger than this: a kidnap
    keep: int = 3000            # fixes kept for the trajectory
    stale_s: float = 2.0        # [s] a test point with an older pose is marked stale


CSV_FIELDS = ['n', 'x_measured', 'y_measured', 'x', 'y', 'z', 'yaw_deg', 'error_m', 'dx_m', 'dy_m',
              'fix_age_s', 'markers', 'stamp']


class LocTest:
    def __init__(self, settings: LocTestSettings = None, clock=time.monotonic):
        self.s = settings or LocTestSettings()
        self.clock = clock
        self.reset()

    def reset(self, points: bool = True):
        self.track = deque(maxlen=self.s.keep)
        self.kidnaps = []
        self.last = None              # the latest fix
        self.last_t = None            # when it arrived (clock)
        self.images = 0
        if points:
            self.points = []
            self.next_n = 1

    # ------------------------------------------------------------ in
    def add(self, rec: dict):
        """One record of the localizer's ~/fixes (core/check.py fix_record)."""
        self.images += 1
        fix = rec.get('fix')
        if fix is None or not rec.get('published', True):
            return
        p = {'t': float(rec.get('stamp') or 0.0), 'x': float(fix['x']), 'y': float(fix['y']),
             'z': None if fix.get('z') is None else float(fix['z']),
             'yaw_deg': float(fix.get('yaw_deg') or 0.0), 'ids': list(fix.get('ids') or []),
             'kidnap': False}
        prev = self.track[-1] if self.track else None
        if prev is not None:
            dt = p['t'] - prev['t']
            jump = math.hypot(p['x'] - prev['x'], p['y'] - prev['y'])
            if dt > self.s.gap_s and jump > self.s.jump_m:
                p['kidnap'] = True
                self.kidnaps.append({'t': p['t'], 'gap_s': round(dt, 2), 'jump_m': round(jump, 3),
                                     'from': [round(prev['x'], 3), round(prev['y'], 3)],
                                     'to': [round(p['x'], 3), round(p['y'], 3)],
                                     'ids': p['ids']})
            elif dt > self.s.gap_s:
                p['gap'] = True
        self.track.append(p)
        self.last, self.last_t = p, self.clock()

    # ------------------------------------------------------------ out
    def fix_age(self, localizer_status: Optional[dict] = None) -> Optional[float]:
        age = (localizer_status or {}).get('last_fix_age')
        if age is not None:
            return float(age)
        return None if self.last_t is None else self.clock() - self.last_t

    def pose(self) -> Optional[dict]:
        if self.last is None:
            return None
        p = self.last
        out = {'x': round(p['x'], 3), 'y': round(p['y'], 3), 'yaw_deg': round(p['yaw_deg'], 1),
               'ids': p['ids'], 'stamp': p['t']}
        if p['z'] is not None:
            out['z'] = round(p['z'], 3)
        return out

    def add_point(self, x, y, localizer_status: Optional[dict] = None) -> dict:
        try:
            xm, ym = float(x), float(y)
        except (TypeError, ValueError):
            raise ValueError('give the measured x and y in metres') from None
        if not (math.isfinite(xm) and math.isfinite(ym)):
            raise ValueError('give the measured x and y in metres')
        if self.last is None:
            raise ValueError('no pose from the localizer yet: stand where a marker is in view')
        p = self.last
        age = self.fix_age(localizer_status)
        dx, dy = p['x'] - xm, p['y'] - ym
        pt = {'n': self.next_n, 'x_measured': round(xm, 3), 'y_measured': round(ym, 3),
              'x': round(p['x'], 3), 'y': round(p['y'], 3),
              'z': None if p['z'] is None else round(p['z'], 3), 'yaw_deg': round(p['yaw_deg'], 1),
              'error_m': round(math.hypot(dx, dy), 4), 'dx_m': round(dx, 4), 'dy_m': round(dy, 4),
              'fix_age_s': None if age is None else round(age, 2),
              'stale': age is not None and age > self.s.stale_s,
              'markers': ' '.join(str(i) for i in p['ids']), 'stamp': p['t']}
        self.points.append(pt)
        self.next_n += 1
        return pt

    def delete_point(self, n) -> bool:
        before = len(self.points)
        self.points = [p for p in self.points if p['n'] != int(n)]
        return len(self.points) < before

    def errors(self) -> dict:
        e = [p['error_m'] for p in self.points]
        if not e:
            return {'count': 0, 'mean_m': None, 'rms_m': None, 'max_m': None}
        return {'count': len(e), 'mean_m': round(sum(e) / len(e), 4),
                'rms_m': round(math.sqrt(sum(v * v for v in e) / len(e)), 4), 'max_m': round(max(e), 4)}

    def summary(self, localizer_status: Optional[dict] = None, track_points: int = 1500) -> dict:
        track = list(self.track)
        if len(track) > track_points:
            # keep every kidnap and gap end, thin the rest evenly
            step = int(math.ceil(len(track) / track_points))
            track = [p for i, p in enumerate(track)
                     if i % step == 0 or p['kidnap'] or p.get('gap') or i == len(track) - 1]
        age = self.fix_age(localizer_status)
        return {'pose': self.pose(), 'fix_age_s': None if age is None else round(age, 2),
                'fixes': len(self.track), 'images': self.images,
                'track': [[round(p['x'], 3), round(p['y'], 3), 1 if p['kidnap'] else 2 if p.get('gap') else 0]
                          for p in track],
                'kidnaps': list(self.kidnaps), 'points': list(self.points), 'errors': self.errors(),
                'settings': {'gap_s': self.s.gap_s, 'jump_m': self.s.jump_m, 'stale_s': self.s.stale_s}}

    def csv(self) -> str:
        buf = io.StringIO()
        w = csv.DictWriter(buf, fieldnames=CSV_FIELDS, extrasaction='ignore', lineterminator='\n')
        w.writeheader()
        for p in self.points:
            w.writerow({k: ('' if p.get(k) is None else p[k]) for k in CSV_FIELDS})
        return buf.getvalue()
