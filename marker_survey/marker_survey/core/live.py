"""What a running survey shows while the robot drives, as plain data. No ROS.

The survey node publishes this as JSON on ~/status for the web interface;
the tests build it from a simulated survey the same way.

Per marker a light, before the solve from what the keyframes say, after
it from the report:

  good        seen often enough, from places far enough apart (provisional
              until the solve);
  pass again  seen too few times, or from one place only;
  bad         the report's verdict (only after a solve).

and a hint in plain words for every marker that is not good yet.
"""

import datetime
import math
import os
import shutil
from typing import Dict, List, Optional

import numpy as np

from marker_survey.core.report import BAD, GOOD, PASS_AGAIN
from marker_survey.core.survey import Survey

MIN_VIEWS = 3            # keyframes; the report's own min_views
MIN_SPREAD = 0.3         # [m] between the camera positions that saw a marker
TRAIL_POINTS = 400       # keyframe positions sent for the top-down map
HISTORY_DIR = 'history'


def yaw_of(T) -> float:
    return math.atan2(T[1, 0], T[0, 0])


def detection_states(survey: Survey, detections, known) -> List[dict]:
    """The markers of one frame: known (seen before), new (first time) or not used
    (too small, cut by the image border)."""
    out = []
    for d in detections:
        if not survey.usable(d):
            state, why = 'unusable', d.rejected or 'small'
        elif d.id in known:
            state, why = 'known', ''
        else:
            state, why = 'new', ''
        out.append({'id': int(d.id), 'state': state, 'why': why,
                    'px': round(d.min_edge(), 1)})
    return out


def marker_places(survey: Survey) -> Dict[int, dict]:
    """Per marker: keyframes that saw it and how far apart the camera was between them."""
    pos: Dict[int, list] = {}
    for kf in survey.keyframes:
        p = kf.T_world_cam[:3, 3]
        for d in kf.detections:
            pos.setdefault(d.id, []).append(p)
    out = {}
    for mid, ps in pos.items():
        a = np.array(ps)
        spread = float(np.linalg.norm(a.max(axis=0) - a.min(axis=0))) if len(a) > 1 else 0.0
        out[mid] = {'views': len(ps), 'spread': spread}
    return out


def live_light(views: int, spread: float):
    """(light, hint) of a marker before the solve."""
    if views < MIN_VIEWS:
        return PASS_AGAIN, f'seen {views} time{"s" if views != 1 else ""} only: drive past it again'
    if spread < MIN_SPREAD:
        return PASS_AGAIN, 'seen from one place only: pass it again from another side'
    return GOOD, ''


def status_dict(survey: Optional[Survey], state: str, frame: dict, report: Optional[dict] = None,
                report_views: Optional[Dict[int, int]] = None, detections: Optional[list] = None,
                T_world_base: Optional[np.ndarray] = None, extra: Optional[dict] = None) -> dict:
    """The survey as the web page shows it.

    report: the last solve's report (survey_report.yaml as a dict); its verdict is shown
    for a marker until the marker gets new views (report_views: views at that solve)."""
    markers, trail = [], []
    counts = {GOOD: 0, PASS_AGAIN: 0, BAD: 0}
    hints = []
    if survey is not None:
        places = marker_places(survey)
        solved = {m['id']: m for m in (report or {}).get('markers', [])}
        for mid, m in sorted(survey.markers.items()):
            pl = places.get(mid, {'views': m.views, 'spread': 0.0})
            light, hint = live_light(pl['views'], pl['spread'])
            source = 'live'
            r = solved.get(mid)
            if r is not None and (report_views or {}).get(mid) == pl['views']:
                light, hint, source = r['verdict'], r.get('hint') or '', 'report'
                if light != GOOD and not hint:
                    hint = '; '.join(r.get('flags') or [])
            T = m.T_world_marker
            markers.append({'id': int(mid), 'x': round(float(T[0, 3]), 3), 'y': round(float(T[1, 3]), 3),
                            'z': round(float(T[2, 3]), 3), 'yaw_deg': round(math.degrees(yaw_of(T)), 1),
                            'normal_z': round(float(T[2, 2]), 2),
                            'size': survey.s.size_of(mid), 'views': pl['views'],
                            'spread_m': round(pl['spread'], 2), 'light': light, 'source': source,
                            'hint': hint})
            counts[light] = counts.get(light, 0) + 1
            if hint:
                hints.append(f'marker {mid}: {hint}')
        kfs = survey.keyframes
        step = max(1, int(math.ceil(len(kfs) / TRAIL_POINTS)))
        base = survey.T_cam_base
        for kf in kfs[::step]:
            T = kf.T_world_cam @ base if base is not None else kf.T_world_cam
            trail.append([round(float(T[0, 3]), 3), round(float(T[1, 3]), 3)])
    robot = None
    if T_world_base is not None:
        robot = {'x': round(float(T_world_base[0, 3]), 3), 'y': round(float(T_world_base[1, 3]), 3),
                 'yaw_deg': round(math.degrees(yaw_of(T_world_base)), 1)}
    out = {'state': state, 'frame': frame,
           'keyframes': len(survey.keyframes) if survey is not None else 0,
           'markers': markers, 'lights': counts, 'hints': hints, 'trail': trail, 'robot': robot,
           'detections': detections or []}
    out.update(extra or {})
    return out


def keep_previous(output_dir: str, files, stamp: Optional[str] = None) -> Optional[str]:
    """Copy the files of the previous solve into output_dir/history/<date-time>/ before a
    new solve overwrites them; returns that directory, or None if there was nothing."""
    present = [f for f in files if os.path.isfile(os.path.join(output_dir, f))]
    if not present:
        return None
    stamp = stamp or datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    dest = os.path.join(output_dir, HISTORY_DIR, stamp)
    n = 1
    while os.path.exists(dest):
        n += 1
        dest = os.path.join(output_dir, HISTORY_DIR, f'{stamp}-{n}')
    os.makedirs(dest)
    for f in present:
        shutil.copy2(os.path.join(output_dir, f), os.path.join(dest, f))
    return dest


def history(output_dir: str) -> List[dict]:
    """The kept solves, newest first."""
    root = os.path.join(output_dir, HISTORY_DIR)
    if not os.path.isdir(root):
        return []
    out = []
    for name in sorted(os.listdir(root), reverse=True):
        d = os.path.join(root, name)
        if os.path.isdir(d):
            out.append({'name': name, 'files': sorted(os.listdir(d))})
    return out
