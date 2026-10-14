"""The survey's result per marker, its flags and verdict, and survey_report.yaml. No ROS.

Per marker: views, reprojection RMS, 1-sigma position (horizontal and
height) and angle, the tilt from its mount, flags, and a one-line verdict:

  good        nothing to do;
  pass again  usable, but seen too few times or too uncertain: drive past it
              again (from another side, or at an angle for a weak height);
  bad         do not use: its corners do not fit (moved, bent, misread), it
              is tilted far from its mount, or it lies below the floor.
"""

from dataclasses import dataclass, field
import datetime
import math
from typing import Dict, List, Optional

import numpy as np
import yaml

from marker_survey.core import marker_map as mm

GOOD, PASS_AGAIN, BAD = 'good', 'pass again', 'bad'


@dataclass
class ReportSettings:
    min_views: int = 3
    max_sd_xy: float = 0.03            # [m] large uncertainty: pass again
    max_sd_z: float = 0.02             # [m] weak height of a ceiling marker: pass it at an angle
    max_sd_angle_deg: float = 1.0
    max_tilt_deg: float = 10.0         # bad
    bad_reproj_px: float = 2.0         # bad
    bad_sd_xy: float = 0.10            # bad


@dataclass
class SurveyedMarker:
    id: int
    T: np.ndarray                      # T_target_marker
    mount: str
    size: float
    views: int
    reproj_rms_px: float
    tilt_deg: float
    height_above_floor: float
    sd_xy: Optional[float] = None      # [m] 1-sigma, worst horizontal direction
    sd_z: Optional[float] = None       # [m] 1-sigma height
    sd_angle_deg: Optional[float] = None   # 1-sigma, worst rotation axis
    flags: List[str] = field(default_factory=list)
    verdict: str = GOOD
    hint: str = ''

    @property
    def x(self):
        return float(self.T[0, 3])

    @property
    def y(self):
        return float(self.T[1, 3])

    @property
    def z(self):
        return float(self.T[2, 3])

    def angles(self) -> Dict[str, float]:
        return mm.angles_from_rotation(self.mount, self.T[:3, :3])


def surveyed_markers(markers_world: Dict[int, np.ndarray], stats: dict, T_target_world: np.ndarray,
                     sizes: Dict[int, float], floor_point, floor_normal,
                     settings: ReportSettings = None) -> List[SurveyedMarker]:
    """Express the solved markers in the target frame, with uncertainty, flags, verdicts."""
    st_ = settings or ReportSettings()
    R = T_target_world[:3, :3]
    fp = T_target_world[:3, :3] @ np.asarray(floor_point) + T_target_world[:3, 3]
    fn = R @ np.asarray(floor_normal)
    out = []
    for mid, Tw in sorted(markers_world.items()):
        T = T_target_world @ Tw
        mount = mm.mount_from_normal(T[:3, 2])
        st = stats.get(mid, {})
        sm = SurveyedMarker(mid, T, mount, float(sizes[mid]), int(st.get('views', 0)),
                            float(st.get('reproj_rms_px', float('nan'))),
                            mm.tilt_from_mount(mount, T[:3, :3]),
                            float((T[:3, 3] - fp) @ fn))
        C = st.get('cov_target')
        if C is None and st.get('cov_world') is not None:
            Cw = st['cov_world']
            C = np.zeros((6, 6))
            C[:3, :3] = Cw[:3, :3]
            C[3:, 3:] = R @ Cw[3:, 3:] @ R.T
        if C is not None:
            Ct = C[3:, 3:]
            sm.sd_xy = float(math.sqrt(max(np.linalg.eigvalsh(Ct[:2, :2]).max(), 0.0)))
            sm.sd_z = float(math.sqrt(max(Ct[2, 2], 0.0)))
            Cr = C[:3, :3]
            sm.sd_angle_deg = float(math.degrees(math.sqrt(max(np.linalg.eigvalsh(Cr).max(), 0.0))))
        judge(sm, st_)
        out.append(sm)
    return out


def judge(sm: SurveyedMarker, s: ReportSettings):
    """Flags and verdict of one marker."""
    bad, again, hints = [], [], []
    if not math.isnan(sm.reproj_rms_px) and sm.reproj_rms_px > s.bad_reproj_px:
        bad.append(f'corners fit badly ({sm.reproj_rms_px:.1f} px)')
        hints.append('check that it is flat and did not move during the survey')
    if sm.tilt_deg > s.max_tilt_deg:
        what = 'from vertical' if sm.mount == 'wall' else 'from level'
        bad.append(f'tilted {sm.tilt_deg:.0f} deg {what}')
        hints.append('mount it straight')
    if sm.mount == 'wall':
        low = sm.size / 2.0
        if sm.height_above_floor < 0:
            bad.append(f'centre {sm.height_above_floor:.2f} m is BELOW the floor')
            hints.append('check floor_offset (vehicle on a stand?) and the camera mount TF')
        elif sm.height_above_floor < low:
            again.append(f'centre {sm.height_above_floor:.2f} m above the floor, below half '
                         f'the marker size ({low:.2f} m)')
            hints.append('check floor_offset (vehicle on a stand?) and the camera mount TF')
    if sm.sd_xy is not None and sm.sd_xy > s.bad_sd_xy:
        bad.append(f'position sd {sm.sd_xy * 100:.1f} cm')
    if sm.views < s.min_views:
        again.append(f'only {sm.views} view(s)')
        hints.append('pass it again from another place')
    if sm.sd_xy is not None and s.max_sd_xy < sm.sd_xy <= s.bad_sd_xy:
        again.append(f'position sd {sm.sd_xy * 100:.1f} cm')
        hints.append('pass it again, closer')
    if sm.sd_angle_deg is not None and sm.sd_angle_deg > s.max_sd_angle_deg:
        again.append(f'angle sd {sm.sd_angle_deg:.1f} deg')
        hints.append('pass it again from another side')
    if sm.sd_z is not None and sm.mount == 'ceiling' and sm.sd_z > s.max_sd_z:
        again.append(f'weak height: sd {sm.sd_z * 100:.1f} cm')
        hints.append('pass it at an angle, not only straight from below')
    elif sm.sd_z is not None and sm.mount != 'ceiling' and sm.sd_z > s.max_sd_xy:
        again.append(f'height sd {sm.sd_z * 100:.1f} cm')
        hints.append('pass it again, closer')
    sm.flags = bad + again
    sm.verdict = BAD if bad else PASS_AGAIN if again else GOOD
    seen = []
    for h in hints:
        if h not in seen:
            seen.append(h)
    sm.hint = '; '.join(seen)


def build_map(surveyed: List[SurveyedMarker], dictionary: str, marker_size: float, frame_id: str,
              header: List[str], previous: Optional[mm.MarkerMap] = None) -> mm.MarkerMap:
    """The marker map of a survey; comments of a previous map are kept for the same ids."""
    markers = []
    for s in surveyed:
        m = mm.MapMarker.from_pose(s.id, s.T, mount=s.mount,
                                   size=None if abs(s.size - marker_size) < 1e-9 else s.size,
                                   decimals=(3, 2))
        markers.append(m)
    out = mm.MarkerMap(dictionary=dictionary, marker_size=float(marker_size), markers=markers,
                       frame_id=frame_id, mount=mm.default_mount(markers), header=list(header),
                       key_comments=dict(mm.DEFAULT_KEY_COMMENTS))
    if previous is not None:
        old = previous.by_id()
        for m in markers:
            if m.id in old:
                m.comment = old[m.id].comment
                m.comment_above = list(old[m.id].comment_above)
        kept = [h for h in previous.header if h is None or not str(h).startswith('Surveyed ')]
        while kept and kept[0] is None:
            kept.pop(0)
        out.header = list(header) + kept
        out.key_comments.update(previous.key_comments)
        out.key_above = dict(previous.key_above)
        out.footer = list(previous.footer)
    if out.mount is None:
        out.key_comments.pop('mount', None)
    return out


def _r(v, n):
    return None if v is None or (isinstance(v, float) and math.isnan(v)) else round(float(v), n)


def report_dict(surveyed: List[SurveyedMarker], info: dict, frame: dict, dictionary: str,
                marker_size: float, start_pose: Optional[dict]) -> dict:
    stamp = datetime.datetime.now().astimezone().isoformat(timespec='seconds')
    counts = {GOOD: 0, PASS_AGAIN: 0, BAD: 0}
    for s in surveyed:
        counts[s.verdict] += 1
    solve = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in info.items()}
    return {
        'date': stamp, 'dictionary': dictionary, 'marker_size': float(marker_size),
        'frame': frame, 'solve': solve, 'recording_start_pose': start_pose,
        'summary': counts,
        'markers': [{
            'id': s.id, 'verdict': s.verdict, 'hint': s.hint, 'mount': s.mount,
            'x': _r(s.x, 4), 'y': _r(s.y, 4), 'z': _r(s.z, 4),
            **{k: _r(v, 2) for k, v in s.angles().items()},
            'tilt_deg': _r(s.tilt_deg, 2), 'views': s.views,
            'reproj_rms_px': _r(s.reproj_rms_px, 3),
            'sd_xy_cm': None if s.sd_xy is None else _r(s.sd_xy * 100, 2),
            'sd_z_cm': None if s.sd_z is None else _r(s.sd_z * 100, 2),
            'sd_angle_deg': _r(s.sd_angle_deg, 3),
            'flags': s.flags} for s in surveyed],
    }


def write_report(path: str, report: dict) -> None:
    with open(path, 'w') as f:
        f.write('# Written by every solve of the survey; marker_map.yaml has the poses.\n')
        yaml.safe_dump(report, f, sort_keys=False, default_flow_style=None, width=100)


def format_table(markers: List[SurveyedMarker]) -> str:
    rows = ['  id  mount      x [m]    y [m]    z [m]  tilt  views  reproj   sd_xy   sd_z  verdict']
    for m in markers:
        sdxy = '    -  ' if m.sd_xy is None else f'{m.sd_xy * 100:5.1f}cm'
        sdz = '    -  ' if m.sd_z is None else f'{m.sd_z * 100:5.1f}cm'
        rows.append(f'  {m.id:3d} {m.mount:7s} {m.x:8.3f} {m.y:8.3f} {m.z:8.3f} {m.tilt_deg:4.1f}d'
                    f' {m.views:6d} {m.reproj_rms_px:6.2f}px {sdxy} {sdz}  {m.verdict}'
                    + (f': {"; ".join(m.flags)}' if m.flags else ''))
    return '\n'.join(rows)


def start_pose_in(T_target_world: np.ndarray, T_world_base0: np.ndarray) -> dict:
    T = T_target_world @ T_world_base0
    return {'x': round(float(T[0, 3]), 3), 'y': round(float(T[1, 3]), 3),
            'yaw_deg': round(math.degrees(math.atan2(T[1, 0], T[0, 0])), 2)}
