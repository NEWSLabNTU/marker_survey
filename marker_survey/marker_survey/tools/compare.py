"""marker_survey_compare: a surveyed marker map against the truth (or a tape survey).

    marker_survey_compare out/marker_map.yaml truth.yaml [--max-median-cm 3 --max-cm 5]

Both maps must be in the same frame. Prints per marker the position error
(dx, dy, dz), the rotation error and the tilt error (angle between the
normals), then the summary; exits 1 when a threshold is exceeded or a marker
of the truth is missing.

With the survey's report (survey_report.yaml beside the result, or --report),
also each marker's verdict and its error in units of the survey's own 1-sigma
in the result frame (frame_sd_xy_cm, frame_sd_z_cm): horizontal / sd_xy and
|dz| / sd_z. Values around 1 mean the reported uncertainty is honest; well
over 2 to 3, the survey is overconfident. --require-good fails unless every
marker's verdict is good.
"""
import argparse
import math
import os
import sys

import numpy as np

from marker_survey.core import marker_map as mm
from marker_survey.core.se3 import rotation_angle


def compare(got: mm.MarkerMap, want: mm.MarkerMap):
    """Per-marker errors {id: dict} and the ids missing from `got`."""
    g = got.by_id()
    rows, missing = {}, []
    for w in want.markers:
        if w.id not in g:
            missing.append(w.id)
            continue
        a, b = g[w.id].pose(), w.pose()
        d = a[:3, 3] - b[:3, 3]
        cosn = float(np.clip(a[:3, 2] @ b[:3, 2], -1.0, 1.0))
        rows[w.id] = {'dx': d[0], 'dy': d[1], 'dz': d[2], 'pos': float(np.linalg.norm(d)),
                      'horiz': float(np.linalg.norm(d[:2])),
                      'rot_deg': math.degrees(rotation_angle(a[:3, :3], b[:3, :3])),
                      'normal_deg': math.degrees(math.acos(cosn))}
    return rows, missing


def load_report(path):
    """{id: report entry} from a survey_report.yaml, or {} if there is none."""
    if not path or not os.path.isfile(path):
        return {}
    import yaml
    with open(path) as f:
        rep = yaml.safe_load(f) or {}
    return {int(m['id']): m for m in rep.get('markers', [])}


def sigma_ratios(rows: dict, report: dict) -> None:
    """Adds 'h_sd' (horizontal error / frame_sd_xy) and 'z_sd' (|dz| / frame_sd_z) to
    each row with a report entry."""
    for mid, r in rows.items():
        m = report.get(mid) or {}
        sxy, sz = m.get('frame_sd_xy_cm'), m.get('frame_sd_z_cm')
        r['h_sd'] = r['horiz'] * 100 / sxy if sxy else None
        r['z_sd'] = abs(r['dz']) * 100 / sz if sz else None
        r['verdict'] = m.get('verdict')


def summary(rows: dict) -> dict:
    if not rows:
        return {}
    col = {k: np.array([r[k] for r in rows.values()]) for k in ('pos', 'horiz', 'dz', 'rot_deg',
                                                                'normal_deg')}
    return {'markers': len(rows),
            'pos_median_cm': float(np.median(col['pos']) * 100), 'pos_max_cm': float(col['pos'].max() * 100),
            'horiz_median_cm': float(np.median(col['horiz']) * 100),
            'horiz_max_cm': float(col['horiz'].max() * 100),
            'dz_max_cm': float(np.abs(col['dz']).max() * 100),
            'rot_max_deg': float(col['rot_deg'].max()), 'normal_max_deg': float(col['normal_deg'].max()),
            **ratio_summary(rows)}


def ratio_summary(rows: dict) -> dict:
    out = {}
    for k in ('h_sd', 'z_sd'):
        v = [r[k] for r in rows.values() if r.get(k) is not None]
        if v:
            out[f'{k}_median'], out[f'{k}_max'] = float(np.median(v)), float(np.max(v))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('result', help='marker_map.yaml written by the survey')
    ap.add_argument('truth', help='marker_map.yaml of the truth, same frame')
    ap.add_argument('--max-median-cm', type=float, help='fail above this median 3D position error')
    ap.add_argument('--max-cm', type=float, help='fail above this largest 3D position error')
    ap.add_argument('--max-dz-cm', type=float, help='fail above this largest height error')
    ap.add_argument('--max-normal-deg', type=float, help='fail above this largest tilt error')
    ap.add_argument('--allow-missing', action='store_true', help='a missing marker is not a failure')
    ap.add_argument('--report', help='the survey\'s survey_report.yaml (default: the one beside RESULT)')
    ap.add_argument('--require-good', action='store_true',
                    help='fail unless the report gives every marker the verdict good')
    args = ap.parse_args(argv)
    got, want = mm.load(args.result), mm.load(args.truth)
    report = load_report(args.report or os.path.join(os.path.dirname(os.path.abspath(args.result)),
                                                     'survey_report.yaml'))
    if got.frame_id != want.frame_id:
        print(f'WARNING: frames differ: {got.frame_id} vs {want.frame_id}')
    rows, missing = compare(got, want)
    sigma_ratios(rows, report)
    print('  id   dx [cm]  dy [cm]  dz [cm]  |d| [cm]  rot [deg]  normal [deg]'
          + ('  h/sd_xy  dz/sd_z  verdict' if report else ''))

    def ratio(v):
        return f'{v:8.2f}' if v is not None else '       -'

    for mid, r in sorted(rows.items()):
        print(f'  {mid:3d} {100 * r["dx"]:8.2f} {100 * r["dy"]:8.2f} {100 * r["dz"]:8.2f} '
              f'{100 * r["pos"]:9.2f} {r["rot_deg"]:10.2f} {r["normal_deg"]:13.2f}'
              + (f' {ratio(r["h_sd"])} {ratio(r["z_sd"])}  {r["verdict"] or "-"}' if report else ''))
    for mid in missing:
        print(f'  {mid:3d}   MISSING from the survey')
    s = summary(rows)
    ok = True
    if s:
        print(f'position: median {s["pos_median_cm"]:.2f} cm, max {s["pos_max_cm"]:.2f} cm '
              f'(horizontal: median {s["horiz_median_cm"]:.2f} cm, max {s["horiz_max_cm"]:.2f} cm); '
              f'height: max {s["dz_max_cm"]:.2f} cm; rotation: max {s["rot_max_deg"]:.2f} deg; '
              f'tilt: max {s["normal_max_deg"]:.2f} deg')
        if 'h_sd_median' in s:
            print(f'error / frame 1-sigma: horizontal median {s["h_sd_median"]:.2f}, max '
                  f'{s["h_sd_max"]:.2f}'
                  + (f'; height median {s["z_sd_median"]:.2f}, max {s["z_sd_max"]:.2f}'
                     if 'z_sd_median' in s else ''))
    else:
        ok = False
        print('no marker in common')
    checks = [('median position', args.max_median_cm, s.get('pos_median_cm'), 'cm'),
              ('largest position', args.max_cm, s.get('pos_max_cm'), 'cm'),
              ('largest height', args.max_dz_cm, s.get('dz_max_cm'), 'cm'),
              ('largest tilt', args.max_normal_deg, s.get('normal_max_deg'), 'deg')]
    for name, limit, value, unit in checks:
        if limit is not None and value is not None and value > limit:
            print(f'FAIL: {name} error {value:.2f} {unit} > {limit} {unit}')
            ok = False
    if args.require_good:
        if not report:
            print('FAIL: --require-good without a survey_report.yaml')
            ok = False
        else:
            notgood = sorted(mid for mid, m in report.items() if m.get('verdict') != 'good')
            if notgood:
                print(f'FAIL: not good: {notgood}')
                ok = False
    if missing and not args.allow_missing:
        print(f'FAIL: {len(missing)} marker(s) missing')
        ok = False
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
