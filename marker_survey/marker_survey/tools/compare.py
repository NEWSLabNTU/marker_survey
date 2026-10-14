"""marker_survey_compare: a surveyed marker map against the truth (or a tape survey).

    marker_survey_compare out/marker_map.yaml truth.yaml [--max-median-cm 3 --max-cm 5]

Both maps must be in the same frame. Prints per marker the position error
(dx, dy, dz), the rotation error and the tilt error (angle between the
normals), then the summary; exits 1 when a threshold is exceeded or a marker
of the truth is missing.
"""
import argparse
import math
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
            'rot_max_deg': float(col['rot_deg'].max()), 'normal_max_deg': float(col['normal_deg'].max())}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('result', help='marker_map.yaml written by the survey')
    ap.add_argument('truth', help='marker_map.yaml of the truth, same frame')
    ap.add_argument('--max-median-cm', type=float, help='fail above this median 3D position error')
    ap.add_argument('--max-cm', type=float, help='fail above this largest 3D position error')
    ap.add_argument('--max-dz-cm', type=float, help='fail above this largest height error')
    ap.add_argument('--max-normal-deg', type=float, help='fail above this largest tilt error')
    ap.add_argument('--allow-missing', action='store_true', help='a missing marker is not a failure')
    args = ap.parse_args(argv)
    got, want = mm.load(args.result), mm.load(args.truth)
    if got.frame_id != want.frame_id:
        print(f'WARNING: frames differ: {got.frame_id} vs {want.frame_id}')
    rows, missing = compare(got, want)
    print('  id   dx [cm]  dy [cm]  dz [cm]  |d| [cm]  rot [deg]  normal [deg]')
    for mid, r in sorted(rows.items()):
        print(f'  {mid:3d} {100 * r["dx"]:8.2f} {100 * r["dy"]:8.2f} {100 * r["dz"]:8.2f} '
              f'{100 * r["pos"]:9.2f} {r["rot_deg"]:10.2f} {r["normal_deg"]:13.2f}')
    for mid in missing:
        print(f'  {mid:3d}   MISSING from the survey')
    s = summary(rows)
    ok = True
    if s:
        print(f'position: median {s["pos_median_cm"]:.2f} cm, max {s["pos_max_cm"]:.2f} cm '
              f'(horizontal: median {s["horiz_median_cm"]:.2f} cm, max {s["horiz_max_cm"]:.2f} cm); '
              f'height: max {s["dz_max_cm"]:.2f} cm; rotation: max {s["rot_max_deg"]:.2f} deg; '
              f'tilt: max {s["normal_max_deg"]:.2f} deg')
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
    if missing and not args.allow_missing:
        print(f'FAIL: {len(missing)} marker(s) missing')
        ok = False
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
