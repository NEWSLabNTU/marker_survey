"""marker_localizer_evaluate: the localizer against the truth, without ROS.

Drives a robot along a route among the markers of a marker map (the same
scene as marker_survey_synthetic), renders every frame (or, with
--corners, projects the corners with pixel noise), runs the localizer in
each mode and the source implementations' methods on the same frames, and
prints the error of every method: overall, by the number of markers in the
fit, and by the distance to the markers. The prior pose (the state
estimator's, for the gate and the single-marker ambiguity) is the truth with
noise (--prior-sd, --prior-sd-deg).

    marker_localizer_evaluate --map examples/sites/office/marker_map.yaml \\
        --route examples/sites/office/route.csv --camera upward --laps 2 --rate 5 --speed 0.5 \\
        --methods planar,3d,ceiling_source

--localize-map localizes against another map than the one rendered (a moved
marker, a wrong id); --timing measures detection and fit per frame on one
thread.
"""

import argparse
import json
import math
import time
from types import SimpleNamespace

import numpy as np

from marker_survey.core import marker_map as mm
from marker_survey.core import simulate as sim
from marker_survey.core.detection import Detector, opencv_dictionary
from marker_survey.core.render import Renderer
from marker_survey.core.se3 import make_T, rot_z, rotation_angle
from marker_survey.tools.synthetic import scene as synthetic_scene

from marker_localizer.core import source_methods as src
from marker_localizer.core.localize import Localizer, LocalizerSettings, wrap, yaw_of

METHODS = ('planar', '3d', 'ceiling_source', 'wall_source')
BANDS = ((0.0, 3.0), (3.0, 5.0), (5.0, 8.0), (8.0, 1e9))


def band_name(b):
    return f'{b[0]:.0f}+ m' if b[1] > 1e8 else f'{b[0]:.0f}-{b[1]:.0f} m'


def noisy_prior(T, rng, sd, sd_deg):
    if sd <= 0 and sd_deg <= 0:
        return T.copy()
    e = make_T(rot_z(math.radians(rng.normal(0, sd_deg))), [rng.normal(0, sd), rng.normal(0, sd), 0])
    return T @ e


def degrade(img, blur_px, noise_grey, rng):
    """A rendered image as a real camera would give it: a Gaussian blur of blur_px
    (sigma) and sensor noise of noise_grey grey levels (sd)."""
    import cv2
    out = img.astype(np.float32)
    if blur_px > 0:
        out = cv2.GaussianBlur(out, (0, 0), blur_px)
    if noise_grey > 0:
        out = out + rng.normal(0, noise_grey, out.shape).astype(np.float32)
    return np.clip(out + 0.5, 0, 255).astype(np.uint8)


def errors(T, T_true, planar):
    d = T[:3, 3] - T_true[:3, 3]
    return {'xy': float(np.hypot(d[0], d[1])), 'xyz': float(np.linalg.norm(d)),
            'yaw_deg': abs(math.degrees(wrap(yaw_of(T[:3, :3]) - yaw_of(T_true[:3, :3])))),
            'rot_deg': math.degrees(rotation_angle(T[:3, :3], T_true[:3, :3]))}


def mean_distance(T_map_cam, site, ids):
    if not ids:
        return float('nan')
    return float(np.mean([np.linalg.norm(site.pose_of(i)[:3, 3] - T_map_cam[:3, 3]) for i in ids]))


def evaluate(site, camera, frames, methods, render=True, localize_map=None, pixel_sd=0.3,
             prior_sd=0.05, prior_sd_deg=1.0, seed=7, settings=None, timing=False,
             colour=False, keep=None, blur_px=0.0, noise_grey=0.0, detect_scale=1.0):
    """Per-frame records of every method. settings: {mode: LocalizerSettings}."""
    import cv2
    loc_map = localize_map or site
    settings = settings or {}
    locs = {m: Localizer(loc_map, camera.K, camera.T_base_cam,
                         settings.get(m) or LocalizerSettings(mode=m))
            for m in methods if m in ('planar', '3d')}
    detector = Detector(site.dictionary, detect_scale=detect_scale)
    renderer = Renderer(opencv_dictionary(site.dictionary), camera.K, camera.width, camera.height) \
        if render else None
    rng = np.random.default_rng(seed)
    rng_p = np.random.default_rng(seed + 1)
    poses, sizes = site.poses(), site.sizes()
    rows = []
    for k, f in enumerate(frames):
        T_wc = f.T_map_base @ camera.T_base_cam
        T_prior = noisy_prior(f.T_map_base, rng_p, prior_sd, prior_sd_deg)
        row = {'k': k, 'stamp': f.stamp, 'methods': {}}
        t_det = None
        if render:
            img = renderer.render(poses, sizes, T_wc)
            if blur_px > 0 or noise_grey > 0:
                img = degrade(img, blur_px, noise_grey, rng)
            if colour:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            t0 = time.perf_counter()
            dets = detector.detect_all(img)
            t_det = time.perf_counter() - t0
        else:
            dets = sim.project(poses, sizes, camera, T_wc, rng, pixel_sd)
        usable = [d for d in dets if d.rejected is None and d.id in loc_map.ids()]
        row['seen'] = [d.id for d in usable]
        row['t_detect'] = t_det
        if not usable:
            rows.append(row)
            continue
        for m in methods:
            t0 = time.perf_counter()
            rec = {'ok': False}
            if m in locs:
                r = locs[m].locate(dets, T_prior=T_prior)
                if r.fix is not None:
                    rec.update(ids=r.fix.ids, rms_px=r.fix.rms_px, reason=r.reason,
                               sd_xy=float(np.sqrt(np.trace(r.fix.covariance[:2, :2]) / 2)),
                               candidates=r.fix.candidates)
                    rec['ok'] = not r.reason
                    rec.update(errors(r.fix.T_map_base, f.T_map_base, m == 'planar'))
                    e = r.fix.T_map_base[:2, 3] - f.T_map_base[:2, 3]
                    rec['nees_xy'] = float(e @ np.linalg.solve(r.fix.covariance[:2, :2], e))
                else:
                    rec['reason'] = r.reason
            elif m == 'ceiling_source':
                sdets = src.source_ceiling_detect(img) if render else dets
                out = src.ceiling_source_fix(sdets, loc_map, camera.K, camera.T_base_cam)
                if out is not None:
                    T, n = out
                    rec.update(ok=True, ids=[d.id for d in sdets if d.id in loc_map.ids()])
                    rec.update(errors(T, f.T_map_base, True))
            elif m == 'wall_source':
                out = src.wall_source_fix(dets, loc_map, camera.K, camera.T_base_cam, T_prior)
                if out is not None:
                    T, _, _ = out
                    rec.update(ok=True, ids=[d.id for d in usable])
                    rec.update(errors(T, f.T_map_base, False))
            rec['t_fit'] = time.perf_counter() - t0
            if rec.get('ids'):
                rec['dist'] = mean_distance(T_wc, site, rec['ids'])
            row['methods'][m] = rec
        rows.append(row)
        if keep is not None:
            keep.append((f, dets, T_prior))
    return rows


def stats(vals):
    v = np.asarray(vals, float)
    if not len(v):
        return None
    return {'n': int(len(v)), 'mean_cm': float(v.mean() * 100), 'median_cm': float(np.median(v) * 100),
            'p95_cm': float(np.percentile(v, 95) * 100), 'max_cm': float(v.max() * 100)}


def summarize(rows, methods, key=None):
    """Error statistics per method; key: 'xy' for planar comparisons, 'xyz' for 3d."""
    out = {}
    framed = [r for r in rows if r['seen']]
    for m in methods:
        recs = [r['methods'][m] for r in framed if m in r['methods']]
        ok = [x for x in recs if x['ok']]
        k = key or ('xy' if m in ('planar', 'ceiling_source') else 'xyz')
        s = {'frames_with_markers': len(framed), 'fixes': len(ok), 'key': k,
             'all': stats([x[k] for x in ok]),
             'yaw_deg_max': max([x['yaw_deg'] for x in ok], default=None),
             'by_count': {}, 'by_distance': {}}
        for n in (1, 2, 3):
            sel = [x[k] for x in ok if (len(x['ids']) == n if n < 3 else len(x['ids']) >= 3)]
            s['by_count'][f'{n}' if n < 3 else '3+'] = stats(sel)
        s['by_distance_xy'] = {}
        for b in BANDS:
            sel = [x for x in ok if b[0] <= x.get('dist', -1) < b[1]]
            s['by_distance'][band_name(b)] = stats([x[k] for x in sel])
            s['by_distance_xy'][band_name(b)] = stats([x['xy'] for x in sel])
        nees = [x['nees_xy'] for x in ok if 'nees_xy' in x]
        if nees:
            s['nees_xy_within_99pct'] = float(np.mean(np.asarray(nees) < 9.21))
        fits = [x['t_fit'] for x in recs]
        s['t_fit_ms_mean'] = float(np.mean(fits) * 1e3) if fits else None
        out[m] = s
    det = [r['t_detect'] for r in rows if r.get('t_detect') is not None]
    out['_t_detect_ms_mean'] = float(np.mean(det) * 1e3) if det else None
    return out


def fmt(s):
    if not s:
        return '-'
    return f'{s["mean_cm"]:.2f} / {s["max_cm"]:.2f} ({s["n"]})'


def print_summary(summ, methods):
    print('error [cm]: mean / max (fixes)')
    for m in methods:
        s = summ[m]
        print(f'{m:15s} {s["key"]:3s} all {fmt(s["all"]):24s} fixes {s["fixes"]}/{s["frames_with_markers"]}'
              f'  yaw max {s["yaw_deg_max"] or 0:.2f} deg  fit {s["t_fit_ms_mean"]:.2f} ms'
              + (f'  NEES ok {s["nees_xy_within_99pct"]:.3f}' if 'nees_xy_within_99pct' in s else ''))
        print('    by markers  ' + '  '.join(f'{k}: {fmt(v)}' for k, v in s['by_count'].items()))
        print('    by distance ' + '  '.join(f'{k}: {fmt(v)}' for k, v in s['by_distance'].items()))
    if summ.get('_t_detect_ms_mean') is not None:
        print(f'detection {summ["_t_detect_ms_mean"]:.2f} ms per frame')


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--map', required=True)
    ap.add_argument('--route', required=True)
    ap.add_argument('--camera', choices=('forward', 'upward'), default='upward')
    ap.add_argument('--width', type=int)
    ap.add_argument('--height', type=int)
    ap.add_argument('--hfov', type=float)
    ap.add_argument('--laps', type=int, default=1)
    ap.add_argument('--open', action='store_true')
    ap.add_argument('--sweeps', type=int, default=0)
    ap.add_argument('--sweep-deg', type=float, default=70.0)
    ap.add_argument('--speed', type=float, default=0.3)
    ap.add_argument('--rate', type=float, default=3.0)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--methods', default='planar,3d')
    ap.add_argument('--corners', action='store_true', help='corner-level: no rendering')
    ap.add_argument('--pixel-sd', type=float, default=0.3, help='--corners: corner noise [px]')
    ap.add_argument('--prior-sd', type=float, default=0.05)
    ap.add_argument('--prior-sd-deg', type=float, default=1.0)
    ap.add_argument('--localize-map', help='localize against this map instead')
    ap.add_argument('--timing', action='store_true', help='one OpenCV thread')
    ap.add_argument('--colour', action='store_true', help='detect on a BGR image (conversion timed)')
    ap.add_argument('--max-frames', type=int, default=0)
    ap.add_argument('--json', help='write the summary here')
    ap.add_argument('--detect-scale', type=float, default=1.0, help='find markers at this scale')
    ap.add_argument('--blur-px', type=float, default=0.0, help='rendered: Gaussian blur sigma [px]')
    ap.add_argument('--noise-grey', type=float, default=0.0, help='rendered: sensor noise sd [grey levels]')
    return ap.parse_args(argv)


def main(argv=None):
    import cv2
    args = parse_args(argv)
    if args.timing:
        cv2.setNumThreads(1)
    scene_args = SimpleNamespace(map=args.map, route=args.route, camera=args.camera, width=args.width,
                                 height=args.height, hfov=args.hfov, marker_size=None, laps=args.laps,
                                 open=args.open, sweeps=args.sweeps, sweep_deg=args.sweep_deg,
                                 speed=args.speed, rate=args.rate, drift=0.0, yaw_drift_deg_per_m=0.0,
                                 map_noise=0.0, map_noise_yaw_deg=0.0, seed=args.seed)
    site, camera, frames = synthetic_scene(scene_args)
    if args.max_frames:
        frames = frames[:args.max_frames]
    methods = [m for m in args.methods.split(',') if m]
    for m in methods:
        if m not in METHODS:
            raise SystemExit(f'unknown method {m}: {", ".join(METHODS)}')
    loc_map = mm.load(args.localize_map) if args.localize_map else None
    rows = evaluate(site, camera, frames, methods, render=not args.corners, localize_map=loc_map,
                    pixel_sd=args.pixel_sd, prior_sd=args.prior_sd, prior_sd_deg=args.prior_sd_deg,
                    colour=args.colour, blur_px=args.blur_px, noise_grey=args.noise_grey,
                    detect_scale=args.detect_scale)
    summ = summarize(rows, methods)
    how = (f'corner-level, {args.pixel_sd} px noise' if args.corners else
           f'rendered, blur {args.blur_px} px, noise {args.noise_grey} grey levels')
    print(f'{len(frames)} frames, {camera.width} x {camera.height}, {how}')
    print_summary(summ, methods)
    if args.json:
        with open(args.json, 'w') as fh:
            json.dump(summ, fh, indent=1)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
