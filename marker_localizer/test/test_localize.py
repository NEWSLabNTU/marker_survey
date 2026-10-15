"""The localizer's tests of phase 3, corner-level (no images): the marker corners are
projected with Gaussian pixel noise (0.5 px), the robot drives the example routes,
and the prior pose is the truth with 5 cm and 1 degree of noise.

| Case                                       | Target                                            |
|--------------------------------------------|---------------------------------------------------|
| ceiling, 9 markers, robot driving routes   | planar: mean 1 cm, max 3 cm                       |
| ceiling, two markers in view               | planar within 2 cm where 3d is worse              |
| wall course, 11 boards                     | 3d: no worse than the source localizer's method   |
| wall course on a flat floor                | planar against 3d per distance band (reported)    |
| misread id, moved marker                   | rejected by the gate                              |

Rendered recordings through the node run in CI (ci/synthetic_localize.sh).
"""
import math
import os

import numpy as np
import pytest

from marker_survey.core import marker_map as mm
from marker_survey.core import simulate as sim
from marker_survey.core.detection import Detection
from marker_survey.core.path_csv import load_path_csv
from marker_survey.core.se3 import make_T, rot_x, rot_y, rot_z

from marker_localizer.core.localize import Localizer, LocalizerSettings, planar_T
from marker_localizer.core import source_methods as src
from marker_localizer.tools.evaluate import BANDS, band_name, evaluate, summarize

EXAMPLES = os.path.join(os.path.dirname(__file__), '..', '..', 'examples')
SITES = {name: os.path.join(EXAMPLES, example, 'sites', name) for example, name in (
    ('wall_course', 'test_course'), ('ceiling', 'office'), ('ceiling', 'office_uneven'))}
PIXEL_SD = 0.5


def site(name):
    return (mm.load(os.path.join(SITES[name], 'marker_map.yaml')),
            load_path_csv(os.path.join(SITES[name], 'route.csv')))


def frames_of(route, laps=2, step=0.1, sweeps=0):
    return sim.make_frames(sim.route(route, laps=laps, step=step, sweeps=sweeps), sim.Noise())


def ok(rows, method):
    return [r['methods'][method] for r in rows if method in r['methods'] and r['methods'][method]['ok']]


@pytest.fixture(scope='module')
def office():
    m, route = site('office')
    cam = sim.upward_camera()
    rows = evaluate(m, cam, frames_of(route), ['planar', '3d', 'ceiling_source'], render=False,
                    pixel_sd=PIXEL_SD)
    return m, cam, rows


@pytest.fixture(scope='module')
def wall():
    m, route = site('test_course')
    cam = sim.forward_camera(960, 600)
    rows = evaluate(m, cam, frames_of(route, laps=1, sweeps=4), ['3d', 'planar', 'wall_source'],
                    render=False, pixel_sd=PIXEL_SD)
    return m, cam, rows


def test_ceiling_planar_on_the_office_routes(office):
    _, _, rows = office
    s = summarize(rows, ['planar'])['planar']
    assert s['fixes'] == s['frames_with_markers'], s
    assert s['all']['mean_cm'] <= 1.0 and s['all']['max_cm'] <= 3.0, s['all']
    assert s['yaw_deg_max'] < 1.5, s


def test_ceiling_planar_matches_the_source_method_on_a_flat_ceiling(office):
    _, _, rows = office
    s = summarize(rows, ['planar', 'ceiling_source'])
    assert s['planar']['all']['mean_cm'] <= s['ceiling_source']['all']['mean_cm'] + 0.05
    assert s['planar']['all']['max_cm'] <= s['ceiling_source']['all']['max_cm'] + 0.1


def test_uneven_ceiling_uses_each_markers_plane():
    m, route = site('office_uneven')
    cam = sim.upward_camera()
    rows = evaluate(m, cam, frames_of(route), ['planar', 'ceiling_source'], render=False,
                    pixel_sd=PIXEL_SD)
    s = summarize(rows, ['planar', 'ceiling_source'])
    assert s['planar']['all']['mean_cm'] <= 1.0 and s['planar']['all']['max_cm'] <= 3.0, s['planar']
    # the horizontal-plane method of the source does not know the 6 and 4 degree tilts
    tilted = [r for r in rows if set(r['seen']) & {4, 7}]
    ours = max(r['methods']['planar']['xy'] for r in tilted if r['methods']['planar']['ok'])
    theirs = max(r['methods']['ceiling_source']['xy'] for r in tilted
                 if r['methods']['ceiling_source']['ok'])
    assert ours < theirs, (ours, theirs)


def test_two_ceiling_markers_planar_within_2_cm_where_3d_is_worse(office):
    _, _, rows = office
    two = [r for r in rows if len(r['seen']) == 2]
    assert len(two) > 30
    p = np.array([r['methods']['planar']['xy'] for r in two])
    d = np.array([r['methods']['3d']['xy'] for r in two])
    assert p.max() < 0.02, p.max()
    assert d.mean() > 3 * p.mean() and d.max() > p.max(), (d.mean(), p.mean())


def test_one_ceiling_marker_3d_is_far_worse_than_planar(office):
    _, _, rows = office
    one = [r for r in rows if len(r['seen']) == 1]
    p = np.array([r['methods']['planar']['xy'] for r in one])
    d = np.array([r['methods']['3d']['xy'] for r in one])
    assert d.mean() > 5 * p.mean(), (d.mean(), p.mean())


def test_wall_3d_no_worse_than_the_source_method(wall):
    _, _, rows = wall
    s = summarize(rows, ['3d', 'wall_source'])
    ours, theirs = s['3d'], s['wall_source']
    assert ours['all']['mean_cm'] <= theirs['all']['mean_cm'], (ours['all'], theirs['all'])
    assert ours['all']['max_cm'] <= theirs['all']['max_cm'], (ours['all'], theirs['all'])
    assert ours['fixes'] >= theirs['fixes']


def test_wall_planar_against_3d_by_distance(wall):
    """Reported in the phase document; on a flat floor planar should not lose."""
    _, _, rows = wall
    s = summarize(rows, ['3d', 'planar'], key='xy')
    lines = []
    for b in BANDS:
        name = band_name(b)
        a, p = s['3d']['by_distance'][name], s['planar']['by_distance'][name]
        lines.append(f'{name}: 3d {a and round(a["mean_cm"], 2)} planar {p and round(p["mean_cm"], 2)}')
    print('\n'.join(lines))
    assert s['planar']['all']['mean_cm'] <= s['3d']['all']['mean_cm'] * 1.1, lines


def test_covariance_matches_the_errors(office, wall):
    for rows, m in ((office[2], 'planar'), (office[2], '3d'), (wall[2], '3d'), (wall[2], 'planar')):
        s = summarize(rows, [m])[m]
        assert s['nees_xy_within_99pct'] >= 0.95, (m, s['nees_xy_within_99pct'])
    # more markers, smaller sd
    recs = ok(office[2], 'planar')
    sd1 = np.median([x['sd_xy'] for x in recs if len(x['ids']) == 1])
    sd2 = np.median([x['sd_xy'] for x in recs if len(x['ids']) == 2])
    assert sd2 < sd1


def test_covariance_grows_with_distance():
    m, _ = site('test_course')
    cam = sim.forward_camera(960, 600)
    L = Localizer(m, cam.K, cam.T_base_cam, LocalizerSettings(mode='3d', max_position_sd=0))
    Tm = m.pose_of(10)
    sds = []
    for dist in (2.0, 4.0, 6.0):
        # straight in front of the board, facing it
        n = Tm[:3, 2]
        p = Tm[:3, 3] + dist * np.array([n[0], n[1], 0]) / np.hypot(n[0], n[1])
        yaw = math.atan2(-n[1], -n[0])
        T = planar_T(p[0], p[1], yaw)
        dets = sim.project({10: Tm}, {10: m.size_of(10)}, cam, T @ cam.T_base_cam,
                           np.random.default_rng(1), 0.0)
        r = L.locate(dets)
        sds.append(math.sqrt(r.fix.covariance[0, 0] + r.fix.covariance[1, 1]))
    assert sds[0] < sds[1] < sds[2], sds


def single_marker_frames(m, cam, rows, mid):
    return [r for r in rows if r['seen'] == [mid]]


def moved(m, mid, dx):
    out = mm.loads(mm.dumps(m))
    mk = out.get(mid)
    mk.x += dx
    return out


def swapped(m, a, b):
    out = mm.loads(mm.dumps(m))
    ma, mb = out.get(a), out.get(b)
    ma.id, mb.id = b, a
    return out


@pytest.mark.parametrize('mode', ['planar', '3d'])
@pytest.mark.parametrize('case', ['moved', 'misread'])
def test_the_gate_rejects_a_misread_id_and_a_moved_marker(mode, case):
    m, route = site('office')
    cam = sim.upward_camera()
    bad = moved(m, 5, 1.5) if case == 'moved' else swapped(m, 5, 2)
    frames = frames_of(route, laps=1)
    rows = evaluate(m, cam, frames, [mode], render=False, localize_map=bad, pixel_sd=PIXEL_SD)
    alone = [r['methods'][mode] for r in rows if r['seen'] == [5]]
    assert len(alone) > 10
    assert not any(x['ok'] for x in alone)
    # by the gate, or before it as too uncertain (3d with one marker)
    assert all(x['reason'].startswith(('gated', 'uncertain')) for x in alone), \
        {x['reason'] for x in alone}
    assert sum(x['reason'].startswith('gated') for x in alone) > len(alone) / 2
    # with others in view, the wrong marker is left out of the fit (an outlier)
    wrong = {5} if case == 'moved' else {5, 2}
    together = [r['methods'][mode] for r in rows if 5 in r['seen'] and len(r['seen']) > 1]
    assert together
    pub = ok(rows, mode)
    assert pub and not any(wrong & set(x['ids']) for x in pub)


def test_no_gate_without_a_prior():
    m, route = site('office')
    cam = sim.upward_camera()
    L = Localizer(moved(m, 5, 1.5), cam.K, cam.T_base_cam, LocalizerSettings(mode='planar'))
    f = [f for f in frames_of(route, laps=1)
         if [d.id for d in sim.project(m.poses(), m.sizes(), cam, f.T_map_base @ cam.T_base_cam,
                                       np.random.default_rng(0), 0)] == [5]][0]
    dets = sim.project(m.poses(), m.sizes(), cam, f.T_map_base @ cam.T_base_cam,
                       np.random.default_rng(0), PIXEL_SD)
    assert L.accepted(L.locate(dets))
    assert not L.accepted(L.locate(dets, T_prior=f.T_map_base))


def test_a_fix_far_from_a_wrong_prior_is_gated_and_marked_for_relocalization():
    """A kidnapped robot: the estimate is 2 m off. The fix is right, the gate turns it
    down, and gated() tells the node it may go to rejected_topic."""
    m, route = site('office')
    cam = sim.upward_camera()
    L = Localizer(m, cam.K, cam.T_base_cam, LocalizerSettings(mode='planar'))
    f = frames_of(route, laps=1)[40]
    dets = sim.project(m.poses(), m.sizes(), cam, f.T_map_base @ cam.T_base_cam,
                       np.random.default_rng(0), PIXEL_SD)
    assert dets
    wrong = f.T_map_base.copy()
    wrong[0, 3] += 2.0
    res = L.locate(dets, T_prior=wrong)
    assert not L.accepted(res) and L.gated(res)
    assert np.linalg.norm(res.fix.T_map_base[:2, 3] - f.T_map_base[:2, 3]) < 0.05
    good = L.locate(dets, T_prior=f.T_map_base)
    assert L.accepted(good) and not L.gated(good)


def test_single_marker_ambiguity_resolved_towards_the_prior():
    """A board seen at an angle from 4 m: its two IPPE solutions reproject alike; the
    prior picks the right one."""
    m, _ = site('test_course')
    cam = sim.forward_camera(960, 600)
    Tm = m.pose_of(10)
    L = Localizer(m, cam.K, cam.T_base_cam, LocalizerSettings(mode='3d', max_position_sd=0))
    rng = np.random.default_rng(3)
    with_prior, without = [], []
    for k in range(200):
        ang = math.radians(rng.uniform(-50, 50))
        n = Tm[:3, 2]
        az = math.atan2(n[1], n[0]) + ang
        dist = rng.uniform(3.0, 6.0)
        p = Tm[:2, 3] + dist * np.array([math.cos(az), math.sin(az)])
        yaw = math.atan2(Tm[1, 3] - p[1], Tm[0, 3] - p[0]) + math.radians(rng.uniform(-15, 15))
        T = planar_T(p[0], p[1], yaw)
        dets = sim.project({10: Tm}, {10: m.size_of(10)}, cam, T @ cam.T_base_cam, rng, PIXEL_SD)
        if not dets:
            continue
        a = L.locate(dets, T_prior=T @ make_T(rot_z(0.01), [0.05, -0.03, 0]))
        b = L.locate(dets)
        with_prior.append(np.linalg.norm(a.fix.T_map_base[:3, 3] - T[:3, 3]))
        without.append(np.linalg.norm(b.fix.T_map_base[:3, 3] - T[:3, 3]))
    with_prior, without = np.array(with_prior), np.array(without)
    assert len(with_prior) > 100
    assert with_prior.mean() < without.mean(), (with_prior.mean(), without.mean())
    assert np.percentile(with_prior, 95) < np.percentile(without, 95)


def test_planar_uses_the_robots_roll_and_pitch():
    m, _ = site('office')
    cam = sim.upward_camera()
    L = Localizer(m, cam.K, cam.T_base_cam, LocalizerSettings(mode='planar'))
    roll, pitch = math.radians(1.5), math.radians(-2.0)
    T = make_T(rot_z(0.3) @ rot_y(pitch) @ rot_x(roll), [0.2, -0.3, 0.0])
    dets = sim.project(m.poses(), m.sizes(), cam, T @ cam.T_base_cam, np.random.default_rng(0), 0.0)
    assert len(dets) >= 1
    good = L.locate(dets, roll=roll, pitch=pitch).fix
    flat = L.locate(dets).fix
    e_good = np.hypot(*(good.T_map_base[:2, 3] - T[:2, 3]))
    e_flat = np.hypot(*(flat.T_map_base[:2, 3] - T[:2, 3]))
    assert e_good < 0.002, e_good
    assert e_flat > 0.05, e_flat          # about 4.8 cm per degree at a 3 m ceiling


def test_unknown_rejected_and_far_markers_are_left_out():
    m, _ = site('office')
    cam = sim.upward_camera()
    L = Localizer(m, cam.K, cam.T_base_cam, LocalizerSettings(mode='planar', max_distance=2.0))
    T = planar_T(0.0, -0.5, 0.0)
    dets = sim.project(m.poses(), m.sizes(), cam, T @ cam.T_base_cam, np.random.default_rng(0), 0.0)
    extra = [Detection(99, dets[0].corners + 5.0), Detection(1, dets[0].corners, 'border')]
    r = L.locate(dets + extra)
    assert r.unknown == [99] and r.dropped[1] == 'border'
    L.s.max_distance = 1.0
    r = L.locate(dets)
    assert r.fix is None and all(v.startswith('far') for v in r.dropped.values())


def test_planar_mode_on_a_wall_board_seen_from_a_flat_floor():
    m, _ = site('test_course')
    cam = sim.forward_camera(960, 600)
    L = Localizer(m, cam.K, cam.T_base_cam, LocalizerSettings(mode='planar'))
    T = planar_T(2.0, 0.0, math.radians(-20))
    dets = sim.project(m.poses(), m.sizes(), cam, T @ cam.T_base_cam, np.random.default_rng(0), 0.0)
    assert dets
    fix = L.locate(dets).fix
    assert np.hypot(*(fix.T_map_base[:2, 3] - T[:2, 3])) < 1e-3


def test_the_source_methods_reproduce_their_behaviour():
    m, route = site('office')
    cam = sim.upward_camera()
    f = frames_of(route, laps=1)[40]
    dets = sim.project(m.poses(), m.sizes(), cam, f.T_map_base @ cam.T_base_cam,
                       np.random.default_rng(0), 0.0)
    T, n = src.ceiling_source_fix(dets, m, cam.K, cam.T_base_cam)
    assert n == len(dets) and np.hypot(*(T[:2, 3] - f.T_map_base[:2, 3])) < 1e-6
    # the wall method matches detections to the nearest map marker, whatever their id
    w, route = site('test_course')
    fc = sim.forward_camera(960, 600)
    f = frames_of(route, laps=1)[5]
    dets = sim.project(w.poses(), w.sizes(), fc, f.T_map_base @ fc.T_base_cam,
                       np.random.default_rng(0), 0.0)
    for d in dets:
        d.id = 999
    T, cov, dist = src.wall_source_fix(dets, w, fc.K, fc.T_base_cam, f.T_map_base)
    assert np.linalg.norm(T[:3, 3] - f.T_map_base[:3, 3]) < 0.05 and cov[0, 0] == 0.05
