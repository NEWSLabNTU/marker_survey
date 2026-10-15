"""Checking a map while driving (core/check.py): fix records and per-marker agreement.

Corner-level drive through the office (no images), the prior the truth with
3 cm and 0.5 degree of noise; one marker of the map moved by 30 cm, less than
the gate, as a marker knocked off its place would be.
"""
import json
import os

import numpy as np
import pytest

from marker_survey.core import marker_map as mm
from marker_survey.core import simulate as sim
from marker_survey.core.path_csv import load_path_csv

from marker_localizer.core.check import CheckSettings, CheckStats, fix_record, marker_views
from marker_localizer.core.localize import Localizer, LocalizerSettings

SITE = os.path.join(os.path.dirname(__file__), '..', '..', 'examples', 'ceiling', 'sites', 'office')
MOVED = 5


def drive(the_map):
    truth = mm.load(os.path.join(SITE, 'marker_map.yaml'))
    cam = sim.upward_camera()
    loc = Localizer(the_map, cam.K, cam.T_base_cam, LocalizerSettings(mode='planar'))
    frames = sim.make_frames(sim.route(load_path_csv(os.path.join(SITE, 'route.csv')), laps=1, step=0.1),
                             sim.Noise(map_sd=0.03, map_sd_yaw_deg=0.5))
    rng = np.random.default_rng(4)
    stats = CheckStats(CheckSettings())
    records = []
    for f in frames:
        dets = sim.project(truth.poses(), truth.sizes(), cam, f.T_map_base @ cam.T_base_cam, rng, 0.5)
        if not dets:
            continue
        res = loc.locate(dets, T_prior=f.T_mapmeas_base)
        rec = fix_record(res, dets, f.T_mapmeas_base, loc.accepted(res), f.stamp, 'planar',
                         marker_views(loc, res, dets, f.T_mapmeas_base))
        json.dumps(rec)                     # what the node publishes
        records.append(rec)
        stats.add(rec)
    return records, stats


@pytest.fixture(scope='module')
def true_map():
    return drive(mm.load(os.path.join(SITE, 'marker_map.yaml')))


@pytest.fixture(scope='module')
def moved_map():
    m = mm.load(os.path.join(SITE, 'marker_map.yaml'))
    m.get(MOVED).x += 0.30
    return drive(m)


def test_records_hold_the_fix_the_prior_and_every_marker(true_map):
    records, _ = true_map
    r = next(r for r in records if r['fix'] and len(r['fix']['ids']) > 1)
    assert r['prior'] is not None and r['distance_m'] is not None
    assert set(r['fix']['ids']) <= {int(k) for k in r['markers']}
    assert all(r['markers'][str(i)]['state'] == 'used' for i in r['fix']['ids'])


def test_on_the_true_map_every_marker_agrees(true_map):
    _, stats = true_map
    s = stats.summary()
    assert s['fixes'] > 50 and s['with_prior'] == s['fixes']
    assert s['agreed'] / s['with_prior'] > 0.95
    assert s['distance_median_m'] < 0.06
    assert s['flagged'] == []


def test_a_moved_marker_is_flagged_and_only_it(moved_map):
    _, stats = moved_map
    s = stats.summary()
    assert s['flagged'] == [MOVED]
    row = next(r for r in s['markers'] if r['id'] == MOVED)
    assert row['agreement'] < 0.8 and row['compared'] >= 5


def test_reset_and_records_without_a_prior():
    stats = CheckStats()
    stats.add({'fix': {'x': 0, 'y': 0, 'yaw_deg': 0, 'ids': [1]}, 'prior': None, 'distance_m': None,
               'published': True, 'markers': {'1': {'state': 'used', 'residual_px': 0.2}}})
    s = stats.summary()
    assert s['fixes'] == 1 and s['with_prior'] == 0 and s['markers'][0]['agreement'] is None
    stats.reset()
    assert stats.summary()['fixes'] == 0


def per_marker(records, key):
    out = {}
    for r in records:
        for mid, m in r['markers'].items():
            if key in m:
                out.setdefault(int(mid), []).append(m[key])
    return out


def test_every_marker_in_view_is_checked_on_its_own(true_map, moved_map):
    """A robot project checks each marker: its residual with the fix and where the
    image alone puts it, from the prior pose, against the map."""
    for records, moved in ((true_map[0], None), (moved_map[0], MOVED)):
        off = {k: float(np.median(v)) for k, v in per_marker(records, 'offset_m').items()}
        res = per_marker(records, 'residual_px')
        assert len(off) == 9 and len(res) == 9
        for mid, o in off.items():
            if mid == moved:
                assert 0.2 < o < 0.4, (mid, o)          # moved by 0.30 m
            else:
                assert o < 0.12, (mid, o)               # the prior's 3 cm noise, at 2.7 m
        others = np.concatenate([v for k, v in res.items() if k != moved])
        assert np.median(others) < 1.0
        if moved is not None:          # alone in view the fix follows it; with others not
            assert np.max(res[moved]) > 5 * np.percentile(others, 95), (res[moved], others)
    r = next(r for r in moved_map[0] if r['markers'].get(str(MOVED), {}).get('state') == 'outlier')
    m = r['markers'][str(MOVED)]
    assert m['seen_from'] == 'prior' and m['residual_px'] > 2 and set(m['seen']) == {'x', 'y', 'z'}


def test_without_a_prior_the_fix_places_the_markers():
    truth = mm.load(os.path.join(SITE, 'marker_map.yaml'))
    cam = sim.upward_camera()
    loc = Localizer(truth, cam.K, cam.T_base_cam, LocalizerSettings(mode='planar'))
    T = truth.pose_of(5).copy()                 # below marker 5
    T[:3, :3] = np.eye(3)
    T[2, 3] = 0.0
    dets = sim.project(truth.poses(), truth.sizes(), cam, T @ cam.T_base_cam,
                       np.random.default_rng(1), 0.3)
    res = loc.locate(dets)
    views = marker_views(loc, res, dets, None)
    assert 5 in views
    assert all(v['seen_from'] == 'fix' and v['offset_m'] < 0.05 for v in views.values()), views


def test_the_checks_as_diagnostics(moved_map):
    pytest.importorskip('diagnostic_msgs')
    from builtin_interfaces.msg import Time
    from diagnostic_msgs.msg import DiagnosticStatus
    from marker_localizer.diagnostics import marker_diagnostics
    r = next(r for r in moved_map[0] if r['markers'].get(str(MOVED), {}).get('state') == 'outlier')
    arr = marker_diagnostics(r, Time(sec=5))
    assert arr.header.stamp.sec == 5
    st = {s.name: s for s in arr.status}
    s = st[f'marker_localizer: marker {MOVED}']
    assert s.level == DiagnosticStatus.WARN and s.message.startswith('outlier')
    vals = {kv.key: kv.value for kv in s.values}
    assert vals['state'] == 'outlier' and float(vals['offset_m']) > 0.2
    assert {'residual_px', 'seen_x', 'seen_y', 'seen_z', 'seen_from', 'angle_deg'} <= set(vals)
    assert all(x.level == DiagnosticStatus.OK for n, x in st.items() if n != s.name)
