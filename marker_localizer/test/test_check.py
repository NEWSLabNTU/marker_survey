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

from marker_localizer.core.check import CheckSettings, CheckStats, fix_record
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
        rec = fix_record(res, dets, f.T_mapmeas_base, loc.accepted(res), f.stamp, 'planar')
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
