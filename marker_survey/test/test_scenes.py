"""The four survey tests of phase 2, as corner-level simulations (no images).

- 11 wall boards, 10 x 4 m oval, 2 laps; forward camera, 102 deg; origin_marker:
  median 1 cm, max 4 cm;
- 9 ceiling markers at 3 m, office routes; upward camera, 70 deg, 640 x 480;
  pose_frame with a noisy map pose (6 cm): median under 4 cm;
- the same, align_to_track: median under 2 cm;
- uneven ceiling (2.8 to 3.4 m), 2 tilted markers; align_to_track: heights
  within 2 cm, tilts within 1 deg.

The same scenes run as rendered recordings through the node in CI
(ci/synthetic_survey.sh).
"""
import os

import numpy as np
import pytest

from marker_survey.core import frames as fr
from marker_survey.core import marker_map as mm
from marker_survey.core import simulate as sim
from marker_survey.core.path_csv import load_path_csv
from marker_survey.core.report import judge, ReportSettings, SurveyedMarker
from marker_survey.core.se3 import inv_T
from marker_survey.core.solve import finish
from marker_survey.tools.compare import compare, summary
from marker_survey.tools.synthetic import truth_map

SITES = os.path.join(os.path.dirname(__file__), '..', '..', 'examples', 'sites')


def site(name):
    return (mm.load(os.path.join(SITES, name, 'marker_map.yaml')),
            load_path_csv(os.path.join(SITES, name, 'route.csv')))


def survey(m, route, camera, mode, laps, step, sweeps=0, map_sd=0.0, seed=1, origin=None):
    noise = sim.Noise(drift=0.01, yaw_drift_deg_per_m=0.3, map_sd=map_sd,
                      map_sd_yaw_deg=1.0 if map_sd else 0.0, pixel_sd=0.3, seed=seed)
    frames = sim.make_frames(sim.route(route, laps=laps, step=step, sweeps=sweeps), noise)
    settings = sim.settings_for(mode, m.marker_size, pose_sd=max(map_sd, 0.01))
    sv = sim.simulate(m.poses(), m.sizes(), frames, camera, settings, mode, noise)
    return finish(sv, fr.FrameSettings(mode, origin), m.dictionary, None, log=lambda *a: None)


def errors(res, truth):
    rows, missing = compare(res['map'], truth)
    assert not missing
    return rows, summary(rows)


def test_wall_boards_in_the_frame_of_an_origin_marker():
    m, route = site('test_course')
    res = survey(m, route, sim.forward_camera(), 'origin_marker', laps=2, step=0.05, sweeps=4,
                 origin=10)
    T = inv_T(fr.origin_marker_frame(m.pose_of(10), np.zeros(3), np.array([0, 0, 1.0])))
    _, s = errors(res, truth_map(m, T, ''))
    assert s['pos_median_cm'] <= 1.0 and s['pos_max_cm'] <= 4.0, s
    assert all(x.mount == 'wall' for x in res['map'].markers)


@pytest.fixture(scope='module')
def office():
    return site('office')


def test_ceiling_in_a_noisy_map_frame(office):
    m, route = office
    res = survey(m, route, sim.upward_camera(), 'pose_frame', laps=2, step=0.1, map_sd=0.06)
    _, s = errors(res, m)
    assert s['pos_median_cm'] < 4.0, s
    assert res['map'].mount == 'ceiling'


def test_ceiling_aligned_to_the_track(office):
    m, route = office
    res = survey(m, route, sim.upward_camera(), 'align_to_track', laps=2, step=0.1, map_sd=0.06)
    _, s = errors(res, m)
    assert s['pos_median_cm'] < 2.0, s
    f = res['frame']
    assert f['fit_points'] > 100 and 0.05 < f['fit_rms_m'] < 0.12     # the 6 cm noise, per axis
    assert all(x.verdict == 'good' for x in res['markers']), [(x.id, x.flags) for x in res['markers']]


def test_uneven_ceiling_heights_and_tilts():
    m, route = site('office_uneven')
    res = survey(m, route, sim.upward_camera(), 'align_to_track', laps=2, step=0.1, map_sd=0.06)
    rows, s = errors(res, m)
    assert s['dz_max_cm'] < 2.0, s
    assert s['normal_max_deg'] < 1.0, s
    got = res['map'].by_id()
    assert got[4].angles['tilt_x_deg'] == pytest.approx(6.0, abs=1.0)
    assert got[7].angles['tilt_y_deg'] == pytest.approx(-4.0, abs=1.0)


def test_report_verdicts():
    def marker(mount='ceiling', **kw):
        sm = SurveyedMarker(1, np.eye(4), mount, 0.45, views=40, reproj_rms_px=0.3, tilt_deg=0.5,
                            height_above_floor=3.0, sd_xy=0.005, sd_z=0.005, sd_angle_deg=0.2)
        for k, v in kw.items():
            setattr(sm, k, v)
        judge(sm, ReportSettings())
        return sm

    assert marker().verdict == 'good' and marker().flags == []
    weak = marker(sd_z=0.03)
    assert weak.verdict == 'pass again' and 'weak height' in weak.flags[0]
    assert 'at an angle' in weak.hint
    assert marker(views=2).verdict == 'pass again'
    assert marker(sd_xy=0.05).verdict == 'pass again'
    assert marker(sd_angle_deg=2.0).verdict == 'pass again'
    assert marker(reproj_rms_px=3.0).verdict == 'bad'
    assert marker(tilt_deg=15.0).verdict == 'bad'
    assert marker(sd_xy=0.2).verdict == 'bad'
    assert marker('wall', height_above_floor=-0.1).verdict == 'bad'
    assert marker('wall', height_above_floor=0.1).verdict == 'pass again'
