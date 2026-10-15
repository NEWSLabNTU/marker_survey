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

EXAMPLES = os.path.join(os.path.dirname(__file__), '..', '..', 'examples')
SITES = {name: os.path.join(EXAMPLES, example, 'sites', name) for example, name in (
    ('wall_course', 'test_course'), ('ceiling', 'office'), ('ceiling', 'office_uneven'))}


def site(name):
    return (mm.load(os.path.join(SITES[name], 'marker_map.yaml')),
            load_path_csv(os.path.join(SITES[name], 'route.csv')))


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


def hall_route():
    """A 24 m x 6 m oval: two straights and two turns of 3 m radius, from (0, 0) heading +x."""
    import math
    pts = [(x * 0.25, 0.0) for x in range(96)]
    pts += [(24 + 3 * math.cos(a), 3 + 3 * math.sin(a))
            for a in np.linspace(-math.pi / 2, math.pi / 2, 38, endpoint=False)]
    pts += [(24 - x * 0.25, 6.0) for x in range(96)]
    pts += [(3 * math.cos(a), 3 + 3 * math.sin(a))
            for a in np.linspace(math.pi / 2, 3 * math.pi / 2, 38, endpoint=False)]
    return pts


def test_uncertainty_tracks_the_real_error_in_a_long_hall():
    """The 1-sigma follows the real error: the verdicts use the relative one (the map
    among itself), and the one in the origin marker's frame, larger far from the
    origin, is reported beside it. The corner noise is measured, not assumed."""
    m = mm.load(os.path.join(os.path.dirname(__file__), 'fixtures', 'hall_marker_map.yaml'))
    res = survey(m, hall_route(), sim.forward_camera(), 'origin_marker', laps=1, step=0.1,
                 sweeps=4, origin=20)
    assert res['info']['pixel_noise_px'] == pytest.approx(0.3, rel=0.2)
    T = inv_T(fr.origin_marker_frame(m.pose_of(20), np.zeros(3), np.array([0, 0, 1.0])))
    truth = truth_map(m, T, '')
    rows, _ = errors(res, truth)
    ms = res['markers']
    assert all(x.verdict == 'good' for x in ms), [(x.id, x.flags) for x in ms]
    # relative: the error left after the best shift and turn of the whole map
    from scipy.optimize import least_squares
    from marker_survey.core.se3 import rot_z
    got = np.array([[x.x, x.y, x.z] for x in ms])
    want = np.array([truth.by_id()[x.id].position for x in ms])
    fit = least_squares(lambda q: (got @ rot_z(q[0]).T + q[1:] - want).ravel(), np.zeros(4))
    rel = np.linalg.norm((got @ rot_z(fit.x[0]).T + fit.x[1:] - want)[:, :2], axis=1)
    z_rel = rel / np.array([x.sd_xy for x in ms])
    z_frame = np.array([rows[x.id]['horiz'] / x.frame_sd_xy for x in ms if x.id != 20])
    assert 0.3 < np.median(z_rel) < 2.0 and z_rel.max() < 4.0, z_rel
    assert 0.3 < np.median(z_frame) < 2.0 and z_frame.max() < 4.0, z_frame
    assert max(x.frame_sd_xy for x in ms) > 2 * max(x.sd_xy for x in ms)
    assert res['report']['frame']['frame_sd_xy_max_cm'] > 0
