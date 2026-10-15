"""What a running survey shows (core/live.py): lights, hints, detections, history."""
import json
import os

import numpy as np
import pytest

from marker_survey.core import live
from marker_survey.core import marker_map as mm
from marker_survey.core import simulate as sim
from marker_survey.core.detection import Detection
from marker_survey.core.frames import FrameSettings
from marker_survey.core.path_csv import load_path_csv
from marker_survey.core.solve import finish

SITE = os.path.join(os.path.dirname(__file__), '..', '..', 'examples', 'ceiling', 'sites', 'office')


def quiet(*a):
    pass


def office_survey(fraction=1.0):
    site = mm.load(os.path.join(SITE, 'marker_map.yaml'))
    poses = sim.route(load_path_csv(os.path.join(SITE, 'route.csv')), laps=1, step=0.1)
    poses = poses[:max(2, int(len(poses) * fraction))]
    noise = sim.Noise(map_sd=0.03, map_sd_yaw_deg=0.5)
    frames = sim.make_frames(poses, noise)
    return site, sim.simulate(site.poses(), site.sizes(), frames, sim.upward_camera(),
                              sim.settings_for('align_to_track', 0.45), 'align_to_track', noise)


def test_lights_before_the_solve():
    assert live.live_light(1, 0.0)[0] == 'pass again'
    assert 'seen 1 time only' in live.live_light(1, 0.0)[1]
    assert 'one place' in live.live_light(5, 0.1)[1]
    assert live.live_light(5, 1.0) == ('good', '')


def test_status_of_a_partial_drive_names_the_markers_to_pass_again():
    _, sv = office_survey(0.25)
    st = live.status_dict(sv, 'recording', {'mode': 'align_to_track'})
    json.dumps(st)                                   # what the node publishes
    assert st['keyframes'] == len(sv.keyframes) and st['markers']
    assert st['trail'] and len(st['trail']) <= live.TRAIL_POINTS
    lights = {m['id']: m['light'] for m in st['markers']}
    assert set(lights) == set(sv.markers)
    assert sum(st['lights'].values()) == len(lights)
    for m in st['markers']:
        assert m['source'] == 'live'
        assert (m['light'] == 'good') == (not m['hint'])


def test_report_verdicts_replace_the_live_lights_until_new_views():
    _, sv = office_survey()
    res = finish(sv, FrameSettings('align_to_track', frame_id='map'), 'tag36h11', None, log=quiet)
    views = {k: v['views'] for k, v in live.marker_places(sv).items()}
    st = live.status_dict(sv, 'paused', {}, res['report'], views)
    verdicts = {m['id']: m['verdict'] for m in res['report']['markers']}
    for m in st['markers']:
        assert m['source'] == 'report' and m['light'] == verdicts[m['id']]
    one = next(iter(views))
    views[one] -= 1                                  # this marker has been seen again since
    st = live.status_dict(sv, 'recording', {}, res['report'], views)
    assert next(m for m in st['markers'] if m['id'] == one)['source'] == 'live'


def test_detections_known_new_and_too_small():
    _, sv = office_survey(0.25)
    c = np.array([[100, 100], [160, 100], [160, 160], [100, 160]], float)
    small = Detection(40, c * 0.1 + 300)
    cut = Detection(41, c, rejected='border')
    known = next(iter(sv.markers))
    states = live.detection_states(sv, [Detection(known, c), Detection(99, c), small, cut], set(sv.markers))
    assert [s['state'] for s in states] == ['known', 'new', 'unusable', 'unusable']
    assert states[3]['why'] == 'border'


def test_keep_previous_copies_the_last_solve(tmp_path):
    d = str(tmp_path)
    assert live.keep_previous(d, ['marker_map.yaml']) is None
    (tmp_path / 'marker_map.yaml').write_text('a')
    (tmp_path / 'survey_report.yaml').write_text('b')
    files = ['marker_map.yaml', 'survey_report.yaml', 'survey_data.npz']
    k1 = live.keep_previous(d, files, '20260101-120000')
    k2 = live.keep_previous(d, ['marker_map.yaml'], '20260101-120000')
    assert k1 != k2 and os.path.isfile(os.path.join(k1, 'survey_report.yaml'))
    assert (tmp_path / 'marker_map.yaml').read_text() == 'a'      # copied, not moved
    names = [h['name'] for h in live.history(d)]
    assert names == sorted(names, reverse=True) and len(names) == 2


@pytest.mark.parametrize('mode', ['origin_marker', 'align_to_track'])
def test_a_recorded_reference_track_anchors_only_an_aligned_result(mode):
    site, sv = office_survey()
    frame = FrameSettings(mode, origin_marker=5 if mode == 'origin_marker' else None, frame_id='map')
    res = finish(sv, frame, 'tag36h11', None, log=quiet)
    assert sv.s.track_sd == 0.05                      # put back after the solve
    sds = [m.sd_xy for m in res['markers']]
    assert all(s is not None and s > 0 for s in sds)
