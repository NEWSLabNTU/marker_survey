"""The handheld case through the HTTP API: plan, a camera-only survey, the localization test.

No ROS, no camera: handheld_fixture.HandheldRobot plays a simulated walk under a
row of ceiling markers through the hub, as the survey node (pose_source
markers) and the localizer (3d, no prior) would report it.
"""
import csv
import io
import json
import math
import os
import threading
import urllib.request

import pytest

from marker_survey.core import marker_plan as mp
from marker_survey.core.solve import MAP_FILE

from marker_survey_web.hub import Hub
from marker_survey_web.loctest import LocTest, LocTestSettings
from marker_survey_web.planner import hfov_from_camera_info, presets
from marker_survey_web.server import make_server

import handheld_fixture as hf


def call(url, path, body=None, raw=False):
    req = urllib.request.Request(url + path, None if body is None else json.dumps(body).encode(),
                                 {'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=120) as r:
        data = r.read()
        return (data.decode(), r.headers) if raw else json.loads(data)


@pytest.fixture(scope='module')
def plain():
    server = make_server('127.0.0.1', 0)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield f'http://127.0.0.1:{server.server_address[1]}'
    server.shutdown()
    server.server_close()


@pytest.fixture(scope='module')
def walk(tmp_path_factory):
    tmp = tmp_path_factory.mktemp('handheld')
    map_path = str(tmp / 'marker_map.yaml')
    hub = Hub(None, map_path)
    robot = hf.HandheldRobot(hub, str(tmp / 'survey'), map_path, speed=0.0)   # walked by the tests
    hub.robot = robot
    robot.push()
    server = make_server('127.0.0.1', 0, hub)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield f'http://127.0.0.1:{server.server_address[1]}', hub, robot
    hub.stop()
    server.shutdown()
    server.server_close()


# ---------------------------------------------------------------- plan
def test_the_plan_is_marker_plan_for_a_handheld_webcam(plain):
    res = call(plain, '/api/plan', {'preset': 'handheld', 'ceiling': 2.9, 'hfov': 70, 'width': 640,
                                    'height': 480, 'route_length': 30})
    pre = presets()['handheld']
    assert res['ok'] and pre['camera_height'] == 1.0 and pre['speed'] == 0.5
    assert res['inputs']['camera_height'] == 1.0 and res['inputs']['speed'] == 0.5
    p = mp.plan(2.9, 1.0, 70, 640, 480, 0.5, 0.02, 6.0, 5.0, 0.05, 0.05)
    r = mp.recommend(p)
    assert res['size_cm'] == round(r['size'] * 100, 1) == 40.0          # the 40 cm boards
    assert res['readable_from_cm'] == round(p['s_min'] * 100, 1)
    assert res['fits_up_to_cm'] == round(p['s_max'] * 100, 1)
    assert res['zone_radius_m'] == round(r['zone_radius'], 3)
    no_odom = r.get('route_spacing_no_odom', 2 * r['zone_radius'])
    assert res['route_spacing_no_odom_m'] == round(no_odom, 3)
    assert abs(res['route_spacing_no_odom_m'] - 2 * res['zone_radius_m']) < 0.002
    assert res['marker_count'] == math.ceil(30 / no_odom) + 1
    assert res['printed_cm'] == 50.0 and res['sheet'] == r['sheet']
    assert res['view_w_m'] > res['view_h_m'] > 1.9


def test_the_plan_takes_a_chosen_size_and_refuses_bad_input(plain):
    res = call(plain, '/api/plan', {'preset': 'robot', 'ceiling': 2.9, 'hfov': 70, 'size_cm': 45})
    assert res['ok'] and res['chosen'] and res['size_cm'] == 45.0 and res['inputs']['camera_height'] == 0.26
    assert 'marker_count' not in res
    assert not call(plain, '/api/plan', {'ceiling': 0.5, 'camera_height': 1.0, 'hfov': 70})['ok']
    assert 'give ceiling' in call(plain, '/api/plan', {'hfov': 70})['error']
    assert not call(plain, '/api/plan', {'ceiling': 2.9, 'hfov': 'wide'})['ok']
    assert not call(plain, '/api/plan', {'ceiling': 2.9, 'hfov': 70, 'preset': 'drone'})['ok']
    res = call(plain, '/api/plan', {'ceiling': 2.9, 'hfov': 70, 'size_cm': 200})
    assert not res['ok'] and 'never fits' in res['error']
    info = call(plain, '/api/info')
    assert set(info['presets']) >= {'robot', 'handheld'}


def test_the_field_of_view_from_a_camera_info():
    assert abs(hfov_from_camera_info(640, 457.0) - 70.0) < 0.01
    hub = Hub(object())
    hub.on_camera_info(640, 480, 457.0, '/camera/camera_info')
    assert hub.camera_info['hfov_deg'] == 70.0 and hub.camera_info['width'] == 640
    hub.on_camera_info(640, 480, 0.0)                 # an uncalibrated camera: ignored
    assert hub.camera_info['fx'] == 457.0


# ---------------------------------------------------------------- the survey with the camera alone
def test_a_camera_only_survey_shows_when_it_lost_track(walk):
    url, hub, robot = walk
    st = call(url, '/api/state')['survey']
    assert st['pose_source'] == 'markers'
    robot_map, marker = st['frame_options']
    assert not robot_map['available'] and 'camera alone' in robot_map['why']
    assert marker['available'] and 'walk on' in marker['text']
    assert not call(url, '/api/survey', {'action': 'start', 'frame': {'use': 'robot_map'}})['ok']

    assert call(url, '/api/survey', {'action': 'start', 'frame': {'use': 'marker', 'marker': 0}})['ok']
    seen = []
    orig = robot.push

    def watch():
        orig()
        s = hub.survey
        seen.append((s['tracking'], s['last_known_marker'], s['lost_since'], len(s['markers'])))
    robot.push = watch
    try:
        robot.play('survey', hf.SURVEY_WALK)
    finally:
        robot.push = orig
    lost = [s for s in seen if s[0] == 'lost' and s[2] is not None]
    assert lost, 'the step aside loses track'
    assert {s[1] for s in lost} <= {4, 5, 6}           # go back to a marker near x = 6
    assert max(s[2] for s in lost) > 2.0
    assert seen[-1][0] == 'ok' and seen[-1][3] == hf.COUNT
    st = call(url, '/api/state')['survey']
    assert len(st['trail']) > 10 and st['robot'] is not None
    res = call(url, '/api/survey', {'action': 'save'})
    assert res['ok'] and len(res['report']['markers']) == hf.COUNT
    m = {x['id']: x for x in res['report']['markers']}
    assert abs(m[10]['x'] - 12.0) < 0.1 and abs(m[10]['y']) < 0.1     # the origin marker's frame


# ---------------------------------------------------------------- the localization test
def test_the_localization_test_marks_the_kidnap_and_keeps_test_points(walk):
    url, hub, robot = walk
    if not os.path.isfile(os.path.join(robot.out_dir, MAP_FILE)):
        pytest.skip('needs the survey of the previous test')
    res = call(url, '/api/localize', {'action': 'map', 'which': 'survey'})
    assert res['ok'] and robot.loc_map.endswith(MAP_FILE)
    lz = call(url, '/api/localize')
    assert lz['fixes'] == 0 and lz['pose'] is None
    assert 'no pose' in call(url, '/api/localize', {'action': 'point', 'x': 0, 'y': 0})['error']

    t = robot.play('test', hf.TEST_WALK[:3], 1000.0)      # to the step aside: no marker in view
    lz = call(url, '/api/localize')
    assert lz['fix_age_s'] > 2.0 and lz['kidnaps'] == []
    frozen = lz['pose']
    assert abs(frozen['x'] - 4.0) < 1.0 and 'z' in frozen and abs(frozen['z'] - 1.0) < 0.1
    state = call(url, '/api/state')
    assert state['localize']['fix_age_s'] == lz['fix_age_s']

    robot.play('test', hf.TEST_WALK[2:], t)
    lz = call(url, '/api/localize')
    assert len(lz['kidnaps']) == 1      # from the frozen pose to the row at x = 8
    k = lz['kidnaps'][0]
    assert 3.5 < k['jump_m'] < 4.5 and k['gap_s'] > 5.0 and abs(k['to'][0] - 8.0) < 0.5
    assert any(p[2] == 1 for p in lz['track'])
    assert lz['fix_age_s'] < 0.5 and abs(lz['pose']['x'] - 11.95) < 0.2   # the walk's last step

    # test points: the tape says (11.95, 0), the localizer about the same
    res = call(url, '/api/localize', {'action': 'point', 'x': 11.95, 'y': 0.0})
    assert res['ok'] and res['point']['n'] == 1 and res['point']['error_m'] < 0.2
    assert not res['point']['stale']
    res = call(url, '/api/localize', {'action': 'point', 'x': 11.5, 'y': 0.3})
    assert res['ok'] and 0.4 < res['point']['error_m'] < 0.8
    assert not call(url, '/api/localize', {'action': 'point', 'x': 'here', 'y': 0})['ok']
    text, headers = call(url, '/api/localize.csv', raw=True)
    assert headers['Content-Type'].startswith('text/csv')
    rows = list(csv.DictReader(io.StringIO(text)))
    assert [r['n'] for r in rows] == ['1', '2'] and rows[1]['x_measured'] == '11.5'
    assert float(rows[0]['error_m']) < 0.2 and rows[0]['markers']
    lz = call(url, '/api/localize')
    assert lz['errors']['count'] == 2 and lz['errors']['max_m'] == float(rows[1]['error_m'])
    assert call(url, '/api/localize', {'action': 'delete', 'n': 2})['errors']['count'] == 1
    assert not call(url, '/api/localize', {'action': 'delete', 'n': 2})['ok']
    assert call(url, '/api/localize', {'action': 'reset', 'points': False})['ok']
    lz = call(url, '/api/localize')
    assert lz['track'] == [] and lz['kidnaps'] == [] and len(lz['points']) == 1
    assert call(url, '/api/localize', {'action': 'clear_points'})['errors']['count'] == 0


def test_a_gap_without_a_jump_is_no_kidnap():
    lt = LocTest(LocTestSettings(gap_s=1.0, jump_m=0.5), clock=lambda: 0.0)

    def fix(t, x, y=0.0):
        lt.add({'stamp': t, 'published': True, 'fix': {'x': x, 'y': y, 'z': 1.0, 'yaw_deg': 0, 'ids': [1]}})
    fix(0.0, 0.0)
    fix(0.1, 0.05)
    fix(3.0, 0.3)            # 2.9 s without a fix, but it stood still: a gap
    fix(3.1, 0.35)
    fix(3.3, 1.5)            # a jump without a gap: not a kidnap either (a bad fix)
    fix(6.0, 4.0)            # both: a kidnap
    lt.add({'stamp': 6.1, 'published': False, 'fix': {'x': 9, 'y': 9, 'yaw_deg': 0, 'ids': [2]}})
    lt.add({'stamp': 6.2, 'published': True, 'fix': None})
    s = lt.summary({'last_fix_age': 0.7})
    assert [p[2] for p in s['track']] == [0, 0, 2, 0, 0, 1]
    assert len(s['kidnaps']) == 1 and s['kidnaps'][0]['jump_m'] == 2.5
    assert s['fixes'] == 6 and s['images'] == 8 and s['fix_age_s'] == 0.7
    assert s['pose'] == {'x': 4.0, 'y': 0.0, 'yaw_deg': 0.0, 'ids': [1], 'stamp': 6.0, 'z': 1.0}


def test_a_long_track_is_thinned_but_keeps_its_kidnaps():
    lt = LocTest(LocTestSettings(keep=10000))
    for i in range(5000):
        t = i * 0.1 + (5.0 if i >= 2500 else 0.0)
        x = i * 0.05 + (3.0 if i >= 2500 else 0.0)
        lt.add({'stamp': t, 'published': True, 'fix': {'x': x, 'y': 0, 'yaw_deg': 0, 'ids': [0]}})
    s = lt.summary()
    assert len(s['track']) <= 1600 and sum(p[2] == 1 for p in s['track']) == 1
    assert s['track'][-1][0] == round(4999 * 0.05 + 3.0, 3)
