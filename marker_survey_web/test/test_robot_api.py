"""The survey, check and map API against recorded survey data: no ROS, no camera.

A survey recording (survey_data.npz, as the survey node writes it) of the
simulated office drive is played through a stand-in for the survey and
localizer nodes (FakeRobot), which answers the hub exactly as ros_bridge
relays the nodes: status dicts built by core/live.py, the solve by
core/solve.py, fix records by the localizer's core/check.py. The page's
steps are then walked through the HTTP API and the WebSocket.
"""
import base64
import json
import os
import shutil
import socket
import threading
import urllib.request

import numpy as np
import pytest

from marker_survey.core import live
from marker_survey.core import marker_map as mm
from marker_survey.core import simulate as sim
from marker_survey.core.frames import FRAME_MODES, FrameSettings
from marker_survey.core.path_csv import load_path_csv
from marker_survey.core.solve import DATA_FILE, MAP_FILE, REPORT_FILE, finish, load_dump, save_dump
from marker_survey.core.survey import Survey

from marker_localizer.core.check import fix_record
from marker_localizer.core.localize import Localizer, LocalizerSettings

from marker_survey_web import websocket as ws
from marker_survey_web.hub import Hub
from marker_survey_web.server import make_server

SITE = os.path.join(os.path.dirname(__file__), '..', '..', 'examples', 'ceiling', 'sites', 'office')


def quiet(*a):
    pass


def record(path):
    """A survey recording of two laps of the office, as the node saves it."""
    site = mm.load(os.path.join(SITE, 'marker_map.yaml'))
    poses = sim.route(load_path_csv(os.path.join(SITE, 'route.csv')), laps=2, step=0.1)
    noise = sim.Noise(map_sd=0.03, map_sd_yaw_deg=0.5)
    sv = sim.simulate(site.poses(), site.sizes(), sim.make_frames(poses, noise), sim.upward_camera(),
                      sim.settings_for('align_to_track', 0.45), 'align_to_track', noise)
    save_dump(path, sv, 'tag36h11', FrameSettings('align_to_track', frame_id='map'))


class FakeRobot:
    """The survey and localizer nodes as the hub sees them, driven from a recording."""

    def __init__(self, hub, recording, out_dir, site_map):
        rec, self.dictionary, _ = load_dump(recording)
        self.recorded, self.K, self.T_cam_base, self.settings = rec.keyframes, rec.K, rec.T_cam_base, rec.s
        self.hub, self.out_dir = hub, out_dir
        self.frame = FrameSettings('origin_marker', None, 'map')
        self.next, self.recording = 0, False
        self.loc_map, self.reloads = site_map, 0
        self.new_survey()

    def new_survey(self):
        self.survey = Survey(self.settings, self.K)
        self.survey.T_cam_base = self.T_cam_base
        self.report, self.report_views, self.last_save = None, {}, None

    def drive(self, n):
        for kf in self.recorded[self.next:self.next + n]:
            if self.recording:
                self.survey.add(kf.stamp, kf.T_world_cam, kf.detections, kf.T_ref_base)
        self.next += n
        self.push()

    def push(self):
        frame = {'mode': self.frame.mode, 'origin_marker': self.frame.origin_marker, 'frame_id': 'map',
                 'reference_frame': 'map', 'pose_source': 'odometry', 'pose_frame': ''}
        self.hub.on_survey_status(live.status_dict(
            self.survey, 'recording' if self.recording else 'paused', frame, self.report, self.report_views,
            [], None, {'camera': True, 'frames': self.next, 'dropped': 0, 'queued': 0,
                       'output_dir': self.out_dir, 'last_save': self.last_save, 'origin_seen': False}))
        m = mm.load(self.loc_map) if os.path.isfile(self.loc_map) else None
        self.hub.on_localizer_status({'map': self.loc_map, 'prior_topic': '/prior_pose', 'mode': 'planar',
                                      'markers': [] if m is None else [{'id': i} for i in m.ids()]})

    # the hub's robot interface
    def connected(self):
        return {'survey': True, 'localizer': True}

    def survey_set(self, params):
        mode = params.get('frame.mode', self.frame.mode)
        if mode not in FRAME_MODES or mode == 'pose_frame':
            return False, f'frame mode {mode} is not possible here'
        origin = params.get('frame.origin_marker', self.frame.origin_marker)
        self.frame = FrameSettings(mode, origin if mode == 'origin_marker' else None, 'map')
        return True, 'set'

    def survey_call(self, name, timeout):
        if name == 'start':
            try:
                self.frame.check()
            except ValueError as e:
                return False, str(e)
            self.recording = True
        elif name == 'pause':
            self.recording = False
        elif name == 'reset':
            self.new_survey()
        elif name == 'save':
            live.keep_previous(self.out_dir, [MAP_FILE, REPORT_FILE, DATA_FILE])
            os.makedirs(self.out_dir, exist_ok=True)
            save_dump(os.path.join(self.out_dir, DATA_FILE), self.survey, self.dictionary, self.frame)
            try:
                res = finish(self.survey, self.frame, self.dictionary, self.out_dir, log=quiet)
            except ValueError as e:
                return False, str(e)
            self.report = res['report']
            self.report_views = {k: v['views'] for k, v in live.marker_places(self.survey).items()}
            self.last_save = {'ok': True, 'message': 'saved'}
        self.push()
        return True, name

    def localizer_set_map(self, path):
        try:
            mm.load(path)
        except (OSError, ValueError) as e:
            return False, str(e)
        self.loc_map = path
        self.reloads += 1
        self.push()
        return True, f'the localizer reads {path}'

    def localizer_reload(self):
        return self.localizer_set_map(self.loc_map)


@pytest.fixture(scope='module')
def setup(tmp_path_factory):
    tmp = tmp_path_factory.mktemp('robot')
    recording = str(tmp / 'recorded.npz')
    record(recording)
    site_dir = tmp / 'site'
    site_dir.mkdir()
    site_map = str(site_dir / 'marker_map.yaml')
    shutil.copy(os.path.join(SITE, 'marker_map.yaml'), site_map)     # the map in use before
    hub = Hub(None, site_map)
    robot = FakeRobot(hub, recording, str(site_dir / 'survey'), site_map)
    hub.robot = robot
    robot.push()
    server = make_server('127.0.0.1', 0, hub)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield f'http://127.0.0.1:{server.server_address[1]}', hub, robot
    hub.stop()
    server.shutdown()
    server.server_close()


def call(url, path, body=None):
    req = urllib.request.Request(url + path, None if body is None else json.dumps(body).encode(),
                                 {'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=120) as r:
        return json.loads(r.read())


def test_without_a_robot_only_printing_answers():
    server = make_server('127.0.0.1', 0)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    url = f'http://127.0.0.1:{server.server_address[1]}'
    try:
        assert call(url, '/api/state') == {'type': 'state', 'robot': False}
        res = call(url, '/api/survey', {'action': 'start'})
        assert not res['ok'] and 'not connected to a robot' in res['error']
    finally:
        server.shutdown()
        server.server_close()


def test_the_survey_walked_through(setup):
    url, hub, robot = setup
    st = call(url, '/api/state')
    assert st['robot'] and st['connected'] == {'survey': True, 'localizer': True}
    opts = st['survey']['frame_options']
    assert opts[0]['available'] and opts[0]['mode'] == 'align_to_track' and "robot's map" in opts[0]['label']

    res = call(url, '/api/survey', {'action': 'start', 'frame': {'use': 'marker', 'marker': 'x'}})
    assert not res['ok'] and 'number' in res['error']
    res = call(url, '/api/survey', {'action': 'start', 'frame': {'use': 'robot_map'}})
    assert res['ok'] and robot.recording and robot.frame.mode == 'align_to_track'

    # the start of the drive: a marker seen once has a hint, and not every marker is seen
    robot.drive(20)
    st = call(url, '/api/state')['survey']
    assert st['state'] == 'recording' and st['keyframes'] == 20
    assert st['lights']['pass again'] == 1
    assert st['hints'] == ['marker 8: seen 1 time only: drive past it again']
    robot.drive(40)
    st = call(url, '/api/state')['survey']
    assert len(st['markers']) < 9
    first = call(url, '/api/survey', {'action': 'save'})
    assert first['ok'] and not robot.recording
    rep = first['report']
    assert rep['frame']['mode'] == 'align_to_track' and rep['markers']
    assert first['history'] == []
    st = call(url, '/api/state')['survey']
    assert all(m['source'] == 'report' for m in st['markers'])

    # pass again: the same recording grows, the first solve is kept
    assert call(url, '/api/survey', {'action': 'resume'})['ok'] and robot.recording
    robot.drive(len(robot.recorded))
    second = call(url, '/api/survey', {'action': 'save'})
    assert second['ok']
    hist = second['history']
    assert len(hist) == 1 and set(hist[0]['files']) == {MAP_FILE, REPORT_FILE, DATA_FILE}
    views1 = {m['id']: m['views'] for m in rep['markers']}
    views2 = {m['id']: m['views'] for m in second['report']['markers']}
    assert set(views2) > set(views1) and all(views2[k] > views1[k] for k in views1)
    assert second['report']['summary']['good'] >= rep['summary']['good']
    kept = os.path.join(robot.out_dir, 'history', hist[0]['name'], DATA_FILE)
    assert len(load_dump(kept)[0].keyframes) < len(robot.survey.keyframes)
    assert call(url, '/api/report')['report']['date'] == second['report']['date']


def test_use_the_map_with_confirmation_versions_and_undo(setup):
    url, hub, robot = setup
    if not os.path.isfile(os.path.join(robot.out_dir, MAP_FILE)):
        pytest.skip('needs the survey of the previous test')
    info = call(url, '/api/map')
    assert info['in_use']['exists'] and info['in_use']['markers'] == 9 and not info['in_use']['can_undo']
    d = info['diff']
    assert not d['first'] and d['markers'] == 9 and d['added'] == [] and d['removed'] == []
    assert not call(url, '/api/map', {'action': 'use'})['ok']          # no confirmation
    n = robot.reloads
    res = call(url, '/api/map', {'action': 'use', 'confirm': True})
    assert res['ok'] and res['kept'] and robot.reloads == n + 1
    assert res['in_use']['can_undo']
    with open(robot.loc_map) as f, open(os.path.join(robot.out_dir, MAP_FILE)) as g:
        assert f.read() == g.read()
    res = call(url, '/api/map', {'action': 'undo', 'confirm': True})
    assert res['ok'] and robot.reloads == n + 2
    with open(robot.loc_map) as f, open(os.path.join(SITE, 'marker_map.yaml')) as g:
        assert f.read() == g.read()
    res = call(url, '/api/map', {'action': 'undo', 'confirm': True})
    assert not res['ok'] and 'nothing to undo' in res['error']


def test_check_flags_a_marker_that_moved(setup):
    url, hub, robot = setup
    survey_map = os.path.join(robot.out_dir, MAP_FILE)
    if not os.path.isfile(survey_map):
        pytest.skip('needs the survey of the first test')
    res = call(url, '/api/check', {'action': 'map', 'which': 'survey'})
    assert res['ok'] and robot.loc_map == survey_map
    assert call(url, '/api/state')['map']['localizer_reads'] == 'survey'
    # drive through the office with marker 5 knocked 30 cm off its surveyed place
    the_map = mm.load(survey_map)
    truth = mm.load(survey_map)
    truth.get(5).x += 0.30
    cam = sim.upward_camera()
    loc = Localizer(the_map, cam.K, cam.T_base_cam, LocalizerSettings(mode='planar'))
    rng = np.random.default_rng(9)
    for f in sim.make_frames(sim.route(load_path_csv(os.path.join(SITE, 'route.csv')), step=0.1),
                             sim.Noise(map_sd=0.03, map_sd_yaw_deg=0.5)):
        dets = sim.project(truth.poses(), truth.sizes(), cam, f.T_map_base @ cam.T_base_cam, rng, 0.5)
        if dets:
            res = loc.locate(dets, T_prior=f.T_mapmeas_base)
            hub.on_fix(fix_record(res, dets, f.T_mapmeas_base, loc.accepted(res), f.stamp, 'planar'))
    c = call(url, '/api/check')
    assert c['fixes'] > 50 and c['flagged'] == [5]
    assert c['recent'] and c['distance_median_m'] < 0.1
    res = call(url, '/api/check', {'action': 'map', 'which': 'site'})
    assert res['ok'] and robot.loc_map == hub.map_path
    assert call(url, '/api/check')['fixes'] == 0             # counted afresh for the new map


def ws_connect(url):
    host, port = url.split('//')[1].split(':')
    s = socket.create_connection((host, int(port)), timeout=10)
    key = base64.b64encode(os.urandom(16)).decode()
    s.sendall((f'GET /api/live HTTP/1.1\r\nHost: {host}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
               f'Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n').encode())
    f = s.makefile('rb')
    head = b''
    while not head.endswith(b'\r\n\r\n'):
        head += f.read(1)
    assert b' 101 ' in head.split(b'\r\n')[0]
    assert ws.accept_key(key).encode() in head
    return s, f


def ws_send(s, text):
    data = text.encode()
    mask = os.urandom(4)
    s.sendall(bytes([0x81, 0x80 | len(data)]) + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))


def test_live_state_and_camera_only_when_asked(setup):
    url, hub, robot = setup
    s, f = ws_connect(url)
    try:
        fin, op, payload = ws.read_frame(f)
        st = json.loads(payload)
        assert op == ws.TEXT and st['type'] == 'state' and st['robot']
        assert hub.camera_wanted() == set()
        hub.on_camera('survey', b'\xff\xd8 not asked for')
        ws_send(s, json.dumps({'camera': 'survey'}))
        for _ in range(50):
            if hub.camera_wanted() == {'survey'}:
                break
            threading.Event().wait(0.05)
        assert hub.camera_wanted() == {'survey'}
        hub.on_camera('check', b'\xff\xd8 the other camera')
        hub.on_camera('survey', b'\xff\xd8 a frame')
        frames = []
        for _ in range(10):
            fin, op, payload = ws.read_frame(f)
            frames.append((op, payload))
            if op == ws.BINARY:
                break
        assert (ws.BINARY, b'\xff\xd8 a frame') in frames
        assert all(p != b'\xff\xd8 not asked for' for _, p in frames)
    finally:
        f.close()
        s.close()
    for _ in range(100):
        if not hub.clients:
            break
        threading.Event().wait(0.05)
    assert hub.camera_wanted() == set()
