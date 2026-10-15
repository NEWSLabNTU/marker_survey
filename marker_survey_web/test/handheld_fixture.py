"""A handheld walk under ceiling markers, for the web interface's tests. No ROS.

The site: eleven tag36h11 markers of 0.40 m in a row under a 2.9 m ceiling,
1.2 m apart (closer than the planner's 1.39 m for a webcam of 70 deg at 1.0 m),
along y = 0 from x = 0 to x = 12. A person holds a 640 x 480 webcam 1.0 m above
the floor, looking up, and walks at 0.5 m/s:

- the survey walk: along the row to x = 6, a step aside to y = 2.2 where no
  marker is in view (the survey loses track), back, and on to the end;
- the test walk: along the row to x = 4, aside to y = 2.2, along to x = 8 with
  no marker in view, back to the row and on: the localizer's pose freezes, then
  jumps by about 4 m (a kidnap).

HandheldRobot stands in for the survey node (pose_source markers) and the
localizer (3d, no prior) as ros_bridge relays them: the survey's status dict
from core/live.py with the tracking fields of the interface, which a camera-only
survey reports (a frame with no mapped marker in view is not placed: tracking
lost, the last mapped marker seen); the localizer's fix records from the real
Localizer and core/check.py on the projected corners; camera frames rendered
with the detections drawn. A walk plays in a thread at `speed` times real time.

    python3 handheld_fixture.py PORT OUT_DIR [SPEED]

serves the page with this stand-in (the browser test without ROS): the survey
walk starts with the survey, the test walk when the localizer is pointed at a map.
"""
import math
import os
import sys
import threading
import time

import numpy as np

from marker_survey.core import live
from marker_survey.core import marker_map as mm
from marker_survey.core import simulate as sim
from marker_survey.core.frames import FrameSettings
from marker_survey.core.se3 import inv_T, make_T, rot_z
from marker_survey.core.solve import DATA_FILE, MAP_FILE, REPORT_FILE, finish, save_dump
from marker_survey.core.survey import Survey, SurveySettings

from marker_localizer.core.check import fix_record
from marker_localizer.core.localize import Localizer, LocalizerSettings

CEILING, CAMERA, SIZE, SPACING, COUNT = 2.9, 1.0, 0.40, 1.2, 11
RATE = 10.0                 # [Hz] frames
SPEED = 0.5                 # [m/s] walking
SURVEY_WALK = [(0.0, 0.0), (6.0, 0.0), (6.0, 2.2), (6.0, 0.0), (12.0, 0.0)]
TEST_WALK = [(0.0, 0.0), (4.0, 0.0), (4.0, 2.2), (8.0, 2.2), (8.0, 0.0), (12.0, 0.0)]
DICTIONARY = 'tag36h11'


def site_yaml() -> str:
    lines = ['version: 1', 'frame_id: map', f'dictionary: {DICTIONARY}', f'marker_size: {SIZE}',
             'mount: ceiling', 'markers:']
    for i in range(COUNT):
        lines.append(f'  - {{id: {i}, x: {i * SPACING:.3f}, y: 0.000, z: {CEILING:.3f}, '
                     f'rotation_deg: {0.0 if i == 0 else (i * 37) % 90 - 45:.1f}}}')
    return '\n'.join(lines) + '\n'


def site():
    return mm.loads(site_yaml(), 'handheld site')


def camera():
    """The webcam on a base on the floor below it, looking straight up."""
    return sim.Camera(sim.pinhole_K(640, 480, 70.0), 640, 480,
                      make_T(sim.R_BASE_UPWARD, [0.0, 0.0, CAMERA]))


def walk(points, speed=SPEED, rate=RATE):
    """(stamp, T_map_base) along a polyline at `speed`, facing the way walked."""
    out, t = [], 0.0
    step = speed / rate
    for (x0, y0), (x1, y1) in zip(points[:-1], points[1:]):
        yaw = math.atan2(y1 - y0, x1 - x0)
        n = max(1, int(round(math.hypot(x1 - x0, y1 - y0) / step)))
        for k in range(n):
            f = k / n
            out.append((t, make_T(rot_z(yaw), [x0 + f * (x1 - x0), y0 + f * (y1 - y0), 0.0])))
            t += 1.0 / rate
    return out


def draw(img_gray, dets, colour_of):
    import cv2
    img = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
    for d in dets:
        c = colour_of(d)
        cv2.polylines(img, [d.corners.astype(np.int32).reshape(-1, 1, 2)], True, c, 3)
        org = (int(d.corners[:, 0].min()), max(20, int(d.corners[:, 1].min()) - 8))
        cv2.putText(img, str(d.id), org, cv2.FONT_HERSHEY_SIMPLEX, 0.9, c, 2, cv2.LINE_AA)
    ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])
    return jpg.tobytes()


GREEN, YELLOW, RED, ORANGE, GREY = (0, 220, 0), (0, 220, 255), (0, 0, 255), (0, 140, 255), (160, 160, 160)


class HandheldRobot:
    """The survey node (pose_source markers) and the localizer (3d, no prior) as the hub
    sees them, on a simulated handheld walk."""

    def __init__(self, hub, out_dir, map_path, speed=1.0, render=False, auto_walk=False):
        self.hub, self.out_dir, self.map_path = hub, out_dir, map_path
        self.truth = site()
        self.cam = camera()
        self.speed = speed               # x real time; 0: as fast as possible
        self.auto_walk = auto_walk       # walk when the survey starts and when the localizer gets a map
        self.render = render
        self.renderer = None
        self.frame = FrameSettings('origin_marker', None, 'map')
        self.recording = False
        self.loc_map = map_path
        self.localizer = None
        self.reloads = 0
        self.lock = threading.RLock()
        self.rng = np.random.default_rng(5)
        self.walking = None
        self.new_survey()
        self.load_localizer()

    # ------------------------------------------------------------ the survey
    def new_survey(self):
        s = SurveySettings(marker_size=SIZE)
        self.survey = Survey(s, self.cam.K)
        self.survey.T_cam_base = inv_T(self.cam.T_base_cam)
        self.report, self.report_views, self.last_save = None, {}, None
        self.tracking = {'tracking': 'lost', 'lost_since': None, 'last_known_marker': None}
        self.last_ok = None
        self.last_dets, self.T_base, self.frames, self.dropped = [], None, 0, 0

    def survey_frame(self, t, T_map_base, dets):
        known = set(self.survey.markers)
        in_view = [d.id for d in dets if d.id in known and self.survey.usable(d)]
        start = (not known and self.frame.origin_marker is not None
                 and any(d.id == self.frame.origin_marker for d in dets))
        self.frames += 1
        self.last_dets = live.detection_states(self.survey, dets, known)
        if in_view or start:
            self.survey.add(t, T_map_base @ self.cam.T_base_cam, dets, None)
            self.last_ok = t
            near = min(dets, key=lambda d: np.linalg.norm(d.corners.mean(0) - [320, 240]))
            self.tracking = {'tracking': 'ok', 'lost_since': None,
                             'last_known_marker': int(near.id if near.id in self.survey.markers
                                                      else in_view[0] if in_view else near.id)}
            self.T_base = T_map_base
        else:
            self.dropped += 1
            self.tracking = dict(self.tracking, tracking='lost',
                                 lost_since=None if self.last_ok is None else round(t - self.last_ok, 2))

    def push(self):
        frame = {'mode': self.frame.mode, 'origin_marker': self.frame.origin_marker, 'frame_id': 'map',
                 'reference_frame': '', 'pose_source': 'markers', 'pose_frame': '', 'odom_frame': 'map'}
        st = live.status_dict(
            self.survey, 'recording' if self.recording else 'paused', frame, self.report,
            self.report_views, self.last_dets, self.T_base,
            {'camera': True, 'frames': self.frames, 'dropped': self.dropped, 'queued': 0,
             'output_dir': self.out_dir, 'dictionary': DICTIONARY, 'marker_size': SIZE,
             'last_save': self.last_save,
             'origin_seen': self.frame.origin_marker in self.survey.markers})
        st['pose_source'] = 'markers'
        st.update(self.tracking)
        self.hub.on_survey_status(st)
        m = self.localizer.map if self.localizer is not None else None
        self.hub.on_localizer_status({
            'map': self.loc_map, 'loaded': m is not None, 'mode': '3d', 'enabled': True, 'camera': True,
            'prior_topic': '', 'counts': {}, 'frame_id': None if m is None else m.frame_id,
            'dictionary': DICTIONARY, 'last_fix_age': self.fix_age(),
            'markers': [] if m is None else [
                {'id': int(k), 'x': round(float(T[0, 3]), 3), 'y': round(float(T[1, 3]), 3),
                 'z': round(float(T[2, 3]), 3)} for k, T in sorted(m.poses().items())]})

    # ------------------------------------------------------------ the localizer
    def load_localizer(self):
        if self.loc_map and os.path.isfile(self.loc_map):
            the_map = mm.load(self.loc_map)
            # handheld: the camera is the localizer's base frame
            self.localizer = Localizer(the_map, self.cam.K, np.eye(4), LocalizerSettings(mode='3d'))
            self.localizer.map = the_map
        else:
            self.localizer = None
        self.last_fix_t = None
        self.sim_t = 0.0

    def fix_age(self):
        if self.last_fix_t is None:
            return None
        return round(max(0.0, self.sim_t - self.last_fix_t), 2)

    def localize_frame(self, t, dets):
        self.sim_t = t
        if self.localizer is None or not dets:
            return
        res = self.localizer.locate(dets)
        ok = self.localizer.accepted(res)
        if ok:
            self.last_fix_t = t
        self.hub.on_fix(fix_record(res, dets, None, ok, t, '3d'))

    # ------------------------------------------------------------ walking
    def detections(self, T_map_base):
        return sim.project(self.truth.poses(), self.truth.sizes(), self.cam, T_map_base @ self.cam.T_base_cam,
                           self.rng, 0.3)

    def play(self, which, points, t0=0.0):
        """Walk `points`; the survey records while recording, the localizer always runs."""
        steps = walk(points)
        start = time.monotonic()
        last_img = -1.0
        for t, T in steps:
            if self.speed > 0:
                delay = start + t / self.speed - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
            dets = self.detections(T)
            with self.lock:
                if self.recording:
                    self.survey_frame(t0 + t, T, dets)
                self.localize_frame(t0 + t, dets)
                if self.render and t - last_img >= 0.2 * max(self.speed, 1.0):
                    last_img = t
                    self.send_images(T, dets)
                if int(t * RATE) % 5 == 0:
                    self.push()
        with self.lock:
            self.push()
        return t0 + (steps[-1][0] if steps else 0.0) + 1.0 / RATE

    def start_walk(self, which, points, t0=0.0):
        def run():
            self.walk_end = self.play(which, points, t0)
        self.walking = threading.Thread(target=run, daemon=True, name=which)
        self.walking.start()

    def send_images(self, T, dets):
        wanted = self.hub.camera_wanted()
        if not wanted:
            return
        if self.renderer is None:
            import cv2
            from marker_survey.core.render import Renderer
            d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            self.renderer = Renderer(d, self.cam.K, 640, 480, background=170, supersample=1)
        gray = self.renderer.render(self.truth.poses(), self.truth.sizes(), T @ self.cam.T_base_cam)
        if 'survey' in wanted:
            known = set(self.survey.markers)
            self.hub.on_camera('survey', draw(gray, dets, lambda d: RED if not self.survey.usable(d)
                                              else GREEN if d.id in known else YELLOW))
        if 'check' in wanted:
            ids = set(self.localizer.map.ids()) if self.localizer else set()
            self.hub.on_camera('check', draw(gray, dets, lambda d: GREEN if d.id in ids else GREY))

    # ------------------------------------------------------------ the hub's robot interface
    def connected(self):
        return {'survey': True, 'localizer': True}

    def survey_set(self, params):
        mode = params.get('frame.mode', self.frame.mode)
        if mode != 'origin_marker':
            return False, f'frame mode {mode}: a camera-only survey starts from a marker'
        self.frame = FrameSettings('origin_marker', params.get('frame.origin_marker'), 'map')
        return True, 'set'

    def survey_call(self, name, timeout):
        with self.lock:
            if name == 'start':
                try:
                    self.frame.check()
                except ValueError as e:
                    return False, str(e)
                self.recording = True
                if self.auto_walk and (self.walking is None or not self.walking.is_alive()):
                    self.start_walk('survey', SURVEY_WALK)
            elif name == 'pause':
                self.recording = False
            elif name == 'reset':
                self.new_survey()
            elif name == 'save':
                live.keep_previous(self.out_dir, [MAP_FILE, REPORT_FILE, DATA_FILE])
                os.makedirs(self.out_dir, exist_ok=True)
                save_dump(os.path.join(self.out_dir, DATA_FILE), self.survey, DICTIONARY, self.frame)
                try:
                    res = finish(self.survey, self.frame, DICTIONARY, self.out_dir, log=lambda *a: None)
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
        with self.lock:
            self.loc_map = path
            self.reloads += 1
            self.load_localizer()
            self.push()
        if self.auto_walk and (self.walking is None or not self.walking.is_alive()):
            self.start_walk('test', TEST_WALK, 1000.0)
        return True, f'the localizer reads {path}'

    def localizer_reload(self):
        return self.localizer_set_map(self.loc_map)


def serve(port: int, out: str, speed: float):
    from marker_survey_web.hub import Hub
    from marker_survey_web.server import make_server
    os.makedirs(out, exist_ok=True)
    map_path = os.path.join(out, 'marker_map.yaml')
    hub = Hub(None, map_path)
    robot = HandheldRobot(hub, os.path.join(out, 'survey'), map_path, speed=speed, render=True,
                          auto_walk=True)
    hub.robot = robot
    robot.push()
    server = make_server('127.0.0.1', port, hub)

    def keep_pushing():                   # the nodes' status timers
        while True:
            time.sleep(0.5)
            with robot.lock:
                robot.push()
    threading.Thread(target=keep_pushing, daemon=True).start()
    print(f'handheld stand-in on http://127.0.0.1:{port}/', flush=True)
    server.serve_forever()


if __name__ == '__main__':
    serve(int(sys.argv[1]), sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 1.0)
