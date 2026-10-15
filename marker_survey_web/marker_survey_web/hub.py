"""The survey, check and map screens' logic, between the page and the robot. No ROS.

The Hub holds what the robot last reported (the survey node's ~/status, the
localizer's ~/status and ~/fixes, camera frames) and carries out the page's
actions through a `robot` object, which the ROS node provides
(ros_bridge.RosRobot) and the tests replace:

    robot.connected() -> {'survey': bool, 'localizer': bool}
    robot.survey_call(name, timeout) -> (ok, message)      name: start, pause, reset, save
    robot.survey_set(params: dict) -> (ok, message)        frame.mode, frame.origin_marker
    robot.localizer_set_map(path) -> (ok, message)         read this map from now on
    robot.localizer_reload() -> (ok, message)              read the map file again

Without a robot (marker_survey_web on a laptop) the page offers printing and
identifying only.
"""

import json
import os
import queue
import threading
import time

import yaml

from marker_localizer.core.check import CheckSettings, CheckStats
from marker_survey.core import live
from marker_survey.core.solve import MAP_FILE, REPORT_FILE

from marker_survey_web.mapstore import MapStore, MapStoreError

SAVE_TIMEOUT = 900.0
STALE = 3.0               # [s] a status older than this: the node is not answering


class ApiError(Exception):
    pass


class Client:
    """One open /api/live connection: what to send it, and which camera it shows."""

    def __init__(self):
        self.out = queue.Queue(maxsize=64)
        self.camera = None              # 'survey', 'check' or None

    def put(self, item, droppable=False):
        if droppable and self.out.qsize() > 2:
            return                       # a slow phone: skip camera frames, never the state
        try:
            self.out.put_nowait(item)
        except queue.Full:
            pass


def frame_options(st: dict) -> list:
    """The ways a survey can be placed, in plain words, for this robot's setup."""
    f = (st or {}).get('frame') or {}
    robot_map, why = None, ''
    if f.get('reference_frame'):
        robot_map = 'align_to_track'
        label = f"Use the robot's map ({f['reference_frame']})"
        text = ('The markers are placed in the map the robot already drives on: the survey '
                'follows the robot\'s own position in that map.')
    elif f.get('pose_source') == 'tf' and f.get('pose_frame'):
        robot_map = 'pose_frame'
        label = f"Use the robot's map ({f['pose_frame']})"
        text = 'The markers are placed in the frame of the robot\'s position.'
    else:
        label = "Use the robot's map"
        text = ''
        why = 'this robot gives the survey no map position; start from a marker instead'
    return [{'use': 'robot_map', 'mode': robot_map, 'label': label, 'text': text,
             'available': robot_map is not None, 'why': why},
            {'use': 'marker', 'mode': 'origin_marker', 'label': 'Start from a marker',
             'text': 'The floor below the marker you name is the origin; x points the way the '
                     'marker faces (for a marker on the ceiling, the way its top edge points).',
             'available': True, 'why': ''}]


class Hub:
    def __init__(self, robot=None, map_path: str = '', check: CheckSettings = None):
        self.robot = robot
        self.map_path = map_path or ''
        self.lock = threading.Lock()
        self.survey, self.survey_t = None, 0.0
        self.localizer, self.localizer_t = None, 0.0
        self.check = CheckStats(check)
        self.clients = set()
        self.busy = ''                   # 'solving' while a save runs
        self._stop = threading.Event()

    # ------------------------------------------------------------ from the robot
    def on_survey_status(self, st: dict):
        self.survey, self.survey_t = st, time.monotonic()

    def on_localizer_status(self, st: dict):
        self.localizer, self.localizer_t = st, time.monotonic()
        if not self.map_path and st.get('map') and not self._trial_path(st['map']):
            self.map_path = st['map']    # the map the robot's localizer was started with

    def on_fix(self, rec: dict):
        with self.lock:
            self.check.add(rec)

    def on_camera(self, which: str, jpeg: bytes):
        for c in list(self.clients):
            if c.camera == which:
                c.put(('binary', jpeg), droppable=True)

    # ------------------------------------------------------------ the page's connections
    def add_client(self) -> Client:
        c = Client()
        self.clients.add(c)
        c.put(('text', json.dumps(self.state())))
        return c

    def remove_client(self, c: Client):
        self.clients.discard(c)

    def on_message(self, c: Client, text: str):
        try:
            msg = json.loads(text)
        except ValueError:
            return
        if 'camera' in msg:
            c.camera = msg['camera'] if msg['camera'] in ('survey', 'check') else None

    def camera_wanted(self) -> set:
        return {c.camera for c in list(self.clients) if c.camera}

    def run_broadcast(self, period: float = 0.5):
        """Push the state to every connection; runs in its own thread."""
        while not self._stop.wait(period):
            if self.clients:
                text = json.dumps(self.state())
                for c in list(self.clients):
                    c.put(('text', text))

    def stop(self):
        self._stop.set()

    # ------------------------------------------------------------ state
    def connected(self) -> dict:
        now = time.monotonic()
        con = {'survey': self.survey is not None and now - self.survey_t < STALE,
               'localizer': self.localizer is not None and now - self.localizer_t < STALE}
        if self.robot is not None:
            try:
                r = self.robot.connected()
                con = {k: con[k] or bool(r.get(k)) for k in con}
            except Exception:  # noqa: BLE001
                pass
        return con

    def survey_map_path(self):
        st = self.survey or {}
        return os.path.join(st['output_dir'], MAP_FILE) if st.get('output_dir') else None

    def _trial_path(self, path) -> bool:
        sp = self.survey_map_path()
        return bool(sp and path and os.path.abspath(path) == os.path.abspath(sp))

    def state(self) -> dict:
        if self.robot is None:
            return {'type': 'state', 'robot': False}
        with self.lock:
            check = self.check.summary()
        loc = self.localizer
        in_use = None
        if loc and loc.get('map'):
            in_use = ('survey' if self._trial_path(loc['map'])
                      else 'site' if self.map_path and os.path.abspath(loc['map']) == os.path.abspath(
                          self.map_path) else 'other')
        survey = dict(self.survey) if self.survey else None
        if survey is not None:
            survey['frame_options'] = frame_options(survey)
        return {'type': 'state', 'robot': True, 'connected': self.connected(), 'busy': self.busy,
                'survey': survey, 'localizer': loc, 'check': check,
                'map': {'path': self.map_path, 'survey_map': self.survey_map_path(),
                        'localizer_reads': in_use}}

    # ------------------------------------------------------------ actions
    def _robot(self):
        if self.robot is None:
            raise ApiError('not connected to a robot: start marker_survey_web_node on the robot')
        return self.robot

    def _need_survey(self):
        if not self.connected()['survey']:
            raise ApiError('the survey node does not answer: is it running?')
        return self.survey or {}

    def survey_action(self, body: dict) -> dict:
        robot = self._robot()
        action = body.get('action')
        if action == 'start':
            st = self._need_survey()
            choice = body.get('frame') or {}
            params = self.frame_params(choice, st)
            ok, msg = robot.survey_set(params)
            if not ok:
                raise ApiError(f'the frame was not accepted: {msg}')
            for name in ('pause', 'reset', 'start'):
                ok, msg = robot.survey_call(name, 10.0)
                if not ok:
                    raise ApiError(msg)
            return {'ok': True, 'message': 'recording'}
        if action in ('resume', 'pause', 'reset'):
            self._need_survey()
            for name in {'resume': ['start'], 'pause': ['pause'], 'reset': ['pause', 'reset']}[action]:
                ok, msg = robot.survey_call(name, 10.0)
                if not ok:
                    raise ApiError(msg)
            return {'ok': True, 'message': msg}
        if action == 'save':
            self._need_survey()
            robot.survey_call('pause', 10.0)
            self.busy = 'solving'
            try:
                ok, msg = robot.survey_call('save', SAVE_TIMEOUT)
            finally:
                self.busy = ''
            if not ok:
                raise ApiError(f'not solved: {msg}')
            out = self.report()
            out['message'] = msg
            return out
        raise ApiError(f'unknown action {action!r}: start, resume, pause, save or reset')

    def frame_params(self, choice: dict, st: dict) -> dict:
        use = choice.get('use')
        if use == 'robot_map':
            opt = frame_options(st)[0]
            if not opt['available']:
                raise ApiError(opt['why'])
            return {'frame.mode': opt['mode']}
        if use == 'marker':
            try:
                mid = int(choice.get('marker'))
            except (TypeError, ValueError):
                raise ApiError('give the number of the marker to start from') from None
            if mid < 0:
                raise ApiError('marker numbers are 0 or more')
            return {'frame.mode': 'origin_marker', 'frame.origin_marker': mid}
        raise ApiError('choose how to place the map: the robot\'s map, or a marker')

    def report(self) -> dict:
        st = self.survey or {}
        out_dir = st.get('output_dir')
        if not out_dir:
            raise ApiError('no survey yet')
        path = os.path.join(out_dir, REPORT_FILE)
        if not os.path.isfile(path):
            return {'ok': True, 'report': None, 'history': live.history(out_dir),
                    'output_dir': out_dir}
        with open(path) as f:
            rep = yaml.safe_load(f)
        return {'ok': True, 'report': rep, 'history': live.history(out_dir), 'output_dir': out_dir,
                'map': os.path.join(out_dir, MAP_FILE)}

    def store(self) -> MapStore:
        if not self.map_path:
            raise ApiError('where the robot reads its map is not known yet: set map_path, or '
                           'start the localizer')
        return MapStore(self.map_path)

    def map_info(self) -> dict:
        out = {'ok': True, 'in_use': self.store().info() if self.map_path else None,
               'survey_map': self.survey_map_path()}
        sp = self.survey_map_path()
        if sp and os.path.isfile(sp) and self.map_path:
            try:
                out['diff'] = self.store().diff(sp)
            except MapStoreError as e:
                out['diff_error'] = str(e)
        return out

    def _reload(self) -> str:
        """Point the localizer at the map in use and read it again."""
        robot = self._robot()
        loc = self.localizer or {}
        if loc.get('map') and os.path.abspath(loc['map']) == os.path.abspath(self.map_path):
            ok, msg = robot.localizer_reload()
        else:
            ok, msg = robot.localizer_set_map(self.map_path)
        with self.lock:
            self.check.reset()
        return msg if ok else f'the localizer did not reload: {msg}'

    def map_action(self, body: dict) -> dict:
        action = body.get('action')
        store = self.store()
        try:
            if action == 'preview':
                sp = self.survey_map_path()
                if not sp or not os.path.isfile(sp):
                    raise ApiError('no surveyed map yet: save a survey first')
                return {'ok': True, 'diff': store.diff(sp)}
            if action not in ('use', 'undo'):
                raise ApiError(f'unknown action {action!r}: preview, use or undo')
            if body.get('confirm') is not True:
                raise ApiError('writing the map needs a confirmation')
            if action == 'use':
                sp = self.survey_map_path()
                if not sp or not os.path.isfile(sp):
                    raise ApiError('no surveyed map yet: save a survey first')
                res = store.use(sp)
            else:
                res = store.undo()
        except MapStoreError as e:
            raise ApiError(str(e)) from None
        res['reload'] = self._reload() if self.connected()['localizer'] else \
            'the localizer does not answer: it reads the map when it starts'
        res['ok'] = True
        res['in_use'] = store.info()
        return res

    def check_action(self, body: dict) -> dict:
        action = body.get('action')
        if action == 'reset':
            with self.lock:
                self.check.reset()
            return {'ok': True}
        if action == 'map':
            robot = self._robot()
            which = body.get('which')
            if which == 'survey':
                path = self.survey_map_path()
                if not path or not os.path.isfile(path):
                    raise ApiError('no surveyed map yet: save a survey first')
            elif which == 'site':
                path = self.store().path
            else:
                raise ApiError('which map: survey or site')
            ok, msg = robot.localizer_set_map(path)
            if not ok:
                raise ApiError(f'the localizer did not take the map: {msg}')
            with self.lock:
                self.check.reset()
            return {'ok': True, 'message': msg, 'map': path}
        raise ApiError(f'unknown action {action!r}: reset or map')

    def check_summary(self) -> dict:
        with self.lock:
            return dict(self.check.summary(), ok=True)
