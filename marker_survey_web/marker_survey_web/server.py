"""marker_survey_web: the browser interface. Plan and print markers, identify markers; on the
robot (or a laptop with a camera) also survey, check, test and use the map.

    marker_survey_web                      # http://127.0.0.1:8090 on this computer
    marker_survey_web --host 0.0.0.0       # reachable from a phone on the same network
    marker_survey_web --open               # and open a browser
    ros2 run marker_survey_web marker_survey_web_node   # on the robot: all five steps (node.py)

Python's standard HTTP server and the core modules of marker_survey: no build
step, no internet. Without ROS (a laptop) the page prints and identifies
markers; the ROS node adds the survey, check and map screens, which talk to
the survey and localizer nodes through hub.py. The page and the command-line
tools share the same code (core/sheets.py, core/identify.py): a PDF
downloaded here is the one marker_sheets writes.

API (JSON in and out unless said otherwise):

  GET  /api/info              dictionaries, papers, defaults, the plan's presets
  POST /api/plan              ceiling and camera heights, field of view, image size, speed,
                              exposure (or a preset: robot, handheld), route length ->
                              size range, recommended size and sheet, zone R, spacings, count
  POST /api/sheets/plan       a print job -> summary, warnings, SVG previews
  POST /api/sheets/download   a print job + "format": pdf | zip | csv -> the file
  POST /api/size              ceiling height, camera -> recommended marker size
  POST /api/identify          {"image": data URL, "dictionary"} -> IDs, annotated image

On the robot (hub.py; without a robot they answer {"ok": false}):

  GET  /api/state             robot connected, survey status, localizer status, check, map
  POST /api/survey            {"action": "start", "frame": {"use": "robot_map"} or
                              {"use": "marker", "marker": 7}}; "resume", "pause", "save"
                              (solves, answers with the report), "reset"
  GET  /api/report            the last survey_report.yaml and the kept earlier solves
  GET  /api/map               the map in use, its versions, what the survey would change
  POST /api/map               {"action": "preview"}; {"action": "use" | "undo", "confirm": true}
  GET  /api/check             fixes against the fused pose, per-marker agreement
  POST /api/check             {"action": "reset"}; {"action": "map", "which": "survey" | "site"}
  GET  /api/localize          the localization test: pose, time since the last fix, the
                              trajectory with its kidnap events, test points and their errors
  POST /api/localize          {"action": "point", "x": 1.2, "y": 0.0} (measured with a tape);
                              {"action": "delete", "n": 3}; {"action": "clear_points"};
                              {"action": "reset"} (the path and the points; "points": false
                              keeps the points);
                              {"action": "map", "which": "survey" | "site"}
  GET  /api/localize.csv      the test points as CSV
  WS   /api/live              the state twice a second (JSON); camera frames as JPEG (binary)
                              while the page asks for them: {"camera": "survey" | "check" | null}
                              (check: the localizer's camera, on the check and test screens)

A print job: {"size": "45cm", "count": 24, "ids": "", "dictionary": "tag36h11",
"paper": "auto", "border": "", "per_page": "auto", "split": false,
"labels": true, "back_labels": false, "exclude": "", "site_map": "<yaml>"}.
"""
import argparse
import base64
import json
import os
import queue
import threading
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from marker_survey_web import websocket as ws
from marker_survey_web.hub import ApiError, Hub

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
MAX_BODY = 30 * 1024 * 1024          # a phone photo as a data URL fits
PREVIEW_PAGES = 6
CONTENT_TYPES = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
                 '.css': 'text/css; charset=utf-8', '.svg': 'image/svg+xml', '.png': 'image/png'}


def _job(body: dict):
    from marker_survey.core import marker_map
    from marker_survey.core.sheets import SheetJob, parse_ids, parse_length
    size = body.get('size')
    if not size:
        raise ApiError('give the marker size, for example 45cm')
    count = body.get('count')
    ids_text = str(body.get('ids') or '').strip()
    if count in ('', None) and not ids_text:
        raise ApiError('give how many markers, or which IDs')
    exclude = set(parse_ids(body['exclude'])) if str(body.get('exclude') or '').strip() else set()
    site = body.get('site_map')
    if site:
        exclude |= set(marker_map.loads(site, 'site map').ids())
    border = str(body.get('border') or '').strip()
    base = str(body.get('basename') or 'markers').strip() or 'markers'
    return SheetJob(size_mm=parse_length(size, 'size'),
                    count=int(count) if count not in ('', None) and not ids_text else None,
                    ids=parse_ids(ids_text) if ids_text else None,
                    dictionary=body.get('dictionary') or 'tag36h11',
                    paper=body.get('paper') or 'auto',
                    border_mm=parse_length(border, 'border') if border else None,
                    per_page=str(body.get('per_page') or 'auto'),
                    labels=bool(body.get('labels', True)),
                    back_labels=bool(body.get('back_labels', False)),
                    exclude=exclude, split=bool(body.get('split', False)), basename=base)


def api_info(_body=None) -> dict:
    import cv2
    from marker_survey.core.detection import opencv_dictionary_name
    from marker_survey.core.identify import COMMON
    from marker_survey.core.sheets import AUTO_PAPERS, DEFAULT_DICTIONARY, PAPERS
    names = COMMON + ['DICT_4X4_50', 'DICT_4X4_100', 'DICT_4X4_250', 'DICT_5X5_50', 'DICT_5X5_100',
                      'DICT_5X5_250', 'DICT_6X6_250', 'tag36h10']
    dicts = [n for n in dict.fromkeys(names) if hasattr(cv2.aruco, opencv_dictionary_name(n))]
    from marker_survey_web.planner import presets
    return {'dictionaries': dicts, 'default_dictionary': DEFAULT_DICTIONARY,
            'papers': list(PAPERS), 'auto_papers': AUTO_PAPERS, 'presets': presets()}


def api_plan(body: dict) -> dict:
    from marker_survey.core.sheets import page_svg, plan_job, size_label, size_text
    plan = plan_job(_job(body))
    marker_pages = [p for p in plan.pages]
    previews = [{'label': p.label, 'svg': page_svg(p, 300)} for p in marker_pages[:PREVIEW_PAGES]]
    if plan.contact:
        previews.append({'label': plan.contact[0].label, 'svg': page_svg(plan.contact[0], 300)})
    s = plan.summary()
    s['size_text'] = size_text(plan.job.size_mm)
    s['size_label'] = size_label(plan.job.size_mm)
    s['more_pages'] = max(0, len(marker_pages) - PREVIEW_PAGES)
    return {'ok': True, 'summary': s, 'previews': previews}


def api_download(body: dict):
    from marker_survey.core.sheets import outputs, plan_job, zip_bytes
    plan = plan_job(_job(body))
    files = outputs(plan)
    fmt = body.get('format') or ('zip' if plan.job.split else 'pdf')
    base = plan.job.basename
    if fmt == 'csv':
        name, data = files[-1]
        return name, 'text/csv; charset=utf-8', data
    if fmt == 'pdf' and not plan.job.split:
        name, data = files[0]
        return name, 'application/pdf', data
    return f'{base}.zip', 'application/zip', zip_bytes(files)


def api_size(body: dict) -> dict:
    from marker_survey.core.marker_plan import plan, recommend
    try:
        p = plan(float(body['ceiling']), float(body.get('camera_height', 0.26)), float(body['hfov']),
                 int(body.get('width', 640)), int(body.get('height', 480)), float(body.get('speed', 0.5)),
                 float(body.get('exposure', 0.02)), 6.0, 5.0, float(body.get('drift', 0.05)), 0.05)
    except (KeyError, ValueError, TypeError):
        raise ApiError('give the ceiling height (m) and the camera field of view (degrees)')
    r = recommend(p)
    if r is None:
        return {'ok': False, 'error': 'no size works with this camera at this height: use a higher '
                                      'resolution, a narrower lens or a slower robot'}
    return {'ok': True, 'size_cm': round(r['size'] * 100, 1), 'px_per_cell': round(r['px_per_cell'], 1),
            'zone_radius_m': round(r['zone_radius'], 2),
            'grid_spacing_m': round(r['grid_spacing'], 2) if r['grid_spacing'] else None,
            'route_spacing_m': round(r['route_spacing'], 2) if r['route_spacing'] else None,
            'readable_from_cm': round(p['s_min'] * 100), 'fits_up_to_cm': round(p['s_max'] * 100)}


def api_site_plan(body: dict) -> dict:
    from marker_survey_web.planner import PlanError, site_plan
    try:
        return site_plan(body)
    except PlanError as e:
        raise ApiError(str(e)) from None


def api_identify(body: dict) -> dict:
    import cv2
    import numpy as np
    from marker_survey.core.identify import annotate, identify, summary
    data = str(body.get('image') or '')
    if ',' in data and data.startswith('data:'):
        data = data.split(',', 1)[1]
    try:
        raw = base64.b64decode(data, validate=False)
    except ValueError:
        raise ApiError('the photo could not be read')
    img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ApiError('the photo could not be read; use a JPEG or PNG')
    found = identify(img, body.get('dictionary') or 'auto')
    family, ids = summary(found)
    out = annotate(img, found)
    h, w = out.shape[:2]
    scale = min(1.0, 1600.0 / max(h, w))
    if scale < 1.0:
        out = cv2.resize(out, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    ok, jpg = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return {'ok': True, 'dictionary': family, 'ids': ids,
            'markers': [f.as_dict() for f in found],
            'image': 'data:image/jpeg;base64,' + base64.b64encode(jpg.tobytes()).decode()}


ROUTES = {'/api/sheets/plan': api_plan, '/api/size': api_size, '/api/identify': api_identify,
          '/api/plan': api_site_plan}


class Handler(BaseHTTPRequestHandler):
    server_version = 'marker_survey_web'

    def log_message(self, fmt, *args):          # quiet; errors still go to stderr
        pass

    def _send(self, status, ctype, data: bytes, extra=None):
        self.send_response(status)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(data)

    def _json(self, obj, status=HTTPStatus.OK):
        self._send(status, 'application/json; charset=utf-8', json.dumps(obj).encode())

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        hub = self.server.hub
        if path == '/api/info':
            return self._json(api_info())
        if path == '/api/live':
            if not ws.is_upgrade(self.headers):
                return self._send(HTTPStatus.BAD_REQUEST, 'text/plain', b'a WebSocket address')
            return self.live(hub)
        if path == '/api/localize.csv':
            return self._send(HTTPStatus.OK, 'text/csv; charset=utf-8', hub.localize_csv().encode(),
                              {'Content-Disposition': 'attachment; filename="localization_test.csv"'})
        gets = {'/api/state': hub.state, '/api/report': hub.report, '/api/map': hub.map_info,
                '/api/check': hub.check_summary, '/api/localize': hub.localize_summary}
        if path in gets:
            try:
                return self._json(gets[path]())
            except ApiError as e:
                return self._json({'ok': False, 'error': str(e)})
        if path in ('/', ''):
            path = '/index.html'
        full = os.path.normpath(os.path.join(STATIC, path.lstrip('/')))
        if not full.startswith(STATIC + os.sep) or not os.path.isfile(full):
            return self._send(HTTPStatus.NOT_FOUND, 'text/plain', b'not found')
        with open(full, 'rb') as f:
            data = f.read()
        self._send(HTTPStatus.OK, CONTENT_TYPES.get(os.path.splitext(full)[1], 'application/octet-stream'),
                   data)

    def do_POST(self):
        from marker_survey.core.marker_map import MarkerMapError
        from marker_survey.core.sheets import SheetError
        length = int(self.headers.get('Content-Length') or 0)
        if length > MAX_BODY:
            return self._json({'ok': False, 'error': 'the request is too large'},
                              HTTPStatus.REQUEST_ENTITY_TOO_LARGE)
        try:
            body = json.loads(self.rfile.read(length) or b'{}')
        except ValueError:
            return self._json({'ok': False, 'error': 'the request is not JSON'}, HTTPStatus.BAD_REQUEST)
        path = self.path.split('?', 1)[0]
        try:
            if path == '/api/sheets/download':
                name, ctype, data = api_download(body)
                return self._send(HTTPStatus.OK, ctype, data,
                                  {'Content-Disposition': f'attachment; filename="{name}"'})
            hub = self.server.hub
            fn = {'/api/survey': hub.survey_action, '/api/map': hub.map_action,
                  '/api/check': hub.check_action,
                  '/api/localize': hub.localize_action}.get(path) or ROUTES.get(path)
            if fn is None:
                return self._json({'ok': False, 'error': 'unknown address'}, HTTPStatus.NOT_FOUND)
            return self._json(fn(body))
        except (ApiError, SheetError, MarkerMapError, ValueError) as e:
            return self._json({'ok': False, 'error': str(e)}, HTTPStatus.OK)

    def live(self, hub):
        """WebSocket /api/live: the state as JSON twice a second, camera frames as JPEG."""
        ws.handshake(self)
        self.close_connection = True
        client = hub.add_client()
        wlock = threading.Lock()

        def send(data: bytes):
            with wlock:
                self.wfile.write(data)
                self.wfile.flush()

        def reader():
            try:
                while True:
                    op, payload = ws.read_message(self.rfile, send)
                    if op == ws.TEXT:
                        hub.on_message(client, payload.decode('utf-8', 'replace'))
            except (ws.Closed, OSError, ValueError):
                pass
            finally:
                try:
                    client.out.put(None, timeout=1.0)
                except queue.Full:
                    pass

        threading.Thread(target=reader, daemon=True).start()
        try:
            while True:
                try:
                    item = client.out.get(timeout=15.0)
                except queue.Empty:
                    send(ws.encode(ws.PING, b''))
                    continue
                if item is None:
                    break
                kind, data = item
                send(ws.encode(ws.TEXT, data.encode()) if kind == 'text' else ws.encode(ws.BINARY, data))
        except OSError:
            pass
        finally:
            hub.remove_client(client)
            try:
                send(ws.encode(ws.CLOSE, b''))
            except OSError:
                pass


def make_server(host='127.0.0.1', port=8090, hub: Hub = None) -> ThreadingHTTPServer:
    """The HTTP server; with a hub that has a robot, also the survey, check and map API
    and the live state pushed to every open page."""
    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    server.hub = hub or Hub(None)
    if server.hub.robot is not None:
        threading.Thread(target=server.hub.run_broadcast, daemon=True).start()
    return server


def main(argv=None):
    p = argparse.ArgumentParser(prog='marker_survey_web', description=__doc__.splitlines()[0])
    p.add_argument('--host', default='127.0.0.1',
                   help='address to listen on; 0.0.0.0 makes the page reachable from other devices')
    p.add_argument('--port', type=int, default=8090)
    p.add_argument('--open', action='store_true', help='open the page in a browser')
    a = p.parse_args(argv)
    server = make_server(a.host, a.port)
    shown = 'this computer\'s address' if a.host == '0.0.0.0' else a.host
    url = f'http://{"127.0.0.1" if a.host == "0.0.0.0" else a.host}:{a.port}/'
    print(f'marker_survey_web: open http://{shown}:{a.port}/ (Ctrl-C to stop)', flush=True)
    if a.open:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    server.server_close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
