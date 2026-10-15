"""The web API against a server on a free port: plan, download, size, identify."""
import base64
import io
import json
import threading
import urllib.request
import zipfile

import numpy as np
import pytest

from marker_survey_web.server import make_server


@pytest.fixture(scope='module')
def url():
    server = make_server('127.0.0.1', 0)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    yield f'http://127.0.0.1:{server.server_address[1]}'
    server.shutdown()
    server.server_close()


def post(url, path, body):
    req = urllib.request.Request(url + path, json.dumps(body).encode(),
                                 {'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=60) as r:
        return r.headers, r.read()


def post_json(url, path, body):
    return json.loads(post(url, path, body)[1])


JOB = {'size': '45cm', 'count': 5}


def test_page_and_info(url):
    with urllib.request.urlopen(url + '/', timeout=10) as r:
        assert b'Print markers' in r.read()
    with urllib.request.urlopen(url + '/app.js', timeout=10) as r:
        assert r.headers['Content-Type'].startswith('text/javascript')
    with urllib.request.urlopen(url + '/api/info', timeout=10) as r:
        info = json.loads(r.read())
    assert info['default_dictionary'] == 'tag36h11' and 'A1' in info['papers']


def test_static_files_stay_inside(url):
    with pytest.raises(urllib.error.HTTPError):
        urllib.request.urlopen(url + '/../server.py', timeout=10)


def test_plan(url):
    res = post_json(url, '/api/sheets/plan', JOB)
    assert res['ok'] and res['summary']['paper'] == 'A1' and len(res['summary']['ids']) == 5
    assert res['previews'][0]['svg'].startswith('<svg')
    bad = post_json(url, '/api/sheets/plan', {'size': '45cm', 'count': 5, 'paper': 'A2'})
    assert not bad['ok'] and 'A1 fits' in bad['error']
    assert not post_json(url, '/api/sheets/plan', {'size': '45', 'count': 5})['ok']


def test_site_map_ids_are_skipped(url):
    site = 'version: 1\ndictionary: tag36h11\nmarker_size: 0.45\nmount: ceiling\nmarkers:\n' \
           '  - {id: 0, x: 0, y: 0, z: 3, rotation_deg: 0}\n'
    res = post_json(url, '/api/sheets/plan', dict(JOB, site_map=site))
    assert 0 not in res['summary']['ids']


def test_download_pdf_zip_csv(url):
    headers, pdf = post(url, '/api/sheets/download', JOB)
    assert headers['Content-Type'] == 'application/pdf' and pdf.startswith(b'%PDF')
    assert 'markers.pdf' in headers['Content-Disposition']
    headers, data = post(url, '/api/sheets/download', dict(JOB, split=True))
    names = zipfile.ZipFile(io.BytesIO(data)).namelist()
    assert headers['Content-Type'] == 'application/zip' and 'contact_sheet.pdf' in names
    assert sum(n.endswith('.pdf') for n in names) == 6
    headers, csv = post(url, '/api/sheets/download', dict(JOB, format='csv'))
    assert csv.decode().startswith('id,dictionary') and len(csv.decode().strip().splitlines()) == 6


def test_size(url):
    res = post_json(url, '/api/size', {'ceiling': 2.9, 'hfov': 70})
    assert res['ok'] and res['size_cm'] == 45.0


def test_identify(url):
    import cv2
    from marker_survey.core import sheets
    cells = sheets.marker_cells(sheets.family('tag36h11'), 240)
    img = np.full((600, 600), 255, np.uint8)
    img[150:450, 150:450] = cv2.resize((1 - cells).astype(np.uint8) * 255, (300, 300),
                                       interpolation=cv2.INTER_NEAREST)
    ok, jpg = cv2.imencode('.jpg', img)
    url_data = 'data:image/jpeg;base64,' + base64.b64encode(jpg.tobytes()).decode()
    res = post_json(url, '/api/identify', {'image': url_data})
    assert res['ok'] and res['ids'] == [240] and res['dictionary'] == 'tag36h11'
    assert res['image'].startswith('data:image/jpeg;base64,')
    assert not post_json(url, '/api/identify', {'image': 'data:image/png;base64,AAAA'})['ok']
