"""The map in use: versions, diff, undo (mapstore.py)."""
import os

import pytest

from marker_survey_web.mapstore import MapStore, MapStoreError

MAP = """version: 1
frame_id: map
dictionary: tag36h11
marker_size: 0.45
mount: ceiling
markers:
  - {{id: 0, x: {x0}, y: 0.000, z: 3.000, rotation_deg: 0.0}}
  - {{id: {id1}, x: 2.000, y: 0.000, z: 3.000, rotation_deg: 0.0}}
"""


def write(path, x0=0.0, id1=1):
    path.write_text(MAP.format(x0=f'{x0:.3f}', id1=id1))
    return str(path)


def test_first_use_then_versions_and_undo(tmp_path):
    site = tmp_path / 'site'
    store = MapStore(str(site / 'marker_map.yaml'))
    a = write(tmp_path / 'a.yaml')
    b = write(tmp_path / 'b.yaml', x0=0.05, id1=2)
    assert not store.info()['exists'] and not store.info()['can_undo']
    d = store.diff(a)
    assert d['first'] and d['markers'] == 2
    assert store.use(a)['kept'] is None
    d = store.diff(b)
    assert d['added'] == [2] and d['removed'] == [1] and d['moved'] == [{'id': 0, 'distance_m': 0.05}]
    res = store.use(b)
    assert res['kept'] and os.path.isfile(os.path.join(store.versions_dir, res['kept']))
    info = store.info()
    assert info['ids'] == [0, 2] and info['can_undo'] and len(info['versions']) == 1
    res = store.undo()
    assert store.info()['ids'] == [0, 1]
    assert res['kept'].endswith('.undone.yaml')
    assert not store.info()['can_undo']             # the undone file is kept, not in the undo order
    with pytest.raises(MapStoreError, match='nothing to undo'):
        store.undo()


def test_a_broken_map_is_refused_and_nothing_changes(tmp_path):
    store = MapStore(str(tmp_path / 'marker_map.yaml'))
    store.use(write(tmp_path / 'a.yaml'))
    bad = tmp_path / 'bad.yaml'
    bad.write_text('version: 1\nmarkers: [}\n')
    with pytest.raises(MapStoreError):
        store.use(str(bad))
    with pytest.raises(MapStoreError):
        store.use(str(tmp_path / 'missing.yaml'))
    assert store.info()['ids'] == [0, 1] and not store.versions()
    with pytest.raises(MapStoreError, match='already'):
        store.use(store.path)
