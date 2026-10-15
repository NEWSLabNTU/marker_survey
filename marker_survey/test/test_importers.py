"""The two existing sites convert into marker_map.yaml and back unchanged."""
import os

import numpy as np
import pytest
import yaml

from marker_survey.core import importers
from marker_survey.core import marker_map as mm
from marker_survey.tools import map_import

FIX = os.path.join(os.path.dirname(__file__), 'fixtures')
SITES = {
    # fixture, kind, importer
    'test_course': ('test_course_source.yaml', 'course',
                    lambda t, **kw: importers.from_course(t, dictionary='DICT_5X5_1000', **kw)),
    'office': ('office_source.yaml', 'ceiling', importers.from_ceiling_list),
}


def source_text(name):
    with open(os.path.join(FIX, SITES[name][0])) as f:
        return f.read()


def source_comments(text, header=True):
    out = []
    lines = text.splitlines()
    first = next(i for i, ln in enumerate(lines) if ln.strip() and not ln.lstrip().startswith('#'))
    for ln in lines if header else lines[first:]:
        if '#' in ln:
            c = ln[ln.index('#') + 1:]
            out.append((c[1:] if c.startswith(' ') else c).rstrip())
    return [c for c in out if c]


@pytest.mark.parametrize('keep_header', [False, True])
@pytest.mark.parametrize('name', sorted(SITES))
def test_import_write_and_load_back_give_the_same_poses_and_comments(name, keep_header, tmp_path):
    text = source_text(name)
    _, kind, convert = SITES[name]
    want = importers.source_poses(kind, text)
    path = tmp_path / 'marker_map.yaml'
    mm.write(convert(text, keep_header=keep_header), str(path))
    back = mm.load(str(path))
    assert sorted(back.ids()) == sorted(want)
    for mid, T in want.items():
        assert np.abs(back.pose_of(mid) - T).max() < map_import.POSE_TOLERANCE, mid
    written = path.read_text()
    for c in source_comments(text, header=keep_header):
        assert c in written, c
    header = set(source_comments(text)) - set(source_comments(text, header=False))
    assert header
    for c in header:
        assert (c in written) == keep_header, c
    assert ('Imported from' in written) != keep_header
    again = mm.dumps(back)
    assert again == written            # and the written file is stable


def test_the_wall_course_keeps_heights_and_tilts():
    m = SITES['test_course'][2](source_text('test_course'))
    assert m.mount == 'wall' and m.marker_size == 0.25 and m.dictionary == 'DICT_5X5_1000'
    assert m.get(0).z == 0.25 and m.get(9).z == 0.30
    assert m.get(9).angles['lean_deg'] == 2.01       # 0.035 rad, 2.005 deg
    assert m.get(10).comment == 'by the start'
    assert m.get(0).comment_above == ['end walls, seen in the turns']


def test_the_office_is_a_ceiling_map():
    m = importers.from_ceiling_list(source_text('office'))
    assert (m.mount, m.dictionary, m.marker_size) == ('ceiling', 'tag36h11', 0.45)
    assert m.get(1).angles['rotation_deg'] == 45.84     # 0.8 rad
    assert m.get(3).comment == 'above the printer'
    assert m.get(8).comment_above == ['by the door']


def test_imported_angles_are_rounded_within_the_source_precision(tmp_path):
    assert [importers._deg(r) for r in (0.5236, 2.3562, 3.1416, -1.0472, -0.7854, 0.0)] == \
        [30.0, 135.0, 180.0, -60.0, -45.0, 0.0]
    assert importers._deg(-0.02) == -1.15            # -1.1459 deg, two decimals at most
    assert importers._deg(0.123456789) == pytest.approx(7.0735530, abs=1e-6)   # precise: kept
    path = tmp_path / 'marker_map.yaml'
    mm.write(SITES['test_course'][2](source_text('test_course')), str(path))
    text = path.read_text()
    assert 'facing_deg: 135.0' in text and '135.000' not in text and '120.0' in text


def test_a_course_without_a_dictionary_needs_one():
    with pytest.raises(mm.MarkerMapError, match='dictionary'):
        importers.from_course(source_text('test_course'))


def test_the_conventions_agree_with_the_mounts():
    rng = np.random.default_rng(2)
    for _ in range(20):
        y, p, r = rng.uniform(-3, 3), rng.uniform(-0.3, 0.3), rng.uniform(-1.5, 1.5)
        R = mm.mount_rotation('wall', {'facing_deg': np.degrees(y), 'lean_deg': np.degrees(p),
                                       'turn_deg': np.degrees(r)})
        assert np.allclose(R, importers.course_marker_axes(y, p, r), atol=1e-12)
        R = mm.mount_rotation('ceiling', {'rotation_deg': np.degrees(y)})
        assert np.allclose(R, importers.ceiling_marker_axes(y), atol=1e-12)


def test_the_import_tool(tmp_path, capsys):
    out = tmp_path / 'office.yaml'
    assert map_import.main(['ceiling-list', os.path.join(FIX, 'office_source.yaml'), str(out)]) == 0
    assert 'largest difference' in capsys.readouterr().out
    assert yaml.safe_load(out.read_text())['mount'] == 'ceiling'
    assert out.read_text().startswith('# Imported from office_source.yaml by marker_map_import.')
    assert 'Fixture for the importer' not in out.read_text()
    assert map_import.main(['ceiling-list', os.path.join(FIX, 'office_source.yaml'), str(out),
                            '--keep-header']) == 0
    assert 'Fixture for the importer' in out.read_text()
