import json
import math
import os

import numpy as np
import pytest
import yaml

from marker_survey.core import marker_map as mm
from marker_survey.tools import map_check

HERE = os.path.dirname(__file__)
SCHEMA = os.path.join(HERE, '..', 'schema', 'marker_map.schema.json')
EXAMPLES = os.path.join(HERE, '..', '..', 'examples')
SITES = {name: os.path.join(EXAMPLES, example, 'sites', name) for example, name in (
    ('wall_course', 'test_course'), ('ceiling', 'office'), ('ceiling', 'office_uneven'))}

CEILING = """\
# Ceiling markers of the warehouse.
version: 1
frame_id: map                 # the frame the robot's map uses
dictionary: tag36h11          # printed family
marker_size: 0.45             # [m] edge of the black square, measured
mount: ceiling                # default for every marker below
markers:
  - {id: 0,  x: -4.000, y:  1.500, z: 3.012, rotation_deg:   0.0}
  - {id: 1,  x: -1.500, y:  1.500, z: 2.998, rotation_deg:  45.8}
  # near the loading door
  - {id: 7,  x:  3.000, y: -2.000, z: 3.204, rotation_deg:  91.7, tilt_x_deg: 1.2}   # under the duct
  - {id: 12, x:  0.300, y:  3.000, z: 3.005, rotation_deg:  17.2, size: 0.30}
"""


def rot(mount, **angles):
    return mm.mount_rotation(mount, angles)


# ------------------------------------------------------------------ mounts

def test_a_wall_board_at_zero_stands_upright_facing_x_with_its_top_up():
    R = rot('wall')
    assert np.allclose(R[:, 2], [1, 0, 0])      # normal: facing +x
    assert np.allclose(R[:, 1], [0, 0, 1])      # top up
    assert np.allclose(R[:, 0], [0, 1, 0])      # image x: the viewer's right, looking at it from +x


def test_a_ceiling_marker_at_zero_faces_down_with_its_image_x_along_x():
    R = rot('ceiling')
    assert np.allclose(R[:, 2], [0, 0, -1])
    assert np.allclose(R[:, 0], [1, 0, 0])
    assert np.allclose(R[:, 1], [0, -1, 0])


def test_a_floor_marker_at_zero_faces_up_with_its_image_x_along_x():
    R = rot('floor')
    assert np.allclose(R, np.eye(3))


def test_the_wall_angles_mean_what_their_names_say():
    assert np.allclose(rot('wall', facing_deg=90)[:, 2], [0, 1, 0])
    lean = rot('wall', lean_deg=10)                  # top leaning back: the normal tilts up
    assert lean[2, 2] == pytest.approx(math.sin(math.radians(10)))
    assert lean[0, 1] == pytest.approx(-math.sin(math.radians(10)))   # top moves away (-x)
    turn = rot('wall', turn_deg=90)                  # counter-clockwise seen from the front:
    assert np.allclose(turn[:, 1], [0, -1, 0])       # the top goes to the viewer's left
    assert np.allclose(turn[:, 2], [1, 0, 0])


def test_the_ceiling_angles_mean_what_their_names_say():
    assert np.allclose(rot('ceiling', rotation_deg=90)[:, 0], [0, 1, 0])
    assert np.allclose(rot('ceiling', rotation_deg=90)[:, 2], [0, 0, -1])
    t = rot('ceiling', tilt_x_deg=5)
    assert np.allclose(t[:, 0], [1, 0, 0])           # tilting about x keeps the x axis
    assert mm.tilt_from_mount('ceiling', t) == pytest.approx(5.0)
    assert mm.tilt_from_mount('ceiling', rot('ceiling', rotation_deg=70, tilt_y_deg=-3)) == \
        pytest.approx(3.0)


@pytest.mark.parametrize('mount', mm.MOUNTS)
def test_angles_and_rotation_are_inverse(mount):
    rng = np.random.default_rng(5)
    for _ in range(50):
        a = dict(zip(mm.ANGLES[mount], [rng.uniform(-179, 179), rng.uniform(-40, 40),
                                        rng.uniform(-80, 80)]))
        back = mm.angles_from_rotation(mount, rot(mount, **a))
        assert np.allclose(rot(mount, **back), rot(mount, **a), atol=1e-12)
        for k in a:
            assert back[k] == pytest.approx(a[k], abs=1e-9)


def test_the_mount_follows_the_normal():
    assert mm.mount_from_normal([0, 0, -1]) == 'ceiling'
    assert mm.mount_from_normal([0.1, 0, 0.99]) == 'floor'
    assert mm.mount_from_normal([0.9, 0.3, 0.1]) == 'wall'


# ------------------------------------------------------------------ reading and writing

def test_the_format_example_loads():
    m = mm.loads(CEILING)
    assert (m.version, m.frame_id, m.dictionary, m.marker_size, m.mount) == \
        (1, 'map', 'tag36h11', 0.45, 'ceiling')
    assert m.ids() == [0, 1, 7, 12]
    assert m.size_of(12) == 0.30 and m.size_of(0) == 0.45
    assert m.get(7).angles == {'rotation_deg': 91.7, 'tilt_x_deg': 1.2, 'tilt_y_deg': 0.0}
    assert m.get(7).line == 11
    corners = m.corners_of(0)
    assert np.allclose(corners[:, 2], 3.012)
    assert np.allclose(np.linalg.norm(corners[0] - corners[1]), 0.45)


def test_comments_and_values_survive_a_round_trip():
    m = mm.loads(CEILING)
    assert m.header == ['Ceiling markers of the warehouse.']
    assert m.key_comments['frame_id'] == "the frame the robot's map uses"
    assert m.get(7).comment == 'under the duct'
    assert m.get(7).comment_above == ['near the loading door']
    text = mm.dumps(m)
    assert text == CEILING                        # this file is already in the writer's layout
    again = mm.loads(text)
    for a, b in zip(m.markers, again.markers):
        assert np.allclose(a.pose(), b.pose(), atol=1e-12)
        assert (a.comment, a.comment_above, a.size) == (b.comment, b.comment_above, b.size)


def test_the_writer_aligns_columns_and_keeps_full_precision():
    m = mm.MarkerMap('DICT_5X5_1000', 0.38, frame_id='map', mount='wall', markers=[
        mm.MapMarker(3, 1.0, -12.25, 0.25, 'wall', {'facing_deg': 180.0}),
        mm.MapMarker(14, 0.123456789, 2.0, 0.25, 'wall', {'facing_deg': -45.0, 'lean_deg': 2.1}),
        mm.MapMarker(5, 0.0, 0.0, 3.0, 'ceiling', {'rotation_deg': 12.5}, size=0.3)])
    lines = [ln for ln in mm.dumps(m).splitlines() if ln.startswith('  - {')]
    assert lines[0].index('y:') == lines[1].index('y:') == lines[2].index('y:')
    assert 'x: 0.123457' in lines[1] and 'lean_deg: 2.1' in lines[1]
    assert 'mount: ceiling' in lines[2] and 'size: 0.30' in lines[2]
    assert 'rotation_deg:  12.5' in lines[2]
    back = mm.loads(mm.dumps(m))
    assert back.get(5).mount == 'ceiling' and back.size_of(5) == 0.3


@pytest.mark.parametrize('site', ['test_course', 'office', 'office_uneven'])
def test_the_example_sites_are_in_the_writers_layout(site):
    path = os.path.join(SITES[site], 'marker_map.yaml')
    with open(path) as f:
        text = f.read()
    assert mm.dumps(mm.load(path)) == text


# ------------------------------------------------------------------ errors

@pytest.mark.parametrize('change, message', [
    (('rotation_deg:  45.8}', 'rotation_deg:  45.8, facing_deg: 3}'),
     ':9: marker 1: facing_deg is an angle of mount wall, not ceiling (use rotation_deg'),
    (('x: -1.500, ', ''), ':9: marker 1: x missing'),
    (('y:  1.500, z: 2.998', 'y:  1.500, z: high'), ':9: marker 1: z: expected a number'),
    (('{id: 12,', '{id: 7,'), ':12: marker 7: id appears twice (first on line 11)'),
    (('size: 0.30}', 'sise: 0.30}'), ":12: marker 12: unknown field 'sise'"),
    (('size: 0.30}', 'size: -1}'), ':12: marker 12: size: expected a positive number'),
    (('mount: ceiling ', 'mount: roof    '), ":6: mount: 'roof' is not one of wall, ceiling, floor"),
    (('version: 1', 'version: 2'), ':2: version: 2 is not supported'),
    (('dictionary: tag36h11', 'dictionary: tag99'), ":4: dictionary: unknown dictionary 'tag99'"),
    (('marker_size: 0.45', 'marker_size: 0'), ':5: marker_size: expected a positive number'),
    (('frame_id: map', 'frame: map'), ":3: unknown key 'frame'"),
    (('{id: 0,  x: -4.000,', '{id: 0,  x: -4.000, x: 1,'), "line 8: 'x' appears twice"),
    (('{id: 0,  x', '{id: 0,  x: [1'), ':8: not valid YAML'),
])
def test_errors_name_the_line_and_the_field(change, message):
    text = CEILING.replace(*change)
    assert text != CEILING
    with pytest.raises(mm.MarkerMapError) as e:
        mm.loads(text, source='site.yaml')
    assert message in str(e.value), str(e.value)
    assert str(e.value).startswith('site.yaml')


def test_a_marker_without_any_mount_is_refused():
    text = CEILING.replace('mount: ceiling                # default for every marker below\n', '')
    with pytest.raises(mm.MarkerMapError, match='marker 0: mount missing'):
        mm.loads(text)


def test_required_keys():
    with pytest.raises(mm.MarkerMapError, match='marker_size missing'):
        mm.loads('version: 1\ndictionary: tag36h11\nmarkers: []\n')


# ------------------------------------------------------------------ schema and check tool

def test_the_schema_accepts_the_examples_and_refuses_a_wrong_angle():
    jsonschema = pytest.importorskip('jsonschema')
    with open(SCHEMA) as f:
        schema = json.load(f)
    for site in ('test_course', 'office', 'office_uneven'):
        with open(os.path.join(SITES[site], 'marker_map.yaml')) as f:
            jsonschema.validate(yaml.safe_load(f), schema)
    jsonschema.validate(yaml.safe_load(CEILING), schema)
    bad = yaml.safe_load(CEILING.replace('rotation_deg:  45.8}', 'facing_deg: 45.8}'))
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(bad, schema)


def test_the_check_tool(tmp_path, capsys):
    p = tmp_path / 'm.yaml'
    p.write_text(CEILING)
    png = tmp_path / 'top.png'
    assert map_check.main([str(p), '--png', str(png)]) == 0
    out = capsys.readouterr().out
    assert '4 markers' in out and 'OK' in out and png.exists()
    p.write_text(CEILING.replace('x: -1.500, ', ''))
    assert map_check.main([str(p)]) == 1
    assert 'marker 1: x missing' in capsys.readouterr().out
