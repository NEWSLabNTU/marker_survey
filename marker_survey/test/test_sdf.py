"""marker_map_to_sdf: models, a world, the textures' orientation, the BEGIN/END block."""
import os
import xml.etree.ElementTree as ET

import pytest

from marker_survey.core import marker_map as mm
from marker_survey.core import sdf
from marker_survey.core import sheets
from marker_survey.core.se3 import make_T, rot_x, rot_y, rot_z
from marker_survey.tools.map_to_sdf import main

MAP = """version: 1
frame_id: map
dictionary: tag36h11
marker_size: 0.40
mount: ceiling
markers:
  - {id: 0, x: 0.0, y: 0.0, z: 2.9, rotation_deg: 0.0}
  - {id: 7, x: 1.4, y: -0.5, z: 2.85, rotation_deg: 120.0, tilt_x_deg: 2.0}
  - {id: 12, x: 4.0, y: 1.0, z: 1.2, mount: wall, facing_deg: 180.0}
  - {id: 13, x: 2.0, y: 2.0, z: 0.0, mount: floor, rotation_deg: 30.0}
"""


@pytest.fixture
def site(tmp_path):
    p = tmp_path / 'marker_map.yaml'
    p.write_text(MAP)
    return str(p)


def test_pose_round_trip():
    R = rot_z(0.7) @ rot_y(-0.2) @ rot_x(3.0)
    roll, pitch, yaw = sdf.rpy_of(R)
    assert rot_z(yaw) @ rot_y(pitch) @ rot_x(roll) == pytest.approx(R, abs=1e-9)


def test_texture_is_the_printed_sheet():
    img, cells = sdf.marker_texture('tag36h11', 7, cell_px=4)
    assert cells == 8 and img.shape == (40, 40)
    assert (img[:4, :] == 255).all() and (img[:, -4:] == 255).all()       # one white cell around
    bits = sheets.marker_cells(sheets.family('tag36h11'), 7)
    inner = img[4:36:4, 4:36:4]
    assert ((inner == 0) == (bits == 1)).all()                             # not flipped, not turned


def test_world_parses_and_places_every_marker(site, tmp_path):
    out = tmp_path / 'w' / 'site.world'
    os.makedirs(out.parent)
    assert main([site, str(out), '--world']) == 0
    root = ET.parse(out).getroot()
    assert root.tag == 'sdf' and root.get('version') == '1.9'
    world = root.find('world')
    names = {m.get('name') for m in world.findall('model')}
    assert {'ground_plane', 'ceiling', 'marker_0', 'marker_7', 'marker_12', 'marker_13'} <= names
    plugins = {p.get('filename') for p in world.findall('plugin')}
    assert plugins == {'gz-sim-physics-system', 'gz-sim-sensors-system',
                       'gz-sim-scene-broadcaster-system', 'gz-sim-user-commands-system'}
    ceiling = next(m for m in world.findall('model') if m.get('name') == 'ceiling')
    assert float(ceiling.find('pose').text.split()[2]) == pytest.approx(2.9 + sdf.STANDOFF)
    the_map = mm.load(site)
    for m in the_map.markers:
        model = next(x for x in world.findall('model') if x.get('name') == f'marker_{m.id}')
        v = [float(t) for t in model.find('pose').text.split()]
        T = make_T(rot_z(v[5]) @ rot_y(v[4]) @ rot_x(v[3]), v[:3])
        assert T == pytest.approx(m.pose(), abs=1e-5)                       # the visual's frame
        plane = model.find('link/visual/geometry/plane')
        assert plane.find('normal').text == '0 0 1'                         # faces along z
        assert float(plane.find('size').text.split()[0]) == pytest.approx(0.5)   # 40 cm + margin
        tex = model.find('link/visual/material/pbr/metal/albedo_map').text
        assert tex == f'marker_{m.id}.png' and (out.parent / tex).is_file()
    text = out.read_text()
    assert sdf.BEGIN in text and sdf.END in text


def test_ceiling_height_given(site, tmp_path):
    out = tmp_path / 'site.world'
    assert main([site, str(out), '--world', '--ceiling', '3.1']) == 0
    world = ET.parse(out).getroot().find('world')
    ceiling = next(m for m in world.findall('model') if m.get('name') == 'ceiling')
    assert float(ceiling.find('pose').text.split()[2]) == pytest.approx(3.1)


def test_ceiling_never_hides_a_marker(site, tmp_path):
    # Measured 2.9 m, but marker 0 was surveyed at 2.9: the plane goes behind it.
    out = tmp_path / 'site.world'
    assert main([site, str(out), '--world', '--ceiling', '2.88']) == 0
    world = ET.parse(out).getroot().find('world')
    ceiling = next(m for m in world.findall('model') if m.get('name') == 'ceiling')
    assert float(ceiling.find('pose').text.split()[2]) == pytest.approx(2.9 + sdf.STANDOFF)


def test_models_only_and_replace_between(site, tmp_path):
    out = tmp_path / 'markers.sdf'
    assert main([site, str(out)]) == 0
    snippet = out.read_text()
    assert '<sdf' not in snippet and snippet.count('<model ') == 4
    world = tmp_path / 'robot.world'
    world.write_text('<sdf version="1.9"><world name="w">\n'
                     '  <!-- BEGIN ceiling markers: keep this text -->\n'
                     '  <model name="old"/>\n'
                     '  <!-- END ceiling markers -->\n'
                     '  <model name="robot"/>\n</world></sdf>\n')
    assert main([site, '--replace-between', str(world)]) == 0
    text = world.read_text()
    assert '<!-- BEGIN ceiling markers: keep this text -->' in text
    assert 'name="old"' not in text and 'name="robot"' in text
    names = [m.get('name') for m in ET.fromstring(text).find('world').findall('model')]
    assert names == ['marker_0', 'marker_7', 'marker_12', 'marker_13', 'robot']
    with pytest.raises(ValueError, match='BEGIN'):
        sdf.replace_between('<sdf/>', '', sdf.BEGIN, sdf.END)


def test_an_id_outside_the_dictionary_is_refused(tmp_path):
    p = tmp_path / 'm.yaml'
    p.write_text(MAP.replace('id: 13,', 'id: 900,'))
    assert main([str(p), str(tmp_path / 'x.sdf')]) == 2
    assert sdf.check_ids([0, 900], 'tag36h11') == {900: 'not in tag36h11 (0 to 586)'}
