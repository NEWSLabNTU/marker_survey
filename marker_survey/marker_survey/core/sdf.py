"""A marker map as Gazebo (SDF 1.9, Gazebo Harmonic) models or a whole world. No ROS.

Every marker becomes a static model: one flat visual (a `plane` geometry) of
the printed sheet, the black square of the map's size plus a white margin of
one cell on each side as marker_sheets prints it, at the marker's pose, its
face towards the marker's z axis (the side a camera sees it from). The
texture is a PNG of the marker drawn from the dictionary's own bits, written
next to the SDF and named in it by a relative path (Gazebo resolves it
against the SDF file's directory).

The plane's texture: Gazebo's plane mesh has its texture's u along the plane's
x axis and v = 0 (the image's top row) at +y, seen from the side its normal
points to; a marker's frame is OpenCV's (x to the right of the printed image,
y to its top, z out of the face), so the visual's frame is the marker's frame
and the image needs no flip.

A world (world_sdf) adds a ground plane, a ceiling plane at the ceiling height
(given, or a few millimetres above the highest ceiling marker), light from
below for markers that face down and from above for the floor, and the system
plugins Gazebo Harmonic needs to simulate and render cameras (physics,
sensors with ogre2, scene broadcaster, user commands).
"""

import os
from typing import Dict, List, Optional, Sequence
from xml.sax.saxutils import escape

import numpy as np

from marker_survey.core import marker_map as mm

BEGIN = '<!-- BEGIN ceiling markers'
END = '<!-- END ceiling markers -->'
CELL_PX = 32              # texture pixels per cell
STANDOFF = 0.002          # [m] the ceiling plane lies this far behind the highest ceiling marker


def marker_texture(dictionary: str, mid: int, cell_px: int = CELL_PX, margin_cells: int = 1):
    """The printed sheet of one marker as a grey image: black square plus a white margin
    of `margin_cells` cells, `cell_px` pixels per cell (nearest neighbour, sharp)."""
    from marker_survey.core import sheets
    fam = sheets.family(dictionary)
    cells = sheets.marker_cells(fam, mid)                  # 1 = black, border included
    n = cells.shape[0] + 2 * margin_cells
    img = np.ones((n, n), np.uint8)
    img[margin_cells:margin_cells + cells.shape[0], margin_cells:margin_cells + cells.shape[0]] = 1 - cells
    img = np.kron(img, np.ones((cell_px, cell_px), np.uint8)) * 255
    return img, fam.cells


def write_png(path: str, img) -> None:
    import cv2
    if not cv2.imwrite(path, img):
        raise OSError(f'could not write {path}')


def rpy_of(R) -> List[float]:
    """SDF's roll, pitch, yaw (fixed axes: R = Rz(yaw) Ry(pitch) Rx(roll))."""
    from scipy.spatial.transform import Rotation
    return [float(v) for v in Rotation.from_matrix(np.asarray(R, float)).as_euler('xyz')]


def _f(v: float) -> str:
    s = f'{v:.6f}'.rstrip('0').rstrip('.')
    return '0' if s in ('-0', '') else s


def pose_text(T) -> str:
    return ' '.join(_f(v) for v in list(T[:3, 3]) + rpy_of(T[:3, :3]))


def texture_name(mid: int, prefix: str = 'marker') -> str:
    return f'{prefix}_{mid}.png'


def marker_model(mid: int, T, size: float, cells: int, texture: str, prefix: str = 'marker',
                 margin_cells: int = 1, indent: str = '    ') -> str:
    """One marker as a static model: a plane of the sheet's size facing along the
    marker's z axis, textured with `texture` (a path relative to the SDF)."""
    sheet = size * (cells + 2 * margin_cells) / cells
    i = indent
    return '\n'.join([
        f'{i}<model name="{prefix}_{mid}">',
        f'{i}  <static>true</static>',
        f'{i}  <pose>{pose_text(T)}</pose>',
        f'{i}  <link name="link">',
        f'{i}    <visual name="sheet">',
        f'{i}      <cast_shadows>false</cast_shadows>',
        f'{i}      <geometry><plane><normal>0 0 1</normal>'
        f'<size>{_f(sheet)} {_f(sheet)}</size></plane></geometry>',
        f'{i}      <material>',
        f'{i}        <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>',
        f'{i}        <pbr><metal><albedo_map>{escape(texture)}</albedo_map><metalness>0</metalness>'
        '<roughness>1</roughness></metal></pbr>',
        f'{i}      </material>',
        f'{i}    </visual>',
        f'{i}  </link>',
        f'{i}</model>'])


def models_sdf(the_map: mm.MarkerMap, out_dir: str, prefix: str = 'marker', cell_px: int = CELL_PX,
               indent: str = '    ', write_textures: bool = True) -> str:
    """The SDF models of every marker (no <sdf> or <world> around them); the textures
    are written to out_dir."""
    blocks = []
    for m in the_map.markers:
        tex = texture_name(m.id, prefix)
        img, cells = marker_texture(the_map.dictionary, m.id, cell_px)
        if write_textures:
            os.makedirs(out_dir or '.', exist_ok=True)
            write_png(os.path.join(out_dir or '.', tex), img)
        blocks.append(f'{indent}<!-- {the_map.dictionary} id {m.id}, {the_map.size_of(m.id) * 100:g} cm '
                      f'black square, {m.mount} -->')
        blocks.append(marker_model(m.id, m.pose(), the_map.size_of(m.id), cells, tex, prefix,
                                   indent=indent))
    return '\n'.join(blocks)


def ceiling_height(the_map: mm.MarkerMap) -> Optional[float]:
    """Where the ceiling plane goes: just behind the highest ceiling marker; None if the
    map has no ceiling marker."""
    zs = [m.z for m in the_map.markers if m.mount == 'ceiling']
    return max(zs) + STANDOFF if zs else None


def extent(the_map: mm.MarkerMap, margin: float = 3.0):
    """(centre x, centre y, size x, size y) of the markers' area plus a margin."""
    if not the_map.markers:
        return 0.0, 0.0, 2 * margin, 2 * margin
    P = np.array([[m.x, m.y] for m in the_map.markers])
    lo, hi = P.min(axis=0) - margin, P.max(axis=0) + margin
    c, s = (lo + hi) / 2, hi - lo
    return float(c[0]), float(c[1]), float(s[0]), float(s[1])


PLUGINS = [
    ('gz-sim-physics-system', 'gz::sim::systems::Physics', ''),
    ('gz-sim-sensors-system', 'gz::sim::systems::Sensors', '<render_engine>ogre2</render_engine>'),
    ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster', ''),
    ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands', ''),
]


def world_sdf(the_map: mm.MarkerMap, models: str, ceiling: Optional[float] = None,
              world_name: str = 'markers', begin: str = BEGIN, end: str = END) -> str:
    """A whole world around the models: ground, ceiling plane, lights, Harmonic's system
    plugins. The models sit between the begin and end comment lines, so that
    --replace-between can update them later."""
    cx, cy, sx, sy = extent(the_map)
    if ceiling is None:
        ceiling = ceiling_height(the_map)
    out = ['<?xml version="1.0"?>',
           '<sdf version="1.9">',
           f'  <world name="{escape(world_name)}">',
           f'    <!-- Generated by marker_map_to_sdf from a marker map ({len(the_map.markers)} '
           f'{escape(the_map.dictionary)} markers, frame {escape(the_map.frame_id)}): '
           'the world frame is the map frame. -->',
           '    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size>'
           '<real_time_factor>1.0</real_time_factor></physics>']
    for fname, cls, inner in PLUGINS:
        out.append(f'    <plugin filename="{fname}" name="{cls}">{inner}</plugin>')
    out += ['    <gravity>0 0 -9.8</gravity>',
            '    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.7 0.7 0.7 1</background>'
            '<shadows>false</shadows><grid>false</grid></scene>',
            '    <!-- light from above for the floor, from below for markers that face down -->',
            '    <light type="directional" name="from_above"><cast_shadows>false</cast_shadows>'
            '<diffuse>0.8 0.8 0.8 1</diffuse><specular>0.1 0.1 0.1 1</specular>'
            '<direction>0.2 0.1 -1</direction></light>',
            '    <light type="directional" name="from_below"><cast_shadows>false</cast_shadows>'
            '<diffuse>0.8 0.8 0.8 1</diffuse><specular>0 0 0 1</specular>'
            '<direction>-0.1 0.2 1</direction></light>',
            '    <model name="ground_plane"><static>true</static><link name="link">',
            '      <collision name="collision"><geometry><plane><normal>0 0 1</normal>'
            f'<size>{_f(sx)} {_f(sy)}</size></plane></geometry></collision>',
            '      <visual name="visual"><geometry><plane><normal>0 0 1</normal>'
            f'<size>{_f(sx)} {_f(sy)}</size></plane></geometry>',
            '        <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse>'
            '</material></visual>',
            f'    </link><pose>{_f(cx)} {_f(cy)} 0 0 0 0</pose></model>']
    if ceiling is not None:
        out += ['    <model name="ceiling"><static>true</static><link name="link">',
                '      <visual name="visual"><cast_shadows>false</cast_shadows>'
                '<geometry><plane><normal>0 0 -1</normal>'
                f'<size>{_f(sx)} {_f(sy)}</size></plane></geometry>',
                '        <material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse>'
                '</material></visual>',
                f'    </link><pose>{_f(cx)} {_f(cy)} {_f(ceiling)} 0 0 0</pose></model>']
    out += [f'    {begin} (marker_map_to_sdf) -->' if not begin.rstrip().endswith('-->') else f'    {begin}',
            models,
            f'    {end}',
            '  </world>',
            '</sdf>', '']
    return '\n'.join(out)


def replace_between(text: str, models: str, begin: str = BEGIN, end: str = END) -> str:
    """`text` with the lines strictly between the line holding `begin` and the line
    holding `end` replaced by `models`; both lines are kept as they are."""
    lines = text.split('\n')
    b = [i for i, ln in enumerate(lines) if begin in ln]
    e = [i for i, ln in enumerate(lines) if end in ln]
    if len(b) != 1 or len(e) != 1 or e[0] <= b[0]:
        raise ValueError(f'expected one line with {begin!r} and, after it, one with {end!r} '
                         f'(found {len(b)} and {len(e)})')
    return '\n'.join(lines[:b[0] + 1] + [models] + lines[e[0]:])


def check_ids(ids: Sequence[int], dictionary: str) -> Dict[int, str]:
    """Ids the dictionary does not have: {id: why}."""
    from marker_survey.core import sheets
    fam = sheets.family(dictionary)
    return {i: f'not in {dictionary} (0 to {fam.count - 1})' for i in ids if not 0 <= i < fam.count}
