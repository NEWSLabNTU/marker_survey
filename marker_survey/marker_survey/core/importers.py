"""One-off migration of existing sites into marker_map.yaml. No ROS.

Two source formats, both lists of markers with angles in radians:

- course: a course description of wall boards,

      marker_size: 0.25          # [m]
      marker_height: 0.25        # [m] default z
      dictionary: DICT_5X5_1000  # optional
      markers:
        - {id: 0, x: 10.0, y: 0.5, yaw: 3.1416}              # z, pitch, roll optional
        - {id: 4, x: 2.0, y: -2.0, z: 0.30, yaw: 2.3562, pitch: 0.05, roll: -0.02}

  yaw is the direction the board faces, pitch its top leaning back, roll
  the board turned in its own plane, counter-clockwise from the front
  (course_marker_axes() below).

- ceiling list: ceiling markers,

      family: tag36h11
      marker_size: 0.45
      markers:
        - {id: 0, x: -4.0, y: 1.5, z: 3.0, yaw: 0.0}

  yaw rotates the marker about the vertical; at yaw 0 its image x axis
  points along +x and it faces down (ceiling_marker_axes() below).

Angles are converted to degrees and rounded within the source's precision
(at most ANGLE_ROUNDING_DEG): 0.5236 rad becomes 30.0, not 30.00007.

Comments next to markers are carried over: comment lines above a marker and
the comment at the end of a marker's line. The file's header and footer
comments (often about the source's own use: routes, tape marks) are replaced
by one line naming the source, unless keep_header is set. Not used at run
time.
"""

import math
import os
from typing import Optional

import numpy as np
import yaml

from marker_survey.core import marker_map as mm


def course_marker_axes(yaw: float, pitch: float = 0.0, roll: float = 0.0) -> np.ndarray:
    """The course format's convention, as its own code wrote it: the marker's x (viewer's
    right), y (up) and z (facing) axes as columns; upright at `yaw`, leaned back by
    `pitch` about its x axis, then turned by `roll` about its normal."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    right, up, facing = np.array([-sy, cy, 0.0]), np.array([0.0, 0.0, 1.0]), np.array([cy, sy, 0.0])
    a = -pitch
    y1 = math.cos(a) * up + math.sin(a) * facing
    z1 = -math.sin(a) * up + math.cos(a) * facing
    x2 = math.cos(roll) * right + math.sin(roll) * y1
    y2 = -math.sin(roll) * right + math.cos(roll) * y1
    return np.column_stack([x2, y2, z1])


def ceiling_marker_axes(yaw: float) -> np.ndarray:
    """The ceiling list's convention: x = (cos, sin, 0), z down, y = z cross x."""
    return np.column_stack([[math.cos(yaw), math.sin(yaw), 0.0],
                            [math.sin(yaw), -math.cos(yaw), 0.0],
                            [0.0, 0.0, -1.0]])


def _parse(text: str, source: str):
    try:
        d = yaml.load(text, Loader=mm._LineLoader)
    except (yaml.YAMLError, mm.MarkerMapError) as e:
        raise mm.MarkerMapError(f'{source}: {e}') from None
    if not isinstance(d, dict) or not isinstance(d.get('markers'), list):
        raise mm.MarkerMapError(f'{source}: expected a mapping with a list of markers')
    lines = d.pop('__lines__')
    for k in ('__line__', '__end_line__'):
        d.pop(k)
    spans = []
    for m in d['markers']:
        if not isinstance(m, dict) or 'id' not in m:
            raise mm.MarkerMapError(f'{source}: every marker needs an id')
        m.pop('__lines__')
        a, b = m.pop('__line__'), m.pop('__end_line__')
        spans.append((int(m['id']), a, b if b > a else a))
    comments = mm._scan_comments(text.splitlines(), lines, spans)
    return d, comments


def _number(m: dict, k: str, source: str, default: Optional[float] = None) -> float:
    v = m.get(k, default)
    if v is None or isinstance(v, bool) or not isinstance(v, (int, float)):
        raise mm.MarkerMapError(f'{source}: marker {m.get("id")}: {k}: expected a number, got {v!r}')
    return float(v)


def _deg(rad: float) -> float:
    """Degrees of a source angle in radians, as round as the source's own precision
    allows: 0.5236 rad is 30.0 deg, not 30.00007. The value is rounded to the fewest
    decimals (0, 1 or 2) that stay within half a unit of the source's last digit and
    within ANGLE_ROUNDING_DEG."""
    deg = math.degrees(rad)
    text = repr(abs(float(rad)))
    digits = len(text.split('.')[1]) if '.' in text and 'e' not in text else 15
    tol = min(math.degrees(0.5 * 10.0 ** -digits), ANGLE_ROUNDING_DEG)
    for n in (0, 1, 2):
        r = round(deg, n)
        if abs(r - deg) <= tol + 1e-12:
            return r + 0.0          # no -0.0
    return deg


ANGLE_ROUNDING_DEG = 0.005   # the most an imported angle moves by rounding


def _finish(markers, comments, dictionary, size, frame_id, mount, source,
            keep_header) -> mm.MarkerMap:
    for mk in markers:
        mk.comment = comments.marker_comment.get(mk.id)
        mk.comment_above = comments.marker_above.get(mk.id, [])
    if keep_header:
        header, footer = list(comments.header), list(comments.footer)
    else:
        header = [f'Imported from {os.path.basename(source)} by marker_map_import.']
        footer = []
    return mm.MarkerMap(dictionary=dictionary, marker_size=float(size), markers=markers,
                        frame_id=frame_id, mount=mount, header=header,
                        key_comments=dict(mm.DEFAULT_KEY_COMMENTS), footer=footer)


def from_course(text: str, source: str = '<course>', dictionary: Optional[str] = None,
                frame_id: str = 'map', keep_header: bool = False) -> mm.MarkerMap:
    d, comments = _parse(text, source)
    dic = dictionary or d.get('dictionary')
    if not dic or dic == 'ALL_DICTS':
        raise mm.MarkerMapError(f'{source}: name the printed dictionary (--dictionary), the '
                                f'file has {dic!r}')
    mm.opencv_dictionary_name(dic)
    size = _number(d, 'marker_size', source)
    height = _number(d, 'marker_height', source, 0.25)
    markers = []
    for m in d['markers']:
        yaw, pitch, roll = (_number(m, k, source, 0.0) for k in ('yaw', 'pitch', 'roll'))
        angles = {'facing_deg': _deg(yaw), 'lean_deg': _deg(pitch), 'turn_deg': _deg(roll)}
        markers.append(mm.MapMarker(int(m['id']), _number(m, 'x', source), _number(m, 'y', source),
                                    _number(m, 'z', source, height), 'wall', angles))
    return _finish(markers, comments, dic, size, frame_id, 'wall', source, keep_header)


def from_ceiling_list(text: str, source: str = '<markers>', frame_id: str = 'map',
                      keep_header: bool = False) -> mm.MarkerMap:
    d, comments = _parse(text, source)
    dic = d.get('family') or d.get('dictionary')
    if not dic:
        raise mm.MarkerMapError(f'{source}: family missing')
    mm.opencv_dictionary_name(dic)
    size = _number(d, 'marker_size', source)
    markers = []
    for m in d['markers']:
        markers.append(mm.MapMarker(int(m['id']), _number(m, 'x', source), _number(m, 'y', source),
                                    _number(m, 'z', source), 'ceiling',
                                    {'rotation_deg': _deg(_number(m, 'yaw', source, 0.0))}))
    return _finish(markers, comments, dic, size, frame_id, 'ceiling', source, keep_header)


def source_poses(kind: str, text: str) -> dict:
    """{id: T_map_marker} of a source file, by the source format's own convention."""
    d = yaml.safe_load(text)
    out = {}
    for m in d['markers']:
        T = np.eye(4)
        if kind == 'course':
            T[:3, :3] = course_marker_axes(m['yaw'], m.get('pitch', 0.0), m.get('roll', 0.0))
            T[:3, 3] = [m['x'], m['y'], m.get('z', d.get('marker_height', 0.25))]
        else:
            T[:3, :3] = ceiling_marker_axes(m.get('yaw', 0.0))
            T[:3, 3] = [m['x'], m['y'], m['z']]
        out[int(m['id'])] = T
    return out
