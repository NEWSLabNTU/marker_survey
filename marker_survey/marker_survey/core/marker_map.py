"""The marker map file, `marker_map.yaml`, version 1. No ROS.

One file per site: the survey writes it, a person can read and fix it, and
the localizer reads it as it is. Example:

    # Ceiling markers of the warehouse.
    version: 1
    frame_id: map                 # the frame the robot's map uses
    dictionary: tag36h11          # printed family
    marker_size: 0.45             # [m] edge of the black square, measured
    mount: ceiling                # default for every marker below
    markers:
      - {id: 0,  x: -4.000, y: 1.500, z: 3.012, rotation_deg:  0.0}
      - {id: 7,  x:  3.000, y: -2.000, z: 3.204, rotation_deg: 91.7, tilt_x_deg: 1.2}   # under the duct
      - {id: 12, x:  0.300, y: 3.000, z: 3.005, rotation_deg: 17.2, size: 0.30}

Units are metres and degrees. A marker's orientation is given relative to
its mounting, so a well-mounted marker has small numbers:

| mount   | reference pose (all angles zero)                   | angles                                    |
|---------|----------------------------------------------------|-------------------------------------------|
| wall    | upright, facing +x, top up                         | facing_deg, lean_deg, turn_deg            |
| ceiling | facing down, image x axis along +x (top towards -y) | rotation_deg, tilt_x_deg, tilt_y_deg      |
| floor   | facing up, image x axis along +x (top towards +y)  | rotation_deg, tilt_x_deg, tilt_y_deg      |

The marker's own axes are OpenCV's (x to the right of the printed image, y
towards its top, z out of the marker). The rotation matrix R (columns: the
marker's axes in the map frame) is, with R_ref the reference pose:

    wall:            R = Rz(facing) Ry(-lean) Rx(turn) R_ref
                     facing: about the vertical, counter-clockwise from above;
                     lean: the top leaning back, away from the viewer;
                     turn: turned in its own plane, counter-clockwise as seen
                     from the front.
    ceiling, floor:  R = Rz(rotation) Rx(tilt_x) Ry(tilt_y) R_ref
                     rotation: about the vertical, counter-clockwise from
                     above; tilt_x, tilt_y: small tilts about the map's x and
                     y axes after the rotation.

Missing angles are zero. The localizer and the survey use only the matrix.

The loader names the line and the field of every error. Comments survive a
load and write: the header block, a comment after a top-level key, comment
lines above a marker and the comment at the end of a marker's line.
"""

from dataclasses import dataclass, field
import math
import os
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

from marker_survey.core.detection import marker_object_points, opencv_dictionary_name
from marker_survey.core.se3 import make_T, rot_x, rot_y, rot_z

VERSION = 1
MOUNTS = ('wall', 'ceiling', 'floor')
ANGLES = {
    'wall': ('facing_deg', 'lean_deg', 'turn_deg'),
    'ceiling': ('rotation_deg', 'tilt_x_deg', 'tilt_y_deg'),
    'floor': ('rotation_deg', 'tilt_x_deg', 'tilt_y_deg'),
}
# Reference orientations, columns: the marker's x, y, z axes in the map frame.
R_REF = {
    'wall': np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
    'ceiling': np.diag([1.0, -1.0, -1.0]),
    'floor': np.eye(3),
}
TOP_KEYS = ('version', 'frame_id', 'dictionary', 'marker_size', 'mount', 'markers')
MARKER_KEYS = ('id', 'x', 'y', 'z', 'mount', 'size')
DEFAULT_KEY_COMMENTS = {
    'frame_id': "the frame the robot's map uses",
    'dictionary': 'printed family',
    'marker_size': '[m] edge of the black square, measured',
    'mount': 'default for every marker below',
}
COMMENT_COLUMN = 30


class MarkerMapError(ValueError):
    """A marker map that cannot be used; the message names the line and field."""


# ------------------------------------------------------------------ mounts

def mount_rotation(mount: str, angles_deg: Dict[str, float]) -> np.ndarray:
    """The marker's rotation matrix in the map frame from its mount and angles."""
    a = [math.radians(float(angles_deg.get(k, 0.0) or 0.0)) for k in ANGLES[mount]]
    if mount == 'wall':
        E = rot_z(a[0]) @ rot_y(-a[1]) @ rot_x(a[2])
    else:
        E = rot_z(a[0]) @ rot_x(a[1]) @ rot_y(a[2])
    return E @ R_REF[mount]


def angles_from_rotation(mount: str, R) -> Dict[str, float]:
    """The inverse of mount_rotation: the mount's angles [deg] for a rotation matrix."""
    from scipy.spatial.transform import Rotation
    E = np.asarray(R, float) @ R_REF[mount].T
    if mount == 'wall':
        z, y, x = Rotation.from_matrix(E).as_euler('ZYX')
        vals = (z, -y, x)
    else:
        z, x, y = Rotation.from_matrix(E).as_euler('ZXY')
        vals = (z, x, y)
    return {k: math.degrees(v) for k, v in zip(ANGLES[mount], vals)}


def mount_from_normal(normal) -> str:
    """The mount a marker normal (its z axis, map frame) suggests."""
    nz = float(np.asarray(normal, float)[2] / np.linalg.norm(normal))
    if nz < -0.7:
        return 'ceiling'
    if nz > 0.7:
        return 'floor'
    return 'wall'


def tilt_from_mount(mount: str, R) -> float:
    """[deg] how far the marker's normal is from its mount's reference normal: the lean
    from vertical for a wall board (turning it in its own plane is not a tilt), the
    angle from straight down (ceiling) or straight up (floor)."""
    n = np.asarray(R, float)[:, 2]
    if mount == 'wall':
        return math.degrees(math.asin(min(1.0, abs(n[2]))))
    ref = -1.0 if mount == 'ceiling' else 1.0
    return math.degrees(math.acos(max(-1.0, min(1.0, ref * n[2]))))


# ------------------------------------------------------------------ the model

@dataclass
class MapMarker:
    id: int
    x: float
    y: float
    z: float
    mount: str
    angles: Dict[str, float] = field(default_factory=dict)   # [deg], keys of ANGLES[mount]
    size: Optional[float] = None          # [m] if different from the map's marker_size
    comment: Optional[str] = None         # at the end of its line
    comment_above: List[str] = field(default_factory=list)   # whole comment lines above it
    line: Optional[int] = None            # where it was read from

    @property
    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], float)

    def rotation(self) -> np.ndarray:
        return mount_rotation(self.mount, self.angles)

    def pose(self) -> np.ndarray:
        """T_map_marker."""
        return make_T(self.rotation(), self.position)

    @classmethod
    def from_pose(cls, mid: int, T, mount: Optional[str] = None, size: Optional[float] = None,
                  decimals: Optional[Tuple[int, int]] = None) -> 'MapMarker':
        """A marker at pose T (map frame); mount from its normal unless given. decimals
        (position, angle) rounds the values, as a survey writes them."""
        T = np.asarray(T, float)
        mount = mount or mount_from_normal(T[:3, 2])
        ang = angles_from_rotation(mount, T[:3, :3])
        x, y, z = (float(v) for v in T[:3, 3])
        if decimals is not None:
            x, y, z = (round(v, decimals[0]) for v in (x, y, z))
            ang = {k: round(v, decimals[1]) for k, v in ang.items()}
        ang = {k: (0.0 if v == 0 else v) for k, v in ang.items()}
        return cls(int(mid), x, y, z, mount, ang, size)


@dataclass
class MarkerMap:
    dictionary: str
    marker_size: float
    markers: List[MapMarker] = field(default_factory=list)
    frame_id: str = 'map'
    mount: Optional[str] = None           # default mount of the markers
    version: int = VERSION
    header: List[Optional[str]] = field(default_factory=list)   # comment lines at the top (None: blank)
    key_comments: Dict[str, str] = field(default_factory=dict)
    key_above: Dict[str, List[str]] = field(default_factory=dict)
    footer: List[str] = field(default_factory=list)
    path: Optional[str] = None

    def ids(self) -> List[int]:
        return [m.id for m in self.markers]

    def by_id(self) -> Dict[int, MapMarker]:
        return {m.id: m for m in self.markers}

    def get(self, mid: int) -> MapMarker:
        for m in self.markers:
            if m.id == mid:
                return m
        raise KeyError(f'marker {mid} is not in the map')

    def size_of(self, mid: int) -> float:
        m = self.get(mid)
        return float(m.size) if m.size is not None else float(self.marker_size)

    def pose_of(self, mid: int) -> np.ndarray:
        return self.get(mid).pose()

    def poses(self) -> Dict[int, np.ndarray]:
        return {m.id: m.pose() for m in self.markers}

    def sizes(self) -> Dict[int, float]:
        return {m.id: self.size_of(m.id) for m in self.markers}

    def corners_of(self, mid: int) -> np.ndarray:
        """4 x 3 map coordinates of the corners, in OpenCV's corner order."""
        T = self.pose_of(mid)
        return marker_object_points(self.size_of(mid)) @ T[:3, :3].T + T[:3, 3]


# ------------------------------------------------------------------ loading

class _LineLoader(yaml.SafeLoader):
    """A SafeLoader that records where every mapping and key was read, and refuses
    duplicate keys."""

    def construct_mapping(self, node, deep=False):
        keys = {}
        for k, _ in node.value:
            kv = k.value
            if kv in keys:
                raise MarkerMapError(f'line {k.start_mark.line + 1}: {kv!r} appears twice '
                                     f'(first on line {keys[kv]})')
            keys[kv] = k.start_mark.line + 1
        m = super().construct_mapping(node, deep=True)
        m['__line__'] = node.start_mark.line + 1
        m['__end_line__'] = node.end_mark.line + 1
        m['__lines__'] = keys
        return m


_LineLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
                            _LineLoader.construct_mapping)


def _split_comment(line: str) -> Tuple[str, Optional[str]]:
    """(code, comment text or None) of one YAML line; '#' inside quotes is not a comment."""
    quote = None
    for i, ch in enumerate(line):
        if quote:
            if ch == quote:
                quote = None
        elif ch in ('"', "'"):
            quote = ch
        elif ch == '#' and (i == 0 or line[i - 1] in ' \t'):
            return line[:i].rstrip(), line[i + 1:].strip()
    return line.rstrip(), None


def _is_comment_line(line: str) -> bool:
    return line.strip().startswith('#')


def _comment_text(line: str) -> str:
    """The text of a comment line: after the '#' and one space, indentation kept."""
    t = line.strip()[1:]
    return (t[1:] if t.startswith(' ') else t).rstrip()


@dataclass
class _Comments:
    header: List[str]
    key_comments: Dict[str, str]
    key_above: Dict[str, List[str]]
    marker_comment: Dict[int, str]
    marker_above: Dict[int, List[str]]
    footer: List[str]


def _scan_comments(lines: List[str], key_lines: Dict[str, int],
                   marker_lines: List[Tuple[int, int, int]]) -> _Comments:
    """Comments of a YAML text. key_lines: top-level key -> line (1-based);
    marker_lines: (id, first line, last line) of every marker entry."""
    n = len(lines)
    first_code = next((i for i, ln in enumerate(lines) if ln.strip() and not _is_comment_line(ln)), n)
    # blank lines between header comments are kept (as None)
    header = []
    for ln in lines[:first_code]:
        if _is_comment_line(ln):
            header.append(_comment_text(ln))
        elif header:
            header.append(None)
    while header and header[-1] is None:
        header.pop()
    key_comments, key_above = {}, {}
    starts = sorted([(ln, ('key', k)) for k, ln in key_lines.items()]
                    + [(a, ('marker', mid, b)) for mid, a, b in marker_lines])
    claimed = set(range(first_code))
    marker_comment, marker_above = {}, {}
    for ln, what in starts:
        i = ln - 1
        # comment lines directly above (not part of the header)
        above = []
        j = i - 1
        while j >= 0 and j not in claimed and _is_comment_line(lines[j]):
            above.insert(0, _comment_text(lines[j]))
            claimed.add(j)
            j -= 1
        if what[0] == 'key':
            _, c = _split_comment(lines[i])
            if c is not None and what[1] != 'markers':
                key_comments[what[1]] = c
            elif c is not None:
                key_comments['markers'] = c
            if above:
                key_above[what[1]] = above
            claimed.add(i)
        else:
            mid, last = what[1], what[2]
            for k in range(i, last):
                claimed.add(k)
            c = None
            for k in range(i, min(last, n)):
                _, ck = _split_comment(lines[k])
                if ck is not None:
                    c = ck
            if c is not None:
                marker_comment[mid] = c
            if above:
                marker_above[mid] = above
    footer = []
    last_code = max([ln for ln, _ in starts] + [b for _, _, b in marker_lines] + [0])
    for k in range(last_code, n):
        if k not in claimed and _is_comment_line(lines[k]):
            footer.append(_comment_text(lines[k]))
    return _Comments(header, key_comments, key_above, marker_comment, marker_above, footer)


def _is_number(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def loads(text: str, source: str = '<string>') -> MarkerMap:
    """Parse and validate a marker map. Raises MarkerMapError naming line and field."""
    def err(line, msg):
        raise MarkerMapError(f'{source}:{line}: {msg}' if line else f'{source}: {msg}')

    try:
        d = yaml.load(text, Loader=_LineLoader)
    except MarkerMapError as e:
        raise MarkerMapError(f'{source}: {e}') from None
    except yaml.YAMLError as e:
        mark = getattr(e, 'problem_mark', None)
        line = mark.line + 1 if mark is not None else None
        err(line, f'not valid YAML: {getattr(e, "problem", e)}')
    if not isinstance(d, dict):
        err(None, 'expected a mapping with version, dictionary, marker_size, markers')
    lines = d.pop('__lines__')
    d.pop('__line__')
    d.pop('__end_line__')
    for k in d:
        if k not in TOP_KEYS:
            err(lines.get(k), f'unknown key {k!r} (known: {", ".join(TOP_KEYS)})')
    for k in ('version', 'dictionary', 'marker_size', 'markers'):
        if k not in d:
            err(None, f'{k} missing')
    if d['version'] != VERSION:
        err(lines['version'], f'version: {d["version"]!r} is not supported (this reader: {VERSION})')
    dictionary = d['dictionary']
    if not isinstance(dictionary, str):
        err(lines['dictionary'], f'dictionary: expected a name, got {dictionary!r}')
    try:
        opencv_dictionary_name(dictionary)
    except ValueError as e:
        err(lines['dictionary'], f'dictionary: {e}')
    size = d['marker_size']
    if not _is_number(size) or size <= 0:
        err(lines['marker_size'], f'marker_size: expected a positive number of metres, got {size!r}')
    frame_id = d.get('frame_id', 'map')
    if not isinstance(frame_id, str) or not frame_id:
        err(lines.get('frame_id'), f'frame_id: expected a frame name, got {frame_id!r}')
    mount = d.get('mount')
    if mount is not None and mount not in MOUNTS:
        err(lines['mount'], f'mount: {mount!r} is not one of {", ".join(MOUNTS)}')
    items = d['markers']
    if items is None:
        items = []
    if not isinstance(items, list):
        err(lines['markers'], 'markers: expected a list of {id: ..., x: ..., ...} entries')

    markers, seen, spans = [], {}, []
    for n, m in enumerate(items):
        if not isinstance(m, dict):
            err(lines['markers'],
                f'markers[{n}]: expected {{id: ..., x: ..., y: ..., z: ...}}, got {m!r}')
        ml = m.pop('__lines__')
        line = m.pop('__line__')
        end = m.pop('__end_line__')
        if 'id' not in m:
            err(line, f'markers[{n}]: id missing')
        mid = m['id']
        if not isinstance(mid, int) or isinstance(mid, bool) or mid < 0:
            err(ml['id'], f'markers[{n}]: id: expected a non-negative integer, got {mid!r}')
        who = f'marker {mid}'
        if mid in seen:
            err(line, f'{who}: id appears twice (first on line {seen[mid]})')
        seen[mid] = line
        mmount = m.get('mount', mount)
        if mmount is None:
            err(line, f'{who}: mount missing (set mount: at the top or on the marker)')
        if mmount not in MOUNTS:
            err(ml.get('mount'), f'{who}: mount: {mmount!r} is not one of {", ".join(MOUNTS)}')
        allowed = MARKER_KEYS + ANGLES[mmount]
        for k in m:
            if k in allowed:
                continue
            other = [mt for mt in MOUNTS if k in ANGLES[mt]]
            if other:
                err(ml[k], f'{who}: {k} is an angle of mount {other[0]}, not {mmount} '
                           f'(use {", ".join(ANGLES[mmount])})')
            err(ml[k], f'{who}: unknown field {k!r} (known: {", ".join(allowed)})')
        for k in ('x', 'y', 'z'):
            if k not in m:
                err(line, f'{who}: {k} missing')
        for k in ('x', 'y', 'z') + ANGLES[mmount]:
            if k in m and not _is_number(m[k]):
                err(ml[k], f'{who}: {k}: expected a number, got {m[k]!r}')
        msize = m.get('size')
        if msize is not None and (not _is_number(msize) or msize <= 0):
            err(ml['size'], f'{who}: size: expected a positive number of metres, got {msize!r}')
        angles = {k: float(m.get(k, 0.0)) for k in ANGLES[mmount]}
        markers.append(MapMarker(mid, float(m['x']), float(m['y']), float(m['z']), mmount, angles,
                                 None if msize is None else float(msize), line=line))
        spans.append((mid, line, end if end > line else line))

    c = _scan_comments(text.splitlines(), lines, spans)
    for mk in markers:
        mk.comment = c.marker_comment.get(mk.id)
        mk.comment_above = c.marker_above.get(mk.id, [])
    return MarkerMap(dictionary=dictionary, marker_size=float(size), markers=markers,
                     frame_id=frame_id, mount=mount, version=d['version'], header=c.header,
                     key_comments=c.key_comments, key_above=c.key_above, footer=c.footer,
                     path=None if source == '<string>' else source)


def load(path: str) -> MarkerMap:
    with open(path) as f:
        text = f.read()
    m = loads(text, source=path)
    m.path = path
    return m


# ------------------------------------------------------------------ writing

def _fmt(v: float, decimals: int) -> str:
    s = f'{v:.{decimals}f}'
    if float(s) == 0.0:
        s = s.lstrip('-')
    return s


def _decimals_needed(v: float, minimum: int, maximum: int = 6) -> int:
    s = f'{v:.{maximum}f}'.rstrip('0')
    frac = s.split('.')[1] if '.' in s else ''
    return max(minimum, len(frac))


def _comment(text: Optional[str]) -> str:
    return '#' if not text else f'# {text}'


def dumps(mm: MarkerMap) -> str:
    """The map as text: header, top-level keys with their comments, one aligned line per
    marker in flow style."""
    out = [_comment(h) if h is not None else '' for h in mm.header]

    def key_line(k: str, value: str):
        for a in mm.key_above.get(k, []):
            out.append(_comment(a))
        line = f'{k}: {value}' if value != '' else f'{k}:'
        c = mm.key_comments.get(k)
        if c is not None:
            line = line.ljust(COMMENT_COLUMN - 1) + ' ' + _comment(c)
        out.append(line)

    key_line('version', str(mm.version))
    key_line('frame_id', mm.frame_id)
    key_line('dictionary', mm.dictionary)
    key_line('marker_size', _fmt(mm.marker_size, _decimals_needed(mm.marker_size, 2)))
    if mm.mount is not None:
        key_line('mount', mm.mount)
    if not mm.markers:
        key_line('markers', '[]')
    else:
        key_line('markers', '')

    # aligned columns: id, x, y, z, the primary angle; secondary fields as needed
    ms = mm.markers
    pos_dec = {k: max(_decimals_needed(getattr(m, k), 3) for m in ms) for k in ('x', 'y', 'z')}
    prim = {m.id: ANGLES[m.mount][0] for m in ms}
    prim_vals = [m.angles.get(prim[m.id], 0.0) for m in ms]
    ang_dec = max(_decimals_needed(v, 1) for v in prim_vals)
    id_w = max(len(str(m.id)) for m in ms)
    cols = {k: max(len(_fmt(getattr(m, k), pos_dec[k])) for m in ms) for k in ('x', 'y', 'z')}
    prim_w = max([len(_fmt(v, ang_dec)) for v in prim_vals] + [ang_dec + 4])   # room for 180.0
    prim_key_w = max(len(prim[m.id]) for m in ms)
    rows = []
    for m in ms:
        parts = [f'id: {m.id},'.ljust(len('id: ,') + id_w)]
        for k in ('x', 'y', 'z'):
            parts.append(f'{k}: {_fmt(getattr(m, k), pos_dec[k]).rjust(cols[k])},')
        pk = prim[m.id]
        value = _fmt(m.angles.get(pk, 0.0), ang_dec).rjust(prim_w)
        parts.append(f'{(pk + ":").ljust(prim_key_w + 1)} {value}')
        extra = []
        if m.mount != mm.mount:
            extra.append(f'mount: {m.mount}')
        for k in ANGLES[m.mount][1:]:
            v = m.angles.get(k, 0.0)
            if v:
                extra.append(f'{k}: {_fmt(v, _decimals_needed(v, 1))}')
        if m.size is not None and m.size != mm.marker_size:
            extra.append(f'size: {_fmt(m.size, _decimals_needed(m.size, 2))}')
        body = ' '.join(parts)
        base = len('  - {' + body + '}')
        if extra:
            body += ', ' + ', '.join(extra)
        rows.append((m, '  - {' + body + '}', base))
    # trailing comments line up after the aligned columns; longer rows push theirs on
    width = max(base for _, _, base in rows)
    for m, r, _ in rows:
        for a in m.comment_above:
            out.append('  ' + _comment(a))
        if m.comment is not None:
            r = r.ljust(width) + '   ' + _comment(m.comment)
        out.append(r)
    out += [_comment(f) for f in mm.footer]
    return '\n'.join(out) + '\n'


def write(mm: MarkerMap, path: str) -> None:
    d = os.path.dirname(os.path.abspath(path))
    os.makedirs(d, exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        f.write(dumps(mm))
    os.replace(tmp, path)


def default_mount(markers: List[MapMarker]) -> Optional[str]:
    """The most common mount, as the map's default."""
    if not markers:
        return None
    counts = {}
    for m in markers:
        counts[m.mount] = counts.get(m.mount, 0) + 1
    return max(MOUNTS, key=lambda k: counts.get(k, 0))
