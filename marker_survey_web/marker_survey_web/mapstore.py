"""The map in use: written with a confirmation, earlier versions kept, undo. No ROS.

The map in use is the file the robot's localizer reads (`path`). Writing a
new one first copies the current file to `<name>_versions/<name>.<date-time>.yaml`
beside it; undo puts the newest of those back, and the file it replaces is
kept there too, as `<name>.<date-time>.undone.yaml`, outside the undo order.
Every file written is checked with the map loader first.
"""

import datetime
import os
import shutil
from typing import Optional

import numpy as np

from marker_survey.core import marker_map as mm

UNDONE = '.undone'


class MapStoreError(ValueError):
    pass


def _summary(path: str) -> dict:
    try:
        m = mm.load(path)
    except (OSError, mm.MarkerMapError) as e:
        return {'error': str(e)}
    return {'markers': len(m.markers), 'ids': m.ids(), 'frame_id': m.frame_id,
            'dictionary': m.dictionary, 'marker_size': m.marker_size}


class MapStore:
    def __init__(self, path: str):
        if not path:
            raise MapStoreError('no map path: where the localizer reads its map')
        self.path = os.path.abspath(path)
        stem, _ = os.path.splitext(os.path.basename(self.path))
        self.stem = stem
        self.versions_dir = os.path.join(os.path.dirname(self.path), f'{stem}_versions')

    # ------------------------------------------------------------ reading
    def versions(self) -> list:
        """The undo order, newest first: files <stem>.<date-time>.yaml."""
        if not os.path.isdir(self.versions_dir):
            return []
        names = [n for n in os.listdir(self.versions_dir)
                 if n.startswith(self.stem + '.') and n.endswith('.yaml') and UNDONE not in n]
        return sorted(names, reverse=True)

    def info(self) -> dict:
        out = {'path': self.path, 'exists': os.path.isfile(self.path), 'versions': [],
               'can_undo': False}
        if out['exists']:
            out.update(_summary(self.path))
            out['modified'] = datetime.datetime.fromtimestamp(
                os.path.getmtime(self.path)).astimezone().isoformat(timespec='seconds')
        for n in self.versions():
            st = n[len(self.stem) + 1:-len('.yaml')]
            try:
                date = datetime.datetime.strptime(st, '%Y%m%d-%H%M%S-%f').strftime('%Y-%m-%d %H:%M:%S')
            except ValueError:
                date = st
            v = {'name': n, 'date': date}
            v.update({k: s for k, s in _summary(os.path.join(self.versions_dir, n)).items()
                      if k in ('markers', 'error')})
            out['versions'].append(v)
        out['can_undo'] = bool(out['versions'])
        return out

    def diff(self, src: str, moved_m: float = 0.02) -> dict:
        """What writing `src` would change: markers added, removed, moved (more than
        moved_m), and the largest move."""
        new = self._load(src)
        old = mm.load(self.path) if os.path.isfile(self.path) else None
        np_ = new.poses()
        op = old.poses() if old is not None else {}
        moved, same = [], 0
        for mid in sorted(set(np_) & set(op)):
            d = float(np.linalg.norm(np_[mid][:3, 3] - op[mid][:3, 3]))
            if d > moved_m:
                moved.append({'id': mid, 'distance_m': round(d, 3)})
            else:
                same += 1
        out = {'source': os.path.abspath(src), 'target': self.path, 'markers': len(new.markers),
               'added': sorted(set(np_) - set(op)), 'removed': sorted(set(op) - set(np_)),
               'moved': moved, 'unchanged': same, 'first': old is None,
               'frame_id': new.frame_id, 'old_frame_id': None if old is None else old.frame_id}
        out['max_move_m'] = max([m['distance_m'] for m in moved], default=0.0)
        return out

    # ------------------------------------------------------------ writing
    def _load(self, path: str) -> mm.MarkerMap:
        try:
            return mm.load(path)
        except OSError as e:
            raise MapStoreError(f'{path}: {e.strerror or e}') from None
        except mm.MarkerMapError as e:
            raise MapStoreError(str(e)) from None

    def _stamp(self, suffix: str = '') -> str:
        # microseconds: the names sort in the order they were written
        stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        return f'{self.stem}.{stamp}{suffix}.yaml'

    def _keep_current(self, suffix: str = '') -> Optional[str]:
        if not os.path.isfile(self.path):
            return None
        os.makedirs(self.versions_dir, exist_ok=True)
        name = self._stamp(suffix)
        shutil.copy2(self.path, os.path.join(self.versions_dir, name))
        return name

    def _put(self, src: str):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + '.writing'
        shutil.copyfile(src, tmp)
        os.replace(tmp, self.path)              # whole or not at all, for a reader

    def use(self, src: str) -> dict:
        """Write `src` as the map in use; the current one becomes a version."""
        self._load(src)
        if os.path.isfile(self.path) and os.path.samefile(src, self.path):
            raise MapStoreError('this map is already the one in use')
        kept = self._keep_current()
        self._put(src)
        return {'written': self.path, 'kept': kept}

    def undo(self) -> dict:
        """Put the newest kept version back."""
        vs = self.versions()
        if not vs:
            raise MapStoreError('nothing to undo: there is no earlier version')
        prev = os.path.join(self.versions_dir, vs[0])
        self._load(prev)
        replaced = self._keep_current(UNDONE)
        self._put(prev)
        os.remove(prev)
        return {'restored': vs[0], 'kept': replaced}
