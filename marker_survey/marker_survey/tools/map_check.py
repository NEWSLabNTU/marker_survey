"""marker_map_check: load and validate a marker map, print it from above.

    marker_map_check site/marker_map.yaml [--png top.png] [--width 72]

Errors name the line and the field, and the exit status is 1. A valid map is
summarised (markers per mount, extent, sizes, duplicated corners) and drawn
from above in the terminal: each marker as its id, an arrow for the
direction a wall board faces. --png draws the same view to an image.
"""
import argparse
import math
import sys

import numpy as np

from marker_survey.core import marker_map as mm


def summary_lines(m: mm.MarkerMap):
    out = [f'{m.path or "map"}: version {m.version}, frame {m.frame_id}, {m.dictionary}, '
           f'marker_size {m.marker_size} m, {len(m.markers)} markers']
    if not m.markers:
        return out
    mounts = {}
    for mk in m.markers:
        mounts[mk.mount] = mounts.get(mk.mount, 0) + 1
    out.append('  mounts: ' + ', '.join(f'{k} {v}' for k, v in sorted(mounts.items())))
    P = np.array([mk.position for mk in m.markers])
    out.append(f'  x {P[:, 0].min():.2f} .. {P[:, 0].max():.2f} m, y {P[:, 1].min():.2f} .. '
               f'{P[:, 1].max():.2f} m, z {P[:, 2].min():.2f} .. {P[:, 2].max():.2f} m')
    sizes = sorted({m.size_of(i) for i in m.ids()})
    if len(sizes) > 1:
        out.append('  sizes: ' + ', '.join(f'{s} m' for s in sizes))
    tilted = [(mk.id, mm.tilt_from_mount(mk.mount, mk.rotation())) for mk in m.markers]
    tilted = [(i, t) for i, t in tilted if t > 10.0]
    if tilted:
        out.append('  WARNING: tilted more than 10 deg from their mount: '
                   + ', '.join(f'{i} ({t:.0f} deg)' for i, t in tilted))
    close = []
    for a in range(len(m.markers)):
        for b in range(a + 1, len(m.markers)):
            if np.linalg.norm(P[a] - P[b]) < max(m.size_of(m.markers[a].id), m.size_of(m.markers[b].id)):
                close.append((m.markers[a].id, m.markers[b].id))
    if close:
        out.append('  WARNING: overlapping markers: ' + ', '.join(f'{a} and {b}' for a, b in close))
    return out


ARROWS = ['>', '/', '^', '\\', '<', '/', 'v', '\\']


def ascii_plot(m: mm.MarkerMap, width: int = 72):
    """Top view, x to the right, y up."""
    if not m.markers:
        return []
    P = np.array([mk.position for mk in m.markers])
    lo, hi = P[:, :2].min(axis=0) - 0.5, P[:, :2].max(axis=0) + 0.5
    span = max(hi - lo)
    sx = (width - 1) / span
    sy = sx / 2.0                          # terminal cells are about twice as tall as wide
    rows = max(3, int(round((hi[1] - lo[1]) * sy)) + 1)
    cols = int(round((hi[0] - lo[0]) * sx)) + 1
    grid = [[' '] * (cols + 4) for _ in range(rows)]
    for mk in m.markers:
        c = int(round((mk.x - lo[0]) * sx))
        r = rows - 1 - int(round((mk.y - lo[1]) * sy))
        label = str(mk.id)
        if mk.mount == 'wall':
            n = mk.rotation()[:, 2]
            k = int(round(math.atan2(n[1], n[0]) / (math.pi / 4))) % 8
            label += ARROWS[k]
        for i, ch in enumerate(label):
            if c + i < len(grid[r]):
                grid[r][c + i] = ch
    lines = ['  ' + ''.join(g).rstrip() for g in grid]
    lines.append(f'  x {lo[0]:.1f} .. {hi[0]:.1f} m (right), y {lo[1]:.1f} .. {hi[1]:.1f} m (up)')
    return lines


def png_plot(m: mm.MarkerMap, path: str, px_per_m: float = 60.0):
    import cv2
    P = np.array([mk.position for mk in m.markers]) if m.markers else np.zeros((1, 3))
    lo, hi = P[:, :2].min(axis=0) - 1.0, P[:, :2].max(axis=0) + 1.0
    W = int((hi[0] - lo[0]) * px_per_m) + 1
    H = int((hi[1] - lo[1]) * px_per_m) + 1
    img = np.full((H, W, 3), 255, np.uint8)

    def px(x, y):
        return int(round((x - lo[0]) * px_per_m)), int(round((hi[1] - y) * px_per_m))

    for gx in range(int(math.ceil(lo[0])), int(hi[0]) + 1):
        cv2.line(img, px(gx, lo[1]), px(gx, hi[1]), (230, 230, 230), 1)
    for gy in range(int(math.ceil(lo[1])), int(hi[1]) + 1):
        cv2.line(img, px(lo[0], gy), px(hi[0], gy), (230, 230, 230), 1)
    cv2.arrowedLine(img, px(0, 0), px(1, 0), (0, 0, 200), 2, tipLength=0.2)
    cv2.arrowedLine(img, px(0, 0), px(0, 1), (0, 160, 0), 2, tipLength=0.2)
    colors = {'wall': (160, 80, 0), 'ceiling': (0, 120, 200), 'floor': (90, 90, 90)}
    for mk in m.markers:
        corners = m.corners_of(mk.id)
        pts = np.array([px(c[0], c[1]) for c in corners], np.int32).reshape(-1, 1, 2)
        col = colors[mk.mount]
        cv2.polylines(img, [pts], True, col, 2)
        tl = px(corners[0][0], corners[0][1])
        cv2.circle(img, tl, 3, col, -1)                    # top-left corner of the print
        n = mk.rotation()[:, 2]
        if mk.mount == 'wall':
            c0 = px(mk.x, mk.y)
            cv2.arrowedLine(img, c0, px(mk.x + 0.5 * n[0], mk.y + 0.5 * n[1]), col, 2, tipLength=0.3)
        cv2.putText(img, f'{mk.id}', px(mk.x + 0.15, mk.y + 0.15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(img, f'{m.frame_id}: x right, y up, 1 m grid', (8, H - 8), cv2.FONT_HERSHEY_SIMPLEX,
                0.45, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.imwrite(path, img)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('map', help='marker_map.yaml')
    ap.add_argument('--png', help='also draw the top view to this image')
    ap.add_argument('--width', type=int, default=72, help='terminal plot width')
    ap.add_argument('--no-plot', action='store_true')
    args = ap.parse_args(argv)
    try:
        m = mm.load(args.map)
    except mm.MarkerMapError as e:
        print(f'ERROR: {e}')
        return 1
    except OSError as e:
        print(f'ERROR: {e}')
        return 1
    print('\n'.join(summary_lines(m)))
    if not args.no_plot:
        print('\n'.join(ascii_plot(m, args.width)))
    if args.png:
        png_plot(m, args.png)
        print(f'wrote {args.png}')
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
