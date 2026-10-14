"""A route as a CSV file of `x, y` waypoints [m]. No ROS.

    # x [m], y [m]
    0.0, 0.0
    6.0, 0.0
    ...

A header row is allowed; lines starting with `#` are comments; further
columns are ignored.
"""

import csv
from typing import List, Tuple


def load_path_csv(path: str) -> List[Tuple[float, float]]:
    pts = []
    with open(path, newline='') as f:
        for n, row in enumerate(csv.reader(f), start=1):
            if not row or row[0].strip().startswith('#'):
                continue
            try:
                vals = [float(c) for c in row if c.strip() != '']
            except ValueError:
                if not pts:
                    continue    # header
                raise ValueError(f'{path}:{n}: expected numbers, got {row}') from None
            if len(vals) < 2:
                raise ValueError(f'{path}:{n}: a waypoint needs x and y, got {row}')
            pts.append((vals[0], vals[1]))
    if len(pts) < 2:
        raise ValueError(f'{path}: fewer than two waypoints')
    return pts
