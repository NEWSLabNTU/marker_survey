"""Ceiling marker size and spacing from the site and the camera. No ROS.

Inputs: the ceiling height (laser distance meter), the camera's height on the
robot, its horizontal field of view at the resolution the robot uses, the
robot's speed and the exposure time (motion blur), and the odometry drift.

- Ground sampling: d = ceiling - camera height, f = (width / 2) / tan(hfov / 2)
  pixels, gsd = d / f metres on the ceiling per pixel.
- Smallest readable marker: CELLS cells of at least `need_px` pixels each, where
  need_px is the larger of `min_px_per_cell` and three times the motion blur.
- Largest marker that fits in view at any heading: (view height - 2 borders) / sqrt(2).
- Zone radius R: within R of a marker, the camera sees all of it at any heading.
- Spacing: a marker always in view on a square grid, D <= sqrt(2) R; along a
  route with odometry in between, D <= 2 R + max_drift_error / drift.
"""
import math

CELLS = 8          # tag36h11: 6 x 6 data cells plus a 1-cell black border
SHEET_FACTOR = 1.25  # printed width / black square: a 1-cell white margin each side
SAFETY = 1.5       # recommended size / smallest readable size
SHEETS = [('A3', 0.297), ('A2', 0.420), ('A1', 0.594), ('A0', 0.841)]   # short side, m


def plan(ceiling, camera_height, hfov_deg, width, height, speed, exposure,
         min_px_per_cell, border_px, drift, max_drift_error):
    d = ceiling - camera_height                       # camera to ceiling, m
    f = (width / 2) / math.tan(math.radians(hfov_deg) / 2)   # focal length, px
    gsd = d / f                                       # m on the ceiling per pixel
    view_w = width * gsd                              # camera view on the ceiling, m
    view_h = height * gsd
    blur_px = speed * exposure / gsd                  # motion blur while driving, px
    need_px = max(min_px_per_cell, 3 * blur_px)       # pixels per tag cell needed
    border = border_px * gsd                          # image border kept free, m
    s_min = CELLS * need_px * gsd                     # smallest readable marker
    s_max = (view_h - 2 * border) / math.sqrt(2)      # largest that fits at any heading
    return {'d': d, 'f': f, 'gsd': gsd, 'view_w': view_w, 'view_h': view_h,
            'blur_px': blur_px, 'need_px': need_px, 'border': border,
            's_min': s_min, 's_max': s_max, 'drift': drift,
            'max_gap': max_drift_error / drift}


def zone_radius(p, s):
    """Radius around a marker within which the camera sees all of it, any heading."""
    return (p['view_h'] - 2 * p['border'] - s * math.sqrt(2)) / 2


def recommend(p, size=None):
    """The recommended size (the smallest readable with a safety factor, rounded up to
    5 cm), its zone, spacings and sheet; None if no size works."""
    if p['s_min'] >= p['s_max']:
        return None
    if size is None:
        size = min(math.ceil(SAFETY * p['s_min'] * 20) / 20, p['s_max'])
    r = zone_radius(p, size)
    printed = size * SHEET_FACTOR
    sheet = next((name for name, side in SHEETS if side >= printed), 'larger than A0 (banner)')
    return {'size': size, 'px_per_cell': p['f'] * size / p['d'] / CELLS, 'printed': printed,
            'sheet': sheet, 'zone_radius': r,
            'grid_spacing': math.sqrt(2) * r if r > 0 else None,
            'route_spacing': 2 * r + p['max_gap'] if r > 0 else None}
