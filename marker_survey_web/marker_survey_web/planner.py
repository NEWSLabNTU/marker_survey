"""The plan screen's numbers: marker size, zone and spacing from the site and the camera.

Every number comes from marker_survey's core/marker_plan.py (the same code as the
marker_plan command); this module only reads the page's inputs, picks the
preset, and shapes the answer. The page draws its side view from these
numbers.
"""
import math

from marker_survey.core import marker_plan as mp
from marker_survey.tools.plan import HANDHELD

MIN_PX_PER_CELL = 6.0
BORDER_PX = 5.0
DRIFT = 0.05
MAX_DRIFT_ERROR = 0.05


class PlanError(ValueError):
    pass


ROBOT = {'camera_height': 0.26, 'speed': 0.5}     # marker_plan's defaults
PRESETS = {'robot': ROBOT, 'handheld': HANDHELD}


def presets() -> dict:
    """Camera height and speed for a robot and for a person holding the camera."""
    return {name: dict(v) for name, v in PRESETS.items()}


def hfov_from_camera_info(width: float, fx: float) -> float:
    """The horizontal field of view [deg] of a camera_info: 2 atan(w / 2 fx)."""
    return math.degrees(2.0 * math.atan(float(width) / (2.0 * float(fx))))


def _num(body, key, default=None, cast=float):
    v = body.get(key, default)
    if v in ('', None):
        if default is None:
            raise PlanError(f'give {key.replace("_", " ")}')
        v = default
    try:
        v = cast(v)
    except (TypeError, ValueError):
        raise PlanError(f'{key.replace("_", " ")}: not a number') from None
    if not math.isfinite(v):
        raise PlanError(f'{key.replace("_", " ")}: not a number')
    return v


def site_plan(body: dict) -> dict:
    preset_name = body.get('preset') or 'robot'
    pre = PRESETS.get(preset_name)
    if pre is None:
        raise PlanError(f'unknown preset {preset_name!r}: {", ".join(PRESETS)}')
    ceiling = _num(body, 'ceiling')
    cam = _num(body, 'camera_height', pre['camera_height'])
    hfov = _num(body, 'hfov', 70.0)
    width = _num(body, 'width', 640, int)
    height = _num(body, 'height', 480, int)
    speed = _num(body, 'speed', pre['speed'])
    exposure = _num(body, 'exposure', 0.02)
    if ceiling <= cam:
        raise PlanError('the ceiling must be higher than the camera')
    if not 1.0 <= hfov < 179.0:
        raise PlanError('the field of view is between 1 and 179 degrees')
    if width < 16 or height < 16:
        raise PlanError('the image is at least 16 x 16 pixels')
    if speed < 0 or exposure < 0:
        raise PlanError('speed and exposure are 0 or more')
    p = mp.plan(ceiling, cam, hfov, width, height, speed, exposure, MIN_PX_PER_CELL, BORDER_PX,
                DRIFT, MAX_DRIFT_ERROR)
    size = body.get('size_cm')
    size_m = None if size in ('', None) else _num(body, 'size_cm') / 100.0
    out = {'ok': True, 'preset': preset_name,
           'inputs': {'ceiling': ceiling, 'camera_height': cam, 'hfov': hfov, 'width': width,
                      'height': height, 'speed': speed, 'exposure': exposure},
           'camera_to_ceiling_m': round(p['d'], 3), 'focal_px': round(p['f'], 1),
           'mm_per_px': round(p['gsd'] * 1000, 2),
           'view_w_m': round(p['view_w'], 3), 'view_h_m': round(p['view_h'], 3),
           'blur_px': round(p['blur_px'], 2), 'need_px_per_cell': round(p['need_px'], 2),
           'readable_from_cm': round(p['s_min'] * 100, 1), 'fits_up_to_cm': round(p['s_max'] * 100, 1)}
    r = mp.recommend(p, size_m)
    if r is None:
        out.update(ok=False, error='no size works with this camera at this height: use a higher '
                                   'resolution, a narrower lens or a slower walk')
        return out
    out.update({'size_cm': round(r['size'] * 100, 1), 'chosen': size_m is not None,
                'px_per_cell': round(r['px_per_cell'], 1), 'printed_cm': round(r['printed'] * 100, 1),
                'sheet': r['sheet'], 'zone_radius_m': round(r['zone_radius'], 3)})
    if r['zone_radius'] <= 0:
        out.update(ok=False, error=f'a {out["size_cm"]} cm marker never fits whole in view: '
                                   f'choose at most {out["fits_up_to_cm"]} cm')
        return out
    out['grid_spacing_m'] = round(r['grid_spacing'], 3)
    out['route_spacing_m'] = round(r['route_spacing'], 3)
    out['route_spacing_no_odom_m'] = round(r['route_spacing_no_odom'], 3)
    length = body.get('route_length')
    if length not in ('', None):
        L = _num(body, 'route_length')
        if L < 0:
            raise PlanError('the route length is 0 m or more')
        out['route_length_m'] = L
        out['marker_count'] = mp.route_markers(L, r['route_spacing_no_odom'])
    return out
