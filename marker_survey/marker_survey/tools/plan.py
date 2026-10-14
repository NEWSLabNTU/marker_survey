"""marker_plan: choose the ceiling marker size and spacing from measurements.

    marker_plan --ceiling 3.0 --hfov 70
    marker_plan --ceiling 4.2 --hfov 90 --width 1280 --height 720

Measure the ceiling height (laser distance meter) and the camera's horizontal
field of view at the resolution the robot uses. core/marker_plan.py explains
the formulas.
"""
import argparse

from marker_survey.core.marker_plan import plan, recommend


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--ceiling', type=float, required=True, help='ceiling height above the floor, m')
    parser.add_argument('--hfov', type=float, required=True, help='camera horizontal field of view, degrees')
    parser.add_argument('--width', type=int, default=640, help='image width, px')
    parser.add_argument('--height', type=int, default=480, help='image height, px')
    parser.add_argument('--camera-height', type=float, default=0.26, help='camera above the floor, m')
    parser.add_argument('--speed', type=float, default=0.5, help='robot top speed, m/s')
    parser.add_argument('--exposure', type=float, default=0.02, help='camera exposure time, s')
    parser.add_argument('--min-px-per-cell', type=float, default=6.0)
    parser.add_argument('--border-px', type=float, default=5.0, help='image border where markers are ignored')
    parser.add_argument('--drift', type=float, default=0.05,
                        help='odometry error per metre driven without a marker (0.05 = 5 %%)')
    parser.add_argument('--max-drift-error', type=float, default=0.05,
                        help='largest error allowed between markers, m')
    parser.add_argument('--size', type=float, help='evaluate this black square size, m')
    a = parser.parse_args(argv)

    p = plan(a.ceiling, a.camera_height, a.hfov, a.width, a.height, a.speed, a.exposure,
             a.min_px_per_cell, a.border_px, a.drift, a.max_drift_error)
    print(f'camera to ceiling d = {p["d"]:.2f} m, focal length f = {p["f"]:.0f} px, '
          f'{p["gsd"] * 1000:.1f} mm per pixel on the ceiling')
    print(f'camera view on the ceiling: {p["view_w"]:.2f} x {p["view_h"]:.2f} m; '
          f'motion blur {p["blur_px"]:.1f} px, so at least {p["need_px"]:.1f} px per tag cell')
    print(f'black square size S: readable from {p["s_min"] * 100:.0f} cm, '
          f'fits in view up to {p["s_max"] * 100:.0f} cm')
    r = recommend(p, a.size)
    if r is None:
        print('NO SIZE WORKS: use a higher resolution, a narrower lens, or a slower robot.')
        return 1
    print(f'\nchosen S = {r["size"] * 100:.0f} cm: {r["px_per_cell"]:.1f} px per cell, '
          f'printed {r["printed"] * 100:.0f} cm square with margin -> sheet {r["sheet"]}')
    if r['zone_radius'] <= 0:
        print('  this size never fits fully in view')
        return 1
    print(f'  whole marker in view while the camera is within R = {r["zone_radius"]:.2f} m of it')
    print(f'spacing, a marker always in view (square grid): D <= {r["grid_spacing"]:.2f} m')
    print(f'spacing along a route, odometry in between:      D <= {r["route_spacing"]:.2f} m '
          f'(2R + {p["max_gap"]:.2f} m without a marker at {a.drift * 100:.0f} % drift)')
    print('plus one marker within R of every goal and dock')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
