import pytest

from marker_survey.core.marker_plan import plan, recommend, route_markers, zone_radius
from marker_survey.tools.plan import main


def office():
    # 3 m ceiling, 640 x 480 webcam with a 70 degree lens, 0.5 m/s, 20 ms exposure
    return plan(ceiling=3.0, camera_height=0.26, hfov_deg=70, width=640, height=480,
                speed=0.5, exposure=0.02, min_px_per_cell=6, border_px=5,
                drift=0.05, max_drift_error=0.05)


def test_camera_view_on_the_ceiling():
    p = office()
    assert p['view_w'] == pytest.approx(3.84, abs=0.01)
    assert p['view_h'] == pytest.approx(2.88, abs=0.01)


def test_size_range_contains_the_simulated_marker():
    p = office()
    assert p['s_min'] < 0.45 < p['s_max']


def test_bigger_markers_shrink_the_zone():
    p = office()
    assert zone_radius(p, 0.45) > zone_radius(p, 1.0) > 0


def handheld():
    # 2.9 m ceiling, a webcam held 1.0 m above the floor, 640 x 480 at 70 degrees
    return plan(ceiling=2.9, camera_height=1.0, hfov_deg=70, width=640, height=480,
                speed=0.5, exposure=0.02, min_px_per_cell=6, border_px=5,
                drift=0.05, max_drift_error=0.05)


def test_route_spacing_without_odometry_is_twice_the_zone():
    r = recommend(handheld(), 0.40)
    assert r['route_spacing_no_odom'] == pytest.approx(2 * r['zone_radius'])
    assert r['route_spacing_no_odom'] == pytest.approx(1.39, abs=0.01)
    assert r['route_spacing'] > r['route_spacing_no_odom']
    assert r['zone_radius_long'] > r['zone_radius']


def test_route_markers():
    assert route_markers(30.0, 1.4) == 23
    assert route_markers(28.0, 1.4) == 21          # one at each end
    assert route_markers(0.0, 1.4) == 1
    assert route_markers(10.0, None) is None


def test_cli_handheld_preset_and_route_count(capsys):
    assert main(['--ceiling', '2.9', '--hfov', '70', '--handheld', '--size', '0.40',
                 '--route-length', '30']) == 0
    out = capsys.readouterr().out
    assert 'camera to ceiling d = 1.90 m' in out
    assert 'no odometry (handheld):   D <= 1.39 m' in out
    assert 'route of 30.0 m at D = 1.39 m: 23 markers' in out
    # an explicit camera height wins over the preset
    assert main(['--ceiling', '2.9', '--hfov', '70', '--handheld', '--camera-height', '0.9']) == 0
    assert 'camera to ceiling d = 2.00 m' in capsys.readouterr().out
