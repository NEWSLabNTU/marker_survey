import pytest

from marker_survey.core.marker_plan import plan, zone_radius


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
