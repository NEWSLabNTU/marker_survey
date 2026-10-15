import os

import numpy as np
import pytest

from marker_survey.core import marker_map as mm
from marker_survey.core.path_csv import load_path_csv
from marker_survey.tools.synthetic import parse_args, scene, truth_map

ROOT = os.path.join(os.path.dirname(__file__), '..', '..')
SITE = os.path.join(ROOT, 'examples', 'wall_course', 'sites', 'test_course')


def test_a_recording_needs_only_a_marker_map_and_a_route(tmp_path):
    args = parse_args([str(tmp_path), '--map', os.path.join(SITE, 'marker_map.yaml'),
                       '--route', os.path.join(SITE, 'route.csv'), '--laps', '1', '--rate', '3'])
    site, camera, frames = scene(args)
    assert len(site.markers) == 11 and site.marker_size == 0.38
    assert camera.width == 960 and camera.height == 600
    assert len(frames) > 200
    # odometry starts at the first pose; the truth is in the map frame
    assert np.allclose(frames[0].T_odom_base, np.eye(4))
    assert np.allclose(frames[0].T_map_base[:3, 3], [0.0, 0.0, 0.0])


def test_the_truth_in_another_frame_keeps_the_markers(tmp_path):
    site = mm.load(os.path.join(SITE, 'marker_map.yaml'))
    T = np.eye(4)
    T[:3, 3] = [1.0, 2.0, 0.0]
    t = truth_map(site, T, 'moved')
    for a, b in zip(site.markers, t.markers):
        assert np.allclose(b.pose(), T @ a.pose(), atol=1e-6)


def test_route_csv(tmp_path):
    p = tmp_path / 'r.csv'
    p.write_text('# comment\nx,y\n0,0\n1.5,0.0,9\n')
    assert load_path_csv(str(p)) == [(0.0, 0.0), (1.5, 0.0)]
    p.write_text('0,0\n')
    with pytest.raises(ValueError, match='fewer than two'):
        load_path_csv(str(p))
