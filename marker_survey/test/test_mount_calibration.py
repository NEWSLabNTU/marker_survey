import math

import cv2
import numpy as np
import pytest

from marker_survey.core.marker_pose import corner_points
from marker_survey.core.mount_calibration import Observations, corrected_mounting, solve
from marker_survey.core.se3 import rot_y

K = np.array([[450.0, 0, 320], [0, 450.0, 240], [0, 0, 1]])
D = np.zeros(5)
MARKERS = {5: {'id': 5, 'x': 0.0, 'y': -0.5, 'z': 3.0, 'yaw': 0.0},
           7: {'id': 7, 'x': 0.8, 'y': 0.4, 'z': 3.0, 'yaw': 0.4}}
SIZE = 0.45
# Nominal mounting: optical axis up, as the URDF (link pitched -90 deg, then optical)
R_LINK = rot_y(-math.pi / 2)
R_LINK_OPT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)   # rpy(-pi/2, 0, -pi/2)
R0 = R_LINK @ R_LINK_OPT
T0 = np.array([-0.12, 0.0, 0.26])


def spin(r_true, t_true, robot=(0.3, 0.1), yaw0=0.5, turns=1.0, steps=90):
    obs = Observations(MARKERS, SIZE, K, D)
    for i in range(steps):
        yaw = 2 * math.pi * turns * i / steps
        c, s = math.cos(yaw0 + yaw), math.sin(yaw0 + yaw)
        r_map_base = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        t_map_base = np.array([robot[0], robot[1], 0.0])
        r_map_cam = r_map_base @ r_true
        t_map_cam = t_map_base + r_map_base @ t_true
        detections = []
        for mid, m in MARKERS.items():
            pts = corner_points(m, SIZE)
            cam = (pts - t_map_cam) @ r_map_cam          # map -> camera
            if np.any(cam[:, 2] <= 0):
                continue
            px, _ = cv2.projectPoints(cam, np.zeros(3), np.zeros(3), K, D)
            px = px.reshape(-1, 2)
            if np.all((px >= 0) & (px < [640, 480])):
                detections.append((mid, px))
        obs.add(yaw, detections)
    return obs


def test_right_mounting_needs_no_correction():
    result, before, after = solve(spin(R0, T0), R0, T0)
    assert before < 1e-4
    assert abs(result['tilt_x']) < 1e-4 and abs(result['tilt_y']) < 1e-4


@pytest.mark.parametrize('tx,ty', [(0.02, -0.015), (-0.01, 0.0)])
def test_finds_tilt(tx, ty):
    r_true, t_true = corrected_mounting(R0, T0, 0.0, 0.0, tx, ty)
    result, before, after = solve(spin(r_true, t_true), R0, T0)
    assert result['tilt_x'] == pytest.approx(tx, abs=2e-4)
    assert result['tilt_y'] == pytest.approx(ty, abs=2e-4)
    assert after < before / 10


@pytest.mark.parametrize('dx,dy,tx,ty', [(0.0, 0.0, 0.02, -0.015), (0.03, -0.02, -0.01, 0.0)])
def test_finds_tilt_and_offset_without_noise(dx, dy, tx, ty):
    r_true, t_true = corrected_mounting(R0, T0, dx, dy, tx, ty)
    result, before, after = solve(spin(r_true, t_true), R0, T0, fit_offset=True)
    assert result['tilt_x'] == pytest.approx(tx, abs=5e-4)
    assert result['tilt_y'] == pytest.approx(ty, abs=5e-4)
    assert result['dx'] == pytest.approx(dx, abs=3e-3)
    assert result['dy'] == pytest.approx(dy, abs=3e-3)
    assert after < before / 10


def test_too_little_turning_is_refused():
    with pytest.raises(ValueError):
        solve(spin(R0, T0, turns=0.3, steps=20), R0, T0)
