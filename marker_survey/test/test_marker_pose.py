import math

import cv2
import numpy as np
import pytest

from marker_survey.core.marker_pose import (
    ceiling_points_in_base, corner_points, fit_pose_2d, position_sigma)

K = np.array([[457.85, 0, 320], [0, 457.85, 240], [0, 0, 1.0]])
D = np.zeros(5)
MARKERS = [{'id': 0, 'x': 0.75, 'y': 0.0, 'z': 3.0, 'yaw': 0.5},
           {'id': 1, 'x': -0.75, 'y': 0.0, 'z': 3.0, 'yaw': -1.0},
           {'id': 2, 'x': 0.75, 'y': 1.5, 'z': 3.0, 'yaw': 2.0}]

# The ceiling camera's mounting in the base frame, as in the URDF: on top,
# looking up; image right is the robot's right, image down is its front.
R_BASE_CAM = np.column_stack([[0, -1, 0], [1, 0, 0], [0, 0, 1]]).astype(float)
T_BASE_CAM = np.array([-0.12, 0.0, 0.26])


def image_of(markers, x, y, yaw):
    """Pixels of the markers' corners seen by the camera of a robot at (x, y, yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    r_map_base = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    r_map_cam = r_map_base @ R_BASE_CAM
    t_map_cam = r_map_base @ T_BASE_CAM + [x, y, 0]
    points = np.vstack([corner_points(m, 0.3) for m in markers])
    rvec, _ = cv2.Rodrigues(r_map_cam.T)
    pixels, _ = cv2.projectPoints(points, rvec, -r_map_cam.T @ t_map_cam, K, D)
    return pixels.reshape(-1, 2), points


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@pytest.mark.parametrize('markers', [MARKERS, MARKERS[:1]], ids=['three markers', 'one marker'])
@pytest.mark.parametrize('pose', [(0.0, 0.0, 0.0), (0.3, 0.4, 1.2), (-0.2, 0.6, -2.5)])
def test_pose_recovered(markers, pose):
    pixels, map_points = image_of(markers, *pose)
    heights = map_points[:, 2]
    base_points = ceiling_points_in_base(pixels, heights, K, D, R_BASE_CAM, T_BASE_CAM)
    x, y, yaw, rms = fit_pose_2d(base_points, map_points)
    assert rms < 1e-6
    assert (x, y) == pytest.approx(pose[:2], abs=1e-6)
    assert wrap(yaw - pose[2]) == pytest.approx(0, abs=1e-6)


def test_corners_are_a_square_of_the_right_size():
    corners = corner_points(MARKERS[0], 0.3)
    sides = [np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)]
    assert sides == pytest.approx([0.3] * 4)
    assert np.allclose(corners[:, 2], 3.0)


def test_fit_reports_mismatch():
    square = np.array([[0, 0], [1, 0], [1, 1], [0, 1.0]])
    bent = square + [[0, 0], [0, 0], [0.1, 0], [0, 0]]
    assert fit_pose_2d(square, bent)[3] > 0.01


def test_fewer_markers_means_less_trust():
    assert position_sigma(1) > position_sigma(2) > position_sigma(3)
