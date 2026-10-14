"""Robot pose from ceiling markers by planar geometry. No ROS.

The robot drives on a flat floor and the camera is bolted on looking up, so
only three numbers are unknown: x, y and yaw. The method uses that:

1. Each marker corner seen in the image is a ray from the camera. Where the
   ray meets the marker's plane is the corner's position relative to the
   robot (ceiling_points_in_base for horizontal planes at known heights,
   plane_points_in_base for any plane).
2. The marker map gives the same corners in the map frame (corner_points).
3. The rotation and shift in the floor plane that best moves the first set of
   points onto the second is the robot's pose (fit_pose_2d).

A full 6-DoF camera pose (cv2.solvePnP) has to guess the camera's tilt too.
With few markers close together, tilting the camera and shifting it look
almost the same: in simulation, two-marker solutions were 36 cm off. Fixing
the tilt to the known mounting removes that ambiguity, also for one marker.
The price: if the robot tilts (a bump, braking), the error grows by about
4.8 cm per degree at a 3 m ceiling.

Frames: map (the site's map), marker (centre of one marker, OpenCV axes),
base (the robot's footprint on the floor), camera (optical frame).
"""
import math

import cv2
import numpy as np

from marker_survey.core.detection import marker_object_points
from marker_survey.core.marker_map import mount_rotation


def marker_axes(yaw):
    """Unit vectors of a ceiling marker's x and y axes in the map frame, for a marker
    rotated by `yaw` [rad] (marker map: mount ceiling, rotation_deg)."""
    R = mount_rotation('ceiling', {'rotation_deg': math.degrees(yaw)})
    return R[:, 0], R[:, 1]


def corner_points(marker, size):
    """Map coordinates (4 x 3) of a marker's corners, in OpenCV's corner order.

    marker: a marker_map.MapMarker, or a dict with x, y, z and yaw [rad] of a
    ceiling marker."""
    if hasattr(marker, 'pose'):
        T = marker.pose()
    else:
        T = np.eye(4)
        T[:3, :3] = mount_rotation('ceiling', {'rotation_deg': math.degrees(marker['yaw'])})
        T[:3, 3] = [marker['x'], marker['y'], marker['z']]
    return marker_object_points(size) @ T[:3, :3].T + T[:3, 3]


def _rays_in_base(pixels, camera_matrix, dist_coeffs, r_base_cam):
    pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
    normalised = cv2.undistortPoints(pixels, camera_matrix, dist_coeffs).reshape(-1, 2)
    rays_cam = np.hstack([normalised, np.ones((len(normalised), 1))])
    return rays_cam @ np.asarray(r_base_cam).T


def ceiling_points_in_base(pixels, heights, camera_matrix, dist_coeffs, r_base_cam, t_base_cam):
    """Where the rays through `pixels` meet horizontal planes at `heights`, in the base frame.

    pixels: N x 2 image points; heights: N plane heights above the floor (m);
    r_base_cam, t_base_cam: the camera's mounting in the base frame. Returns
    N x 3 points, or None if a ray does not point up."""
    rays_base = _rays_in_base(pixels, camera_matrix, dist_coeffs, r_base_cam)
    if np.any(rays_base[:, 2] <= 1e-6):
        return None
    scale = (np.asarray(heights) - t_base_cam[2]) / rays_base[:, 2]
    return t_base_cam + rays_base * scale[:, None]


def plane_points_in_base(pixels, plane_points, plane_normals, camera_matrix, dist_coeffs,
                         r_base_cam, t_base_cam):
    """Where the rays through `pixels` meet planes (a point and a normal each, base frame).
    Returns N x 3 points, or None if a ray is parallel to its plane or the plane is
    behind the camera."""
    rays = _rays_in_base(pixels, camera_matrix, dist_coeffs, r_base_cam)
    p = np.asarray(plane_points, float).reshape(-1, 3)
    n = np.asarray(plane_normals, float).reshape(-1, 3)
    den = np.sum(rays * n, axis=1)
    if np.any(np.abs(den) < 1e-9):
        return None
    s = np.sum((p - t_base_cam) * n, axis=1) / den
    if np.any(s <= 0):
        return None
    return t_base_cam + rays * s[:, None]


def fit_pose_2d(points_base, points_map):
    """Robot (x, y, yaw) in the map that best maps base points onto map points.

    Least-squares rigid fit in the floor plane (2D Kabsch). Returns
    (x, y, yaw, rms) with rms the remaining mismatch in metres."""
    a = np.asarray(points_base, dtype=np.float64)[:, :2]
    b = np.asarray(points_map, dtype=np.float64)[:, :2]
    a_mean, b_mean = a.mean(axis=0), b.mean(axis=0)
    a0, b0 = a - a_mean, b - b_mean
    yaw = math.atan2(np.sum(a0[:, 0] * b0[:, 1] - a0[:, 1] * b0[:, 0]),
                     np.sum(a0[:, 0] * b0[:, 0] + a0[:, 1] * b0[:, 1]))
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    t = b_mean - rotation @ a_mean
    rms = float(np.sqrt(np.mean(np.sum((a @ rotation.T + t - b) ** 2, axis=1))))
    return float(t[0]), float(t[1]), yaw, rms


def position_sigma(marker_count):
    """Expected position error (m, one standard deviation) by markers in view, as
    measured in simulation with this method; real cameras add calibration and tilt
    errors."""
    if marker_count <= 1:
        return 0.15
    if marker_count == 2:
        return 0.08
    return 0.05
