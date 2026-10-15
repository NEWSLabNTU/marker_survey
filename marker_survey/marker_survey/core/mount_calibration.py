"""The ceiling camera's mounting from a spin under the markers. No ROS.

A planar localizer assumes the camera's mounting (the URDF): its position in
the robot and its tilt. A tilt of one degree moves every ceiling point by
about 5 cm at a 3 m ceiling, so the mounting must be measured, and a tape
measure cannot measure a tilt.

The robot spins on the spot under one or more markers. Its position stays
the same and its heading is known up to a constant from odometry. If the
assumed mounting were right, every image would place the markers where the
marker map has them; a wrong tilt or position makes them trace small circles
instead. A least-squares fit over all images finds the robot's position and
start heading together with the correction:

    dx, dy   camera position offset in the base frame (m)
    tilt_x   rotation about the base's x axis (rad), roll
    tilt_y   rotation about the base's y axis (rad), pitch

The camera's rotation about the vertical axis cannot be told from the
robot's heading this way; it stays as mounted (the URDF).

Tilt and position are almost interchangeable at one ceiling height: tilting
by an angle a moves every ceiling point by about h * a, as a sideways shift
of the camera would; only perspective (a second-order effect) tells them
apart. With rendered images that is not enough (in simulation a 0.015 rad
tilt came out as 0.004 rad and a shift). So by default the position is taken from
the URDF, measured with a tape measure, and only the tilt is fitted;
fit_offset=True fits both, best with markers at different heights.
"""
import math

import cv2
import numpy as np
from scipy.optimize import least_squares

from marker_survey.core.marker_pose import corner_points, fit_pose_2d
from marker_survey.core.se3 import rot_x, rot_y


def corrected_mounting(r0, t0, dx, dy, tilt_x, tilt_y):
    """The mounting (R, t) after the correction, in the base frame."""
    return rot_y(tilt_y) @ rot_x(tilt_x) @ r0, t0 + np.array([dx, dy, 0.0])


class Observations:
    """Marker corners seen during the spin, as camera rays.

    markers: a marker_map.MarkerMap (version 1: tilted markers and per-marker
    sizes included; marker_size may then be None), or {id: MapMarker}, or the
    older {id: dict with x, y, z, yaw [rad]} of level ceiling markers, with
    marker_size for all. Each corner is cut with the horizontal plane at its
    own map height, so a tilted marker counts with its true corners."""

    def __init__(self, markers, marker_size, camera_matrix, dist_coeffs):
        if hasattr(markers, 'by_id'):                 # a MarkerMap
            self.markers = markers.by_id()
            self.size_of = markers.size_of
        else:
            self.markers = dict(markers)
            self.size_of = lambda mid: self._size(mid, marker_size)
        self.size = marker_size
        self.k, self.d = camera_matrix, dist_coeffs
        self.frames = []    # (odometry yaw, rays N x 3, map corners N x 2, heights N)

    def _size(self, mid, default):
        size = getattr(self.markers[mid], 'size', None)
        if size is None:
            if default is None:
                raise ValueError(f'marker {mid}: no size (give marker_size or a MarkerMap)')
            return float(default)
        return float(size)

    def add(self, yaw, detections):
        """detections: [(marker_id, 4 x 2 pixel corners)] of one image."""
        rays, corners, heights = [], [], []
        for marker_id, pixels in detections:
            marker = self.markers.get(marker_id)
            if marker is None:
                continue
            p = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
            n = cv2.undistortPoints(p, self.k, self.d).reshape(-1, 2)
            rays.append(np.hstack([n, np.ones((len(n), 1))]))
            c = corner_points(marker, self.size_of(marker_id))
            corners.append(c[:, :2])
            heights += list(c[:, 2])
        if rays:
            self.frames.append((yaw, np.vstack(rays), np.vstack(corners), np.array(heights)))

    def __len__(self):
        return len(self.frames)

    def residuals(self, params, r0, t0):
        x0, y0, yaw0, dx, dy, tilt_x, tilt_y = params
        r, t = corrected_mounting(r0, t0, dx, dy, tilt_x, tilt_y)
        out = []
        for yaw, rays, corners, heights in self.frames:
            rays_base = rays @ r.T
            points = t + rays_base * ((heights - t[2]) / rays_base[:, 2])[:, None]
            c, s = math.cos(yaw0 + yaw), math.sin(yaw0 + yaw)
            mx = x0 + c * points[:, 0] - s * points[:, 1]
            my = y0 + s * points[:, 0] + c * points[:, 1]
            out.append(mx - corners[:, 0])
            out.append(my - corners[:, 1])
        return np.concatenate(out)

    def start(self, r0, t0):
        """Robot pose from the first image, with the assumed mounting."""
        yaw, rays, corners, heights = self.frames[0]
        rays_base = rays @ r0.T
        points = t0 + rays_base * ((heights - t0[2]) / rays_base[:, 2])[:, None]
        x, y, map_yaw, _ = fit_pose_2d(points, np.hstack([corners, heights[:, None]]))
        return x, y, map_yaw - yaw


def solve(obs, r0, t0, fit_offset=False):
    """Fit the correction. Returns (dict of results, RMS before, RMS after) in m."""
    if len(obs) < 10:
        raise ValueError(f'only {len(obs)} images with markers; spin under a marker')
    yaws = [f[0] for f in obs.frames]
    if max(yaws) - min(yaws) < math.pi:
        raise ValueError('the robot turned less than half a turn while seeing markers')
    x0, y0, yaw0 = obs.start(r0, t0)
    before = least_squares(lambda p: obs.residuals(np.r_[p, 0, 0, 0, 0], r0, t0),
                           [x0, y0, yaw0])
    rms_before = math.sqrt(np.mean(before.fun ** 2))
    if fit_offset:
        result = least_squares(lambda p: obs.residuals(p, r0, t0),
                               np.r_[before.x, 0, 0, 0, 0],
                               x_scale=[0.1, 0.1, 0.01, 0.01, 0.01, 0.005, 0.005])
        x0, y0, yaw0, dx, dy, tilt_x, tilt_y = result.x
    else:
        result = least_squares(lambda p: obs.residuals(np.r_[p[:3], 0, 0, p[3:]], r0, t0),
                               np.r_[before.x, 0, 0], x_scale=[0.1, 0.1, 0.01, 0.005, 0.005])
        x0, y0, yaw0, tilt_x, tilt_y = result.x
        dx = dy = 0.0
    rms_after = math.sqrt(np.mean(result.fun ** 2))
    return ({'robot_x': x0, 'robot_y': y0, 'dx': dx, 'dy': dy,
             'tilt_x': tilt_x, 'tilt_y': tilt_y}, rms_before, rms_after)
