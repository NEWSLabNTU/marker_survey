"""The two source implementations' localizer methods, re-implemented for comparison
on the same frames. Not used by the node. No ROS.

- `ceiling_source_fix`: the ceiling implementation. Detection with
  CORNER_REFINE_APRILTAG and cornerSubPix (fixed 2 px half-window, no edge
  fit); every corner's ray cut with a horizontal plane at its marker's height;
  a plain least-squares 2D rigid fit (no robust loss); fits whose corners stay
  more than 5 cm RMS off are dropped; covariance by the number of markers.
- `wall_source_fix`: the wall-board implementation (a C++ node built on a
  larger stack). Each marker's single-view pose (IPPE for a square: the
  solution with the lower reprojection error), markers farther than 5 m
  ignored; each detection is placed in the map with the state estimator's
  pose and matched to the nearest marker of the map, whatever its id; the
  base pose is that map marker's pose times the detection's inverse, for the
  detection closest to its match; dropped when more than 1 m from the
  estimator's pose; covariance a constant (0.05 m^2, 0.01 rad^2), scaled by
  max(1, (d / 5)^3) with d that distance.

Both use the corner positions they are given, so a comparison with the same
detections isolates the pose method; `source_ceiling_detect` gives the
ceiling implementation's own corners.
"""

from typing import List, Optional, Tuple

import numpy as np

from marker_survey.core.detection import Detection, marker_object_points, marker_pose_in_camera
from marker_survey.core.marker_pose import ceiling_points_in_base, fit_pose_2d, position_sigma
from marker_survey.core.se3 import inv_T, make_T, rot_z

WALL_BASE_COV = np.diag([0.05, 0.05, 0.05, 0.01, 0.01, 0.01])


def source_ceiling_detect(gray, border_margin: float = 5.0) -> List[Detection]:
    """tag36h11 detection as the ceiling implementation does it."""
    import cv2
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = (cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, 'DetectorParameters_create')
              else cv2.aruco.DetectorParameters())
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    if hasattr(cv2.aruco, 'ArucoDetector'):
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    out = []
    if ids is None:
        return out
    h, w = gray.shape[:2]
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    for c, i in zip(corners, ids.ravel()):
        c = c.reshape(4, 2)
        if c.min() < border_margin or (c[:, 0] > w - border_margin).any() or \
                (c[:, 1] > h - border_margin).any():
            continue
        pts = c.astype(np.float32).copy()
        cv2.cornerSubPix(gray, pts, (2, 2), (-1, -1), crit)
        out.append(Detection(int(i), pts.astype(float)))
    return out


def ceiling_source_fix(dets: List[Detection], marker_map, K, T_base_cam,
                       max_fit_error: float = 0.05) -> Optional[Tuple[np.ndarray, int]]:
    """(T_map_base, markers used) or None."""
    ids = set(marker_map.ids())
    use = [d for d in dets if d.rejected is None and d.id in ids]
    if not use:
        return None
    pix = np.vstack([d.corners for d in use])
    C = np.vstack([marker_map.corners_of(d.id) for d in use])
    heights = np.repeat([marker_map.get(d.id).z for d in use], 4)
    pts = ceiling_points_in_base(pix, heights, np.asarray(K, float), np.zeros(5),
                                 T_base_cam[:3, :3], T_base_cam[:3, 3])
    if pts is None:
        return None
    x, y, yaw, err = fit_pose_2d(pts, C)
    if err > max_fit_error:
        return None
    return make_T(rot_z(yaw), [x, y, 0.0]), len(use)


def ceiling_source_sigma(count: int) -> float:
    return position_sigma(count)


def _ippe(d: Detection, K, size) -> Optional[np.ndarray]:
    """T_cam_marker: the IPPE solution with the lower reprojection error. If OpenCV's
    IPPE fails (NaN, or the 4.5 bug for a marker at the camera's height), the best
    general solution instead, so that an OpenCV defect does not count against the
    method."""
    import cv2
    obj = marker_object_points(size)
    img = np.asarray(d.corners, float).reshape(-1, 1, 2)
    best, best_e = None, np.inf
    try:
        n, rv, tv, errs = cv2.solvePnPGeneric(obj, img, K, np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE)
        for r, t in zip(rv[:n], tv[:n]):
            if not (np.all(np.isfinite(r)) and np.all(np.isfinite(t))) or t.ravel()[2] <= 0:
                continue
            proj, _ = cv2.projectPoints(obj, r, t, K, np.zeros(5))
            e = float(np.sqrt(np.mean(np.sum((proj - img) ** 2, axis=2))))
            if e < best_e:
                best, best_e = make_T(cv2.Rodrigues(r)[0], t.ravel()), e
    except cv2.error:
        pass
    if best is None or best_e > 2.0:
        T, e = marker_pose_in_camera(d, K, size)
        if np.isfinite(e) and e < best_e:
            best = T
    return best


def wall_source_fix(dets: List[Detection], marker_map, K, T_base_cam, T_prior,
                    distance_threshold: float = 5.0, position_tolerance: float = 1.0):
    """(T_map_base, covariance, distance from the prior) or None (nothing within
    distance_threshold, or the fix farther than position_tolerance from the prior)."""
    K = np.asarray(K, float)
    mapped = list(marker_map.poses().values())
    best = None
    for d in dets:
        if d.rejected is not None:
            continue
        size = marker_map.size_of(d.id) if d.id in marker_map.ids() else marker_map.marker_size
        T_cm = _ippe(d, K, size)
        if T_cm is None or np.linalg.norm(T_cm[:3, 3]) > distance_threshold:
            continue
        T_bm = T_base_cam @ T_cm                       # the detection in the base frame
        p_map = (T_prior @ T_bm)[:3, 3]
        for Tm in mapped:
            dist = float(np.linalg.norm(Tm[:3, 3] - p_map))
            if best is None or dist <= best[0]:
                best = (dist, Tm @ inv_T(T_bm))
    if best is None:
        return None
    T = best[1]
    d = float(np.linalg.norm(T[:3, 3] - T_prior[:3, 3]))
    if d > position_tolerance:
        return None
    return T, WALL_BASE_COV * max(1.0, (d / 5.0) ** 3), d
