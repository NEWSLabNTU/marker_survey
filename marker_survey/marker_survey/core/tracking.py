"""A camera-only survey: every frame placed on the markers already mapped. No ROS.

With `pose_source: markers` there is no odometry and no TF pose: the markers
themselves are the only pose source (a person walking with a laptop and a
webcam, a robot without odometry).

  - The first frame with a usable marker starts the map: its camera pose is
    the origin marker's frame (frame mode origin_marker, the origin marker in
    view), or the identity; every marker in it is mapped from there.
  - Every later frame is placed by PnP on all the mapped markers it sees
    (place_camera): each marker's single-view solutions as candidates, the
    one that explains all corners best (with one marker, the planar target's
    two solutions are told apart by the previous pose, when it is recent), a
    robust refinement over every corner, a marker that does not fit left out.
  - A frame with no mapped marker in view cannot be placed: it is skipped,
    and new markers in it stay unmapped (the tracking is `lost`) until a
    frame sees them together with a mapped one.

The bundle adjustment afterwards has no motion factor: only the markers link
the keyframes. The placement only starts it; the solve redoes every pose.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from marker_survey.core import frames as fr
from marker_survey.core.detection import Detection, marker_object_points, marker_pose_in_camera
from marker_survey.core.se3 import inv_T, make_T, rotation_angle, rotvec_to_matrix


@dataclass
class Placement:
    T_world_cam: np.ndarray
    ids: List[int]                       # mapped markers the pose rests on
    rms_px: float
    candidates: int = 1                  # plausible solutions before the previous pose chose
    dropped: Dict[int, str] = field(default_factory=dict)


@dataclass
class TrackSettings:
    outlier_px: float = 3.0              # a marker fitting worse is left out, the rest refit
    max_rms_px: float = 2.0              # a placement fitting worse is refused
    robust_px: float = 1.0               # robust loss scale [px]
    ambiguity_ratio: float = 3.0         # solutions within this factor of the best...
    ambiguity_px: float = 1.0            # ...or under this many px are equally plausible
    max_gap: float = 1.0                 # [s] the previous pose decides an ambiguity if this recent


def _reprojection(T_world_cam, corners_w, dets, K):
    T_cw = inv_T(T_world_cam)
    out = []
    for X, d in zip(corners_w, dets):
        Xc = X @ T_cw[:3, :3].T + T_cw[:3, 3]
        z = np.maximum(Xc[:, 2:3], 1e-3)
        uv = (Xc @ K.T)[:, :2] / z
        r = (uv - d.corners).ravel()
        if np.any(Xc[:, 2] <= 1e-3):
            r = r + 1e3
        out.append(r)
    return np.concatenate(out)


def _rms(v) -> float:
    v = np.asarray(v, float).ravel()
    return float(np.sqrt(np.mean(v ** 2))) if len(v) else 0.0


def _refine(T0, corners_w, dets, K, robust_px):
    from scipy.optimize import least_squares
    R0, t0 = T0[:3, :3], T0[:3, 3]

    def pose(p):
        return make_T(rotvec_to_matrix(p[:3]) @ R0, t0 + p[3:])

    sol = least_squares(lambda p: _reprojection(pose(p), corners_w, dets, K), np.zeros(6),
                        method='trf', loss='soft_l1', f_scale=robust_px, x_scale='jac', max_nfev=50)
    return pose(sol.x)


def single_view_poses(d: Detection, T_world_marker, size, K) -> List[np.ndarray]:
    """Camera poses in the world from one marker: both IPPE solutions and the best
    of the general solvers (detection.marker_pose_in_camera)."""
    import cv2
    obj = marker_object_points(size)
    img = np.asarray(d.corners, float).reshape(-1, 1, 2)
    T_cms = []
    try:
        n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, img, K, np.zeros(5),
                                                 flags=cv2.SOLVEPNP_IPPE_SQUARE)
        for r, t in zip(rvecs[:n], tvecs[:n]):
            if np.all(np.isfinite(r)) and np.all(np.isfinite(t)) and t.ravel()[2] > 0:
                T_cms.append(make_T(cv2.Rodrigues(r)[0], t.ravel()))
    except cv2.error:
        pass
    T, err = marker_pose_in_camera(d, K, size)
    if np.isfinite(err) and np.all(np.isfinite(T)):
        T_cms.append(T)
    return [T_world_marker @ inv_T(T_cm) for T_cm in T_cms]


def place_camera(detections: List[Detection], known: Dict[int, np.ndarray], size_of, K,
                 T_prev: Optional[np.ndarray] = None,
                 settings: Optional[TrackSettings] = None) -> Optional[Placement]:
    """The camera pose T_world_cam from the mapped markers (`known`: id -> T_world_marker)
    in view; None if none is in view or the corners do not fit. `size_of(id)` gives a
    marker's size; `T_prev`, a recent camera pose, decides between the two solutions
    of a single marker."""
    s = settings or TrackSettings()
    K = np.asarray(K, float)
    use = [d for d in detections if d.id in known and d.rejected is None]
    if not use:
        return None
    dropped: Dict[int, str] = {}
    while True:
        corners_w = [marker_object_points(size_of(d.id)) @ known[d.id][:3, :3].T + known[d.id][:3, 3]
                     for d in use]
        cands = [T for d in use for T in single_view_poses(d, known[d.id], size_of(d.id), K)]
        if not cands:
            return None

        def cost(T):
            r = _reprojection(T, corners_w, use, K) / s.robust_px
            return float(np.sum(2 * (np.sqrt(1 + r ** 2) - 1)))

        cands.sort(key=cost)
        refined = []
        for T0 in cands[:4]:
            T = _refine(T0, corners_w, use, K, s.robust_px)
            e = _rms(_reprojection(T, corners_w, use, K))
            if all(np.linalg.norm(T[:3, 3] - U[:3, 3]) > 1e-3 or
                   rotation_angle(T[:3, :3], U[:3, :3]) > 1e-3 for U, _ in refined):
                refined.append((T, e))
        best_e = min(e for _, e in refined)
        plausible = [c for c in refined if c[1] <= max(s.ambiguity_px, s.ambiguity_ratio * best_e)]
        if T_prev is not None and len(plausible) > 1:
            T, e = min(plausible, key=lambda c: np.linalg.norm(c[0][:3, 3] - T_prev[:3, 3])
                       + rotation_angle(c[0][:3, :3], T_prev[:3, :3]))
        else:
            T, e = min(refined, key=lambda c: c[1])
        r = _reprojection(T, corners_w, use, K).reshape(len(use), 8)
        per = [_rms(x) for x in r]
        worst = int(np.argmax(per))
        if len(use) > 1 and per[worst] > s.outlier_px:
            dropped[use[worst].id] = f'outlier ({per[worst]:.1f} px)'
            use = use[:worst] + use[worst + 1:]
            continue
        break
    if e > s.max_rms_px:
        return None
    return Placement(T, [d.id for d in use], e, len(plausible), dropped)


def start_pose(detections: List[Detection], origin_marker: Optional[int], size_of, K,
               T_cam_base: Optional[np.ndarray], floor_offset: float = 0.0,
               vertical: str = 'base') -> np.ndarray:
    """The first frame's camera pose: in the origin marker's frame (origin_marker mode,
    the origin marker in view; the floor from this one frame), else the identity."""
    d = next((d for d in detections if d.id == origin_marker and d.rejected is None), None)
    if d is None:
        return np.eye(4)
    T_co, err = marker_pose_in_camera(d, K, size_of(d.id))
    if not (np.isfinite(err) and np.all(np.isfinite(T_co))):
        return np.eye(4)
    T_cb = np.eye(4) if T_cam_base is None else T_cam_base
    if vertical == 'ceiling':
        n = -T_co[:3, 2]
    else:
        n = T_cb[:3, 2]
    p = T_cb[:3, 3] - float(floor_offset) * n
    return inv_T(fr.origin_marker_frame(T_co, p, n))


class MarkerTracker:
    """Places the frames of a camera-only survey and keeps the tracking state the
    survey node reports: ok while the last frame was placed, lost (since when, the
    last mapped marker seen) when it could not be."""

    def __init__(self, survey, origin_marker: Optional[int] = None, floor_offset: float = 0.0,
                 vertical: str = 'base', settings: Optional[TrackSettings] = None):
        self.survey = survey
        self.origin_marker = origin_marker
        self.floor_offset = floor_offset
        self.vertical = vertical
        self.s = settings or TrackSettings()
        self.last_T = None              # (stamp, T_world_cam) of the last placed frame
        self.last_ok = None             # stamp of the last placed frame
        self.first_stamp = None
        self.last_stamp = None
        self.ok = False
        self.last_known_marker = None
        self.placed = 0
        self.unplaced = 0
        self.last_placement = None

    def track(self, stamp: float, detections: List[Detection]) -> Optional[np.ndarray]:
        """The camera pose of a frame, or None: no mapped marker in view (the frame is
        then skipped)."""
        sv = self.survey
        if self.first_stamp is None:
            self.first_stamp = stamp
        self.last_stamp = stamp
        usable = [d for d in detections if sv.usable(d)]
        T = None
        self.last_placement = None
        if not sv.markers:
            if usable:
                T = start_pose(usable, self.origin_marker, sv.s.size_of, sv.K, sv.T_cam_base,
                               self.floor_offset, self.vertical)
        else:
            prev = None
            if self.last_T is not None and stamp - self.last_T[0] <= self.s.max_gap:
                prev = self.last_T[1]
            known = {mid: m.T_world_marker for mid, m in sv.markers.items()}
            pl = place_camera(usable, known, sv.s.size_of, sv.K, prev, self.s)
            if pl is not None:
                T = pl.T_world_cam
                self.last_placement = pl
                # the largest of them in the image: what the person sees
                big = max((d for d in usable if d.id in pl.ids), key=lambda d: d.area())
                self.last_known_marker = int(big.id)
        if T is None:
            self.ok = False
            self.unplaced += 1
            return None
        if not sv.markers and usable:
            self.last_known_marker = int(max(usable, key=lambda d: d.area()).id)
        self.ok = True
        self.placed += 1
        self.last_ok = stamp
        self.last_T = (stamp, T)
        return T

    def status(self) -> dict:
        """tracking ok or lost; lost_since: seconds since the last placed frame (since
        the first frame if none was), None while ok; the last mapped marker in view."""
        if self.ok or self.last_stamp is None:
            lost_since = None
        else:
            ref = self.last_ok if self.last_ok is not None else self.first_stamp
            lost_since = round(max(0.0, self.last_stamp - ref), 2)
        return {'tracking': 'ok' if self.ok or self.last_stamp is None else 'lost',
                'lost_since': lost_since, 'last_known_marker': self.last_known_marker,
                'placed': self.placed, 'unplaced': self.unplaced}
