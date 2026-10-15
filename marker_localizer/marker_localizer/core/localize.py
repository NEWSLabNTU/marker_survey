"""Robot pose from the markers in one image, 6-DoF or planar. No ROS.

Two solvers share the input (detections, the marker map, the camera's
intrinsics and its mounting on the robot) and the output (a Fix: the base
pose in the map frame, its covariance, the residual of every corner):

- `3d`: every marker's single-view solutions (IPPE: a square seen once has
  two), the one that explains all corners best as the start, then a robust
  Levenberg-Marquardt over the six pose parameters on the reprojection of
  every corner of every marker. One marker alone keeps the planar-target
  ambiguity: with a prior pose, the solution closer to it is kept among those
  that reproject about as well.
- `planar`: the robot drives on a flat floor, so z, roll and pitch are known
  (roll and pitch from an IMU, a TF, or zero) and only x, y and yaw are
  fitted. Each corner's ray is cut with its own marker's plane from the map
  (uneven ceilings, tilted or wall-mounted boards), and the rigid motion in
  the floor plane that brings those points onto the corners' map positions
  is fitted, robustly, with the residuals weighted to pixels.

With few markers close together a 6-DoF solve trades the camera's tilt for a
shift (36 cm off with two ceiling markers in the source implementation's
measurements); the planar fit has no tilt to trade.

Covariance: the fit's own, from the Jacobian of the pixel-weighted
residuals, so it grows with distance and shrinks with the number of markers
and corners; plus, in planar mode, what an unknown tilt of the robot does at
the markers' distance; then floor values, so that a state estimator averages
the fixes instead of jumping to each one.

Robust loss: scipy's `soft_l1` (Huber-like: quadratic near zero, linear far
out), not its `huber`, which zeroes the Jacobian rows of residuals in its
linear zone and lets them drop out of the solve (phase 0).

Gate: a fix farther than `gate_distance` or `gate_angle_deg` from a prior
pose (the state estimator's) is rejected: a misread id, a moved marker, a far
marker with a poor angle.

Frames: map (the marker map's frame), base (the robot), camera (optical:
x right, y down, z forward). T_a_b maps b-frame points into frame a. ROS
covariance order: x, y, z, rotation about x, y, z (of the map frame).
"""

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from marker_survey.core.detection import Detection, marker_object_points
from marker_survey.core.se3 import inv_T, make_T, rot_x, rot_y, rot_z, rotation_angle, rotvec_to_matrix

MODES = ('3d', 'planar')
UNMEASURED_VARIANCE = 1e3       # z, roll and pitch of a planar fix: not measured


@dataclass
class LocalizerSettings:
    mode: str = 'planar'
    pixel_sd: float = 0.5            # [px] corner noise the covariance assumes
    robust_px: float = 1.0           # [px] where the robust loss turns linear
    max_rms_px: float = 2.0          # a fix whose corners fit worse is dropped
    outlier_px: float = 3.0          # a marker fitting worse than this is dropped, the rest refit
    max_distance: float = 0.0        # [m] ignore markers farther from the camera; 0: no limit
    max_position_sd: float = 0.25    # [m] a fix less certain than this is dropped; 0: keep all
    min_sd: float = 0.01             # [m] floor of the position sd
    min_sd_deg: float = 0.3          # floor of the angle sd
    tilt_sd_deg: float = 0.2         # planar: the robot's roll and pitch uncertainty
    base_height: float = 0.0         # planar: [m] base_frame's height above the map's z = 0
    ambiguity_ratio: float = 3.0     # 3d, one marker: solutions reprojecting within this
    ambiguity_px: float = 1.0        # factor (or under this many px) of the best are ambiguous
    gate_distance: float = 1.0       # [m]
    gate_angle_deg: float = 15.0

    def check(self):
        if self.mode not in MODES:
            raise ValueError(f'mode {self.mode!r} is not one of {", ".join(MODES)}')
        if self.pixel_sd <= 0 or self.robust_px <= 0:
            raise ValueError('pixel_sd and robust_px must be positive')
        if self.gate_distance <= 0 or self.gate_angle_deg <= 0:
            raise ValueError('gate_distance and gate_angle_deg must be positive')


@dataclass
class Fix:
    T_map_base: np.ndarray                     # 4 x 4
    covariance: np.ndarray                     # 6 x 6, ROS order
    mode: str
    ids: List[int]                             # markers in the fit
    rms_px: float                              # corner reprojection RMS of the fit
    residuals: Dict[int, np.ndarray]           # per marker, per corner: px (3d) or m (planar)
    projected: Dict[int, np.ndarray]           # per marker, its map corners projected (4 x 2 px)
    dropped: Dict[int, str] = field(default_factory=dict)   # markers left out, and why
    candidates: int = 1                        # 3d: plausible solutions before the prior chose
    residuals_px: Dict[int, np.ndarray] = field(default_factory=dict)   # per corner, px

    @property
    def x(self) -> float:
        return float(self.T_map_base[0, 3])

    @property
    def y(self) -> float:
        return float(self.T_map_base[1, 3])

    @property
    def yaw(self) -> float:
        return yaw_of(self.T_map_base[:3, :3])


@dataclass
class Result:
    fix: Optional[Fix]
    reason: str = ''                           # why there is no fix, or why it was gated
    unknown: List[int] = field(default_factory=list)       # decoded ids not in the map
    dropped: Dict[int, str] = field(default_factory=dict)  # markers not used, and why


def yaw_of(R) -> float:
    R = np.asarray(R, float)
    return math.atan2(R[1, 0], R[0, 0])


def roll_pitch_of(R) -> Tuple[float, float]:
    """Roll and pitch of a rotation R = Rz(yaw) Ry(pitch) Rx(roll)."""
    R = np.asarray(R, float)
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    return roll, pitch


def planar_T(x, y, yaw, roll=0.0, pitch=0.0, z=0.0) -> np.ndarray:
    return make_T(rot_z(yaw) @ rot_y(pitch) @ rot_x(roll), [x, y, z])


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Localizer:
    """Pose fixes from detections against one marker map, for one camera."""

    def __init__(self, marker_map, K, T_base_cam, settings: LocalizerSettings = None):
        self.s = settings or LocalizerSettings()
        self.s.check()
        self.map = marker_map
        self.K = np.asarray(K, float)
        self.Kinv = np.linalg.inv(self.K)
        self.T_base_cam = np.asarray(T_base_cam, float)
        self.T_cam_base = inv_T(self.T_base_cam)
        self.poses = marker_map.poses()
        self.sizes = marker_map.sizes()
        self.corners = {mid: marker_map.corners_of(mid) for mid in self.poses}

    # ------------------------------------------------------------ the entry point
    def locate(self, detections: List[Detection], T_prior: Optional[np.ndarray] = None,
               roll: float = 0.0, pitch: float = 0.0) -> Result:
        """A fix from one image's detections (pinhole pixels: undistorted corners).

        T_prior: the state estimator's base pose at the image's time, or None; it
        resolves the single-marker ambiguity (3d) and gates the fix. roll, pitch: the
        robot's tilt [rad] (planar)."""
        res = Result(None)
        use = []
        for d in detections:
            if d.rejected is not None:
                res.dropped[d.id] = d.rejected
            elif d.id not in self.poses:
                res.unknown.append(d.id)
            else:
                use.append(d)
        if self.s.max_distance > 0 and use:
            keep = []
            for d in use:
                dist = self.distance_estimate(d)
                if dist > self.s.max_distance:
                    res.dropped[d.id] = f'far ({dist:.1f} m)'
                else:
                    keep.append(d)
            use = keep
        if not use:
            res.reason = 'no marker of the map in view'
            return res
        while True:
            fix = (self.solve_3d(use, T_prior) if self.s.mode == '3d'
                   else self.solve_planar(use, roll, pitch))
            if fix is None:
                res.reason = 'no solution (a ray parallel to its marker, or behind the camera)'
                return res
            worst = max(fix.ids, key=lambda i: rms(fix.residuals_px[i]))
            if len(use) > 1 and rms(fix.residuals_px[worst]) > self.s.outlier_px:
                res.dropped[worst] = f'outlier ({rms(fix.residuals_px[worst]):.1f} px)'
                use = [d for d in use if d.id != worst]
                continue
            break
        fix.dropped = dict(res.dropped)
        if fix.rms_px > self.s.max_rms_px:
            res.reason = f'corners fit {fix.rms_px:.1f} px RMS, over max_rms_px {self.s.max_rms_px}'
            res.fix = fix
            return res
        res.fix = fix
        sd = math.sqrt(max(fix.covariance[0, 0], fix.covariance[1, 1]))
        if self.s.max_position_sd > 0 and sd > self.s.max_position_sd:
            res.reason = f'uncertain: position sd {sd:.2f} m, over max_position_sd {self.s.max_position_sd}'
            return res
        if T_prior is not None:
            g = self.gate(fix, T_prior)
            if g:
                res.reason = g
        return res

    def accepted(self, res: Result) -> bool:
        return res.fix is not None and not res.reason

    def gated(self, res: Result) -> bool:
        """A good fix that only the gate turned down: far from the prior. What a
        relocalizer recovering from a wrong estimate (a kidnapped robot) needs."""
        return res.fix is not None and res.reason.startswith('gated')

    def gate(self, fix: Fix, T_prior: np.ndarray) -> str:
        """'' if the fix agrees with the prior pose, else why not."""
        d = fix.T_map_base[:3, 3] - T_prior[:3, 3]
        if self.s.mode == 'planar':
            dist = float(np.hypot(d[0], d[1]))
            ang = abs(wrap(fix.yaw - yaw_of(T_prior[:3, :3])))
        else:
            dist = float(np.linalg.norm(d))
            ang = rotation_angle(fix.T_map_base[:3, :3], T_prior[:3, :3])
        if dist > self.s.gate_distance:
            return f'gated: {dist:.2f} m from the prior (gate_distance {self.s.gate_distance})'
        if math.degrees(ang) > self.s.gate_angle_deg:
            return (f'gated: {math.degrees(ang):.1f} deg from the prior '
                    f'(gate_angle_deg {self.s.gate_angle_deg})')
        return ''

    # ------------------------------------------------------------ helpers
    def distance_estimate(self, d: Detection) -> float:
        """Rough camera-to-marker distance from the marker's apparent size [m]."""
        edge = np.mean([np.linalg.norm(d.corners[i] - d.corners[(i + 1) % 4]) for i in range(4)])
        return float(self.sizes[d.id] * self.K[0, 0] / max(edge, 1e-6))

    def project(self, T_map_base: np.ndarray, X: np.ndarray) -> np.ndarray:
        T_cam_map = self.T_cam_base @ inv_T(T_map_base)
        Xc = X @ T_cam_map[:3, :3].T + T_cam_map[:3, 3]
        uv = Xc @ self.K.T
        return uv[:, :2] / uv[:, 2:3]

    def finish(self, T: np.ndarray, use: List[Detection], cov: np.ndarray, residuals_m=None,
               candidates: int = 1) -> Fix:
        res_px, projected = {}, {}
        for d in use:
            p = self.project(T, self.corners[d.id])
            projected[d.id] = p
            res_px[d.id] = np.linalg.norm(p - d.corners, axis=1)
        all_px = np.concatenate([res_px[d.id] for d in use])
        return Fix(T, cov, self.s.mode, [d.id for d in use], rms(all_px),
                   residuals_m if residuals_m is not None else res_px, projected,
                   candidates=candidates, residuals_px=res_px)

    def floor(self, cov6: np.ndarray, planar: bool) -> np.ndarray:
        c = cov6.copy()
        lo, lo_r = self.s.min_sd ** 2, math.radians(self.s.min_sd_deg) ** 2
        idx = [0, 1, 5] if planar else range(6)
        for i in idx:
            c[i, i] += lo if i < 3 else lo_r
        return c

    # ------------------------------------------------------------ 3d
    def single_view(self, d: Detection) -> List[np.ndarray]:
        """Base poses in the map from one marker alone: both IPPE solutions, and the
        best of the general solvers (OpenCV 4.5's IPPE fails for some views)."""
        import cv2
        from marker_survey.core.detection import marker_pose_in_camera
        obj = marker_object_points(self.sizes[d.id])
        img = np.asarray(d.corners, float).reshape(-1, 1, 2)
        T_cms = []
        try:
            n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, img, self.K, np.zeros(5),
                                                     flags=cv2.SOLVEPNP_IPPE_SQUARE)
            for r, t in zip(rvecs[:n], tvecs[:n]):
                if np.all(np.isfinite(r)) and np.all(np.isfinite(t)) and t.ravel()[2] > 0:
                    T_cms.append(make_T(cv2.Rodrigues(r)[0], t.ravel()))
        except cv2.error:
            pass
        T, err = marker_pose_in_camera(d, self.K, self.sizes[d.id])
        if np.isfinite(err):
            T_cms.append(T)
        return [self.poses[d.id] @ inv_T(T_cm) @ self.T_cam_base for T_cm in T_cms]

    def reprojection(self, T_map_base, use: List[Detection]) -> np.ndarray:
        """Corner reprojection errors (2N, px) for a pose; behind-camera corners count
        as far off."""
        out = []
        T_cam_map = self.T_cam_base @ inv_T(T_map_base)
        for d in use:
            X = self.corners[d.id] @ T_cam_map[:3, :3].T + T_cam_map[:3, 3]
            z = np.maximum(X[:, 2:3], 1e-3)
            uv = (X @ self.K.T)[:, :2] / z
            r = (uv - d.corners).ravel()
            if np.any(X[:, 2] <= 1e-3):
                r = r + 1e3
            out.append(r)
        return np.concatenate(out)

    def refine_3d(self, T0: np.ndarray, use: List[Detection]) -> Tuple[np.ndarray, object]:
        from scipy.optimize import least_squares
        R0, t0 = T0[:3, :3], T0[:3, 3]

        def pose(p):
            return make_T(rotvec_to_matrix(p[:3]) @ R0, t0 + p[3:])

        sol = least_squares(lambda p: self.reprojection(pose(p), use), np.zeros(6), method='trf',
                            loss='soft_l1', f_scale=self.s.robust_px, x_scale='jac', max_nfev=50)
        return pose(sol.x), sol

    def cov_3d(self, sol, n_res: int) -> np.ndarray:
        J = sol.jac
        r = sol.fun
        sd = max(self.s.pixel_sd, float(np.sqrt(np.mean(r ** 2))) if n_res else 0.0)
        try:
            C = sd ** 2 * np.linalg.inv(J.T @ J)
        except np.linalg.LinAlgError:
            C = np.eye(6) * 1e2
        # parameters (rotation about the map's axes, translation) -> ROS (translation, rotation)
        P = np.zeros((6, 6))
        P[0:3, 3:6] = np.eye(3)
        P[3:6, 0:3] = np.eye(3)
        return self.floor(P @ C @ P.T, planar=False)

    def solve_3d(self, use: List[Detection], T_prior=None) -> Optional[Fix]:
        cands = [T for d in use for T in self.single_view(d)]
        if not cands:
            return None

        def cost(T):
            r = self.reprojection(T, use) / self.s.robust_px
            return float(np.sum(2 * (np.sqrt(1 + r ** 2) - 1)))

        cands.sort(key=cost)
        if len(use) == 1:
            # the planar-target ambiguity: refine every distinct solution
            refined = []
            for T0 in cands[:4]:
                T, sol = self.refine_3d(T0, use)
                e = rms(self.reprojection(T, use))
                if all(np.linalg.norm(T[:3, 3] - U[:3, 3]) > 1e-3 or
                       rotation_angle(T[:3, :3], U[:3, :3]) > 1e-3 for U, _, _ in refined):
                    refined.append((T, sol, e))
            best_e = min(e for _, _, e in refined)
            plausible = [c for c in refined
                         if c[2] <= max(self.s.ambiguity_px, self.s.ambiguity_ratio * best_e)]
            if T_prior is not None and len(plausible) > 1:
                T, sol, _ = min(plausible, key=lambda c: pose_distance(c[0], T_prior))
            else:
                T, sol, _ = min(refined, key=lambda c: c[2])
            return self.finish(T, use, self.cov_3d(sol, 8), candidates=len(plausible))
        T, sol = self.refine_3d(cands[0], use)
        return self.finish(T, use, self.cov_3d(sol, 8 * len(use)))

    # ------------------------------------------------------------ planar
    def rays_cam(self, use: List[Detection]) -> np.ndarray:
        px = np.vstack([d.corners for d in use])
        return np.hstack([px, np.ones((len(px), 1))]) @ self.Kinv.T

    def planes(self, use: List[Detection]):
        """Per corner: a point of its marker's plane, its normal, and two in-plane axes."""
        P, N, A, B = [], [], [], []
        for d in use:
            T = self.poses[d.id]
            for _ in range(4):
                P.append(T[:3, 3])
                A.append(T[:3, 0])
                B.append(T[:3, 1])
                N.append(T[:3, 2])
        return np.array(P), np.array(N), np.array(A), np.array(B)

    def planar_residuals(self, p, rays_c, R_tilt, planes, C, weight=True):
        """Per corner, where its ray meets its marker's plane minus the corner's map
        position, along the plane's two axes; in pixels (weight) or metres."""
        x, y, yaw = p
        R_mb = rot_z(yaw) @ R_tilt
        t_mb = np.array([x, y, self.s.base_height])
        R_mc = R_mb @ self.T_base_cam[:3, :3]
        o = R_mb @ self.T_base_cam[:3, 3] + t_mb
        d = rays_c @ R_mc.T
        P, N, A, B = planes
        den = np.sum(d * N, axis=1)
        den = np.where(np.abs(den) < 1e-9, 1e-9, den)
        s = np.sum((P - o) * N, axis=1) / den
        X = o + d * s[:, None]
        e = X - C
        r = np.stack([np.sum(e * A, axis=1), np.sum(e * B, axis=1)], axis=1)
        if weight:
            # metres on the plane per pixel at that range, foreshortening included
            mpp = np.abs(s) * np.linalg.norm(d, axis=1) / self.K[0, 0]
            cosang = np.abs(den) / np.linalg.norm(d, axis=1)
            r = r * (cosang / np.maximum(mpp, 1e-9))[:, None]
        bad = s <= 0
        if np.any(bad):
            r[bad] += 1e3
        return r.ravel(), s

    def planar_start(self, use, rays_c, R_tilt, planes, C) -> Optional[np.ndarray]:
        """(x, y, yaw) to start from: the source implementation's method, each corner's
        ray cut with the horizontal plane at the corner's own height and a 2D rigid
        fit; for markers that are not near horizontal, the 3d solution."""
        _, N, _, _ = planes
        if np.all(np.abs(N[:, 2]) > 0.7):
            from marker_survey.core.marker_pose import fit_pose_2d
            R_lc = R_tilt @ self.T_base_cam[:3, :3]          # level frame: yaw 0 at the base
            o = R_tilt @ self.T_base_cam[:3, 3] + [0, 0, self.s.base_height]
            d = rays_c @ R_lc.T
            ok = np.abs(d[:, 2]) > 1e-6
            s = (C[:, 2] - o[2]) / np.where(ok, d[:, 2], 1.0)
            if np.all(ok) and np.all(s > 0):
                pts = o + d * s[:, None]
                x, y, yaw, _ = fit_pose_2d(pts, C)
                return np.array([x, y, yaw])
        fix = self.solve_3d(use)
        if fix is None:
            return None
        return np.array([fix.x, fix.y, fix.yaw])

    def solve_planar(self, use: List[Detection], roll=0.0, pitch=0.0) -> Optional[Fix]:
        from scipy.optimize import least_squares
        rays_c = self.rays_cam(use)
        R_tilt = rot_y(pitch) @ rot_x(roll)
        planes = self.planes(use)
        C = np.vstack([self.corners[d.id] for d in use])
        p0 = self.planar_start(use, rays_c, R_tilt, planes, C)
        if p0 is None:
            return None
        sol = least_squares(lambda p: self.planar_residuals(p, rays_c, R_tilt, planes, C)[0], p0,
                            method='trf', loss='soft_l1', f_scale=self.s.robust_px,
                            x_scale=np.array([0.1, 0.1, 0.01]), max_nfev=50)
        p = sol.x
        p[2] = wrap(p[2])
        r_m, s = self.planar_residuals(p, rays_c, R_tilt, planes, C, weight=False)
        if np.any(s <= 0):
            return None
        T = planar_T(p[0], p[1], p[2], roll, pitch, self.s.base_height)
        # covariance of x, y, yaw from the pixel-weighted fit
        J, r = sol.jac, sol.fun
        sd = max(self.s.pixel_sd, float(np.sqrt(np.mean(r ** 2))))
        try:
            C3 = sd ** 2 * np.linalg.inv(J.T @ J)
        except np.linalg.LinAlgError:
            C3 = np.eye(3) * 1e2
        rng = float(np.mean(np.abs(s) * np.linalg.norm(rays_c @ (R_tilt @ self.T_base_cam[:3, :3]).T,
                                                       axis=1)))
        tilt = (rng * math.radians(self.s.tilt_sd_deg)) ** 2
        cov = np.zeros((6, 6))
        idx = [0, 1, 5]
        for a in range(3):
            for b in range(3):
                cov[idx[a], idx[b]] = C3[a, b]
        cov[0, 0] += tilt
        cov[1, 1] += tilt
        for i in (2, 3, 4):
            cov[i, i] = UNMEASURED_VARIANCE
        res_m = {}
        rr = np.linalg.norm(r_m.reshape(-1, 2), axis=1)
        for k, d in enumerate(use):
            res_m[d.id] = rr[4 * k:4 * k + 4]
        return self.finish(T, use, self.floor(cov, planar=True), residuals_m=res_m)


def rms(v) -> float:
    v = np.asarray(v, float).ravel()
    return float(np.sqrt(np.mean(v ** 2))) if len(v) else 0.0


def pose_distance(Ta, Tb, metres_per_rad: float = 1.0) -> float:
    """A distance between poses: translation [m] plus rotation [rad] times a scale."""
    return float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3])
                 + metres_per_rad * rotation_angle(Ta[:3, :3], Tb[:3, :3]))
