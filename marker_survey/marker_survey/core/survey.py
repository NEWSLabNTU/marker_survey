"""Marker survey from a moving camera: keyframes, live map, bundle adjustment. No ROS.

The camera moves among markers. Every frame comes with a camera pose from a
pose source (visual-inertial or wheel odometry, or any TF frame), which links
frames in which the markers are seen apart. The survey:

  1. keeps a frame as a keyframe after 15 cm or 5 degrees of motion, or when a
     new marker appears;
  2. keeps a live estimate of every marker as soon as it is first seen (the
     pose source's camera pose times the marker's single-frame pose), for
     feedback while driving;
  3. at the end solves a bundle adjustment over every keyframe and marker:
     marker corner reprojection (robust) plus, by the pose source's kind,
     - motion factors: its relative motion between consecutive keyframes,
       weighted for its drift (odometry; keyframe 0 fixes the frame), or
     - pose priors: its absolute pose of every keyframe, weighted by its
       noise (a localizer's map pose; the result stays in its frame).
     Markers seen together correct the drift; markers seen alone still land
     in the frame;
  4. gives the joint covariance of the markers and of the floor (the mean of
     the base poses, which the result frame stands on), with the corner noise
     measured from the solve's own reprojection residuals rather than assumed.

Choosing the frame the result is expressed in is frames.py's job; writing
it, solve.py's.

Conventions: detection.py for the marker frame and corner order. Poses are
4x4 matrices; T_a_b maps b-frame points into frame a.
"""

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional

import numpy as np

from marker_survey.core.detection import Detection, marker_object_points, marker_pose_in_camera
from marker_survey.core.se3 import inv_T, make_T, matrix_to_rotvec, params_from_T


@dataclass
class Keyframe:
    stamp: float
    T_world_cam: np.ndarray            # from the pose source
    detections: List[Detection]
    T_ref_base: Optional[np.ndarray] = None   # reference track at this stamp (align_to_track)


@dataclass
class MarkerEstimate:
    id: int
    T_world_marker: np.ndarray
    views: int = 0
    best_area: float = 0.0


@dataclass
class SurveySettings:
    marker_size: float
    marker_sizes: Dict[int, float] = field(default_factory=dict)   # per-marker exceptions
    keyframe_distance: float = 0.15    # [m] new keyframe after this much motion...
    keyframe_angle: float = math.radians(5.0)   # ...or this much rotation
    min_marker_px: float = 20.0        # ignore detections smaller than this (edge length)
    max_reproj_px: float = 3.0         # ...or with a worse single-frame fit
    motion_factors: bool = True        # relative motion of the pose source (odometry)
    odom_drift: float = 0.01           # translation sd per metre travelled
    odom_floor_t: float = 0.003        # [m] and its floor
    odom_rot_per_rad: float = 0.01     # rotation sd per radian turned
    odom_floor_r: float = math.radians(0.2)
    pose_priors: bool = False          # absolute poses of the pose source (a map pose)
    pose_sd: float = 0.05              # [m] their noise, per axis
    pose_sd_rot: float = math.radians(1.0)   # [rad]
    # [m] reference track noise, for the uncertainty of an align_to_track result
    track_sd: float = 0.05
    pixel_sd: float = 0.5              # [px] corner noise the solve weights the corners by
    # [px] the covariance uses the corner noise measured in the solve, at least this
    min_pixel_sd: float = 0.05
    huber_px: float = 1.5              # robust loss scale, in pixel sds
    # scipy least_squares loss. soft_l1 is Huber-like but keeps far-off residuals in the
    # Jacobian; with 'huber' they drop out of it, the start keyframe's constraint can
    # vanish and the whole map drift away from it.
    loss: str = 'soft_l1'

    def size_of(self, mid: int) -> float:
        return float(self.marker_sizes.get(mid, self.marker_size))


class Survey:
    """Accumulates keyframes and keeps a live marker map."""

    def __init__(self, settings: SurveySettings, K: np.ndarray):
        self.s = settings
        self.K = np.asarray(K, float)
        self.keyframes: List[Keyframe] = []
        self.markers: Dict[int, MarkerEstimate] = {}
        self.T_cam_base: Optional[np.ndarray] = None    # camera optical frame -> base frame
        # (marker ids, joint covariance) of the last solve: 6 per marker (rotation vector,
        # translation), then 6 for the floor if T_cam_base is known (FLOOR_PARAMS)
        self.joint_cov = None

    def wants_keyframe(self, T_world_cam: np.ndarray, detections: List[Detection]) -> bool:
        if not detections:
            return False
        if not self.keyframes:
            return True
        last = self.keyframes[-1].T_world_cam
        d = np.linalg.norm(T_world_cam[:3, 3] - last[:3, 3])
        a = np.linalg.norm(matrix_to_rotvec(last[:3, :3].T @ T_world_cam[:3, :3]))
        new_marker = any(det.id not in self.markers for det in detections)
        return new_marker or d >= self.s.keyframe_distance or a >= self.s.keyframe_angle

    def usable(self, det: Detection) -> bool:
        return det.rejected is None and det.min_edge() >= self.s.min_marker_px

    def add(self, stamp: float, T_world_cam: np.ndarray, detections: List[Detection],
            T_ref_base: Optional[np.ndarray] = None) -> List[int]:
        """Add a frame if it is a keyframe. Returns the ids seen for the first time."""
        dets = [d for d in detections if self.usable(d)]
        if not self.wants_keyframe(T_world_cam, dets):
            return []
        kept, new = [], []
        for d in dets:
            T_cm, err = marker_pose_in_camera(d, self.K, self.s.size_of(d.id))
            if not (err <= self.s.max_reproj_px and np.all(np.isfinite(T_cm))):
                continue
            kept.append(d)
            self._update_live(d, T_world_cam @ T_cm, new)
        if kept:
            self.keyframes.append(Keyframe(stamp, T_world_cam.copy(), kept,
                                           None if T_ref_base is None else T_ref_base.copy()))
        return new

    def _update_live(self, d: Detection, T_wm: np.ndarray, new: List[int]):
        area = d.area()
        m = self.markers.get(d.id)
        if m is None:
            self.markers[d.id] = MarkerEstimate(d.id, T_wm, 1, area)
            new.append(d.id)
        else:
            m.views += 1
            if area > m.best_area:       # the live estimate follows the best view
                m.T_world_marker = T_wm
                m.best_area = area

    def rebuild_live(self):
        """Live estimates from the keyframes (after loading a recording)."""
        self.markers = {}
        for kf in self.keyframes:
            for d in kf.detections:
                T_cm, err = marker_pose_in_camera(d, self.K, self.s.size_of(d.id))
                if np.isfinite(err) and np.all(np.isfinite(T_cm)):
                    self._update_live(d, kf.T_world_cam @ T_cm, [])

    # -------------------------------------------------------------- the solve
    def solve(self, max_nfev: int = 300):
        """Bundle adjustment. Returns (markers {id: T_world_marker}, keyframe poses
        T_world_cam, per-marker stats {id: dict}, info) in the pose source's frame."""
        from scipy.optimize import least_squares
        from scipy.sparse import lil_matrix
        from scipy.spatial.transform import Rotation

        kfs = self.keyframes
        ids = sorted(self.markers)
        if len(kfs) < 2 or not ids:
            raise ValueError('not enough data: drive past the markers first')
        s = self.s
        mi = {mid: n for n, mid in enumerate(ids)}
        nk, nm = len(kfs), len(ids)
        free0 = s.pose_priors            # with priors nothing is fixed; else keyframe 0 is
        nkp = nk if free0 else nk - 1    # keyframe parameter blocks
        k_off = 0 if free0 else 1
        unit = marker_object_points(1.0)
        obs = [(k, mi[d.id], d.corners) for k, kf in enumerate(kfs) for d in kf.detections]
        o_k = np.array([o[0] for o in obs])
        o_m = np.array([o[1] for o in obs])
        o_uv = np.stack([o[2] for o in obs])            # n x 4 x 2
        sizes = np.array([s.size_of(mid) for mid in ids])
        obj_m = unit[None, :, :] * sizes[:, None, None]   # nm x 4 x 3
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]

        odo = []
        if s.motion_factors:
            for a in range(nk - 1):
                T_ab = inv_T(kfs[a].T_world_cam) @ kfs[a + 1].T_world_cam
                dist = np.linalg.norm(T_ab[:3, 3])
                ang = np.linalg.norm(matrix_to_rotvec(T_ab[:3, :3]))
                st = s.odom_floor_t + s.odom_drift * dist
                sr = s.odom_floor_r + s.odom_rot_per_rad * ang
                odo.append((a, T_ab, st, sr))
        o_a = np.array([o[0] for o in odo], int)
        R_ab = np.stack([o[1][:3, :3] for o in odo]) if odo else np.zeros((0, 3, 3))
        t_ab = np.stack([o[1][:3, 3] for o in odo]) if odo else np.zeros((0, 3))
        w_t = np.array([1.0 / o[2] for o in odo])
        w_r = np.array([1.0 / o[3] for o in odo])
        R_pr = np.stack([kf.T_world_cam[:3, :3] for kf in kfs])
        t_pr = np.stack([kf.T_world_cam[:3, 3] for kf in kfs])

        x0 = np.concatenate([params_from_T(kf.T_world_cam) for kf in kfs[k_off:]]
                            + [params_from_T(self.markers[mid].T_world_marker) for mid in ids])
        T0 = kfs[0].T_world_cam
        p0 = params_from_T(T0)

        def unpack(x):
            kp = x[:6 * nkp].reshape(-1, 6)
            if not free0:
                kp = np.vstack([p0[None, :], kp])
            mp = x[6 * nkp:].reshape(nm, 6)
            Rk = Rotation.from_rotvec(kp[:, :3]).as_matrix()
            tk = kp[:, 3:].copy()
            if not free0:
                Rk[0] = T0[:3, :3]
                tk[0] = T0[:3, 3]
            Rm = Rotation.from_rotvec(mp[:, :3]).as_matrix()
            return Rk, tk, Rm, mp[:, 3:]

        def residuals(x):
            Rk, tk, Rm, tm = unpack(x)
            Xw = np.einsum('nij,ncj->nci', Rm[o_m], obj_m[o_m]) + tm[o_m][:, None, :]   # n x 4 x 3
            Xc = np.einsum('nji,ncj->nci', Rk[o_k], Xw - tk[o_k][:, None, :])        # R^T (X - t)
            z = np.maximum(Xc[..., 2], 1e-3)
            u = fx * Xc[..., 0] / z + cx
            v = fy * Xc[..., 1] / z + cy
            out = [(np.stack([u, v], -1) - o_uv).reshape(-1) / s.pixel_sd]
            if len(o_a):
                Ra, ta, Rb, tb = Rk[o_a], tk[o_a], Rk[o_a + 1], tk[o_a + 1]
                R_est = np.einsum('nji,njk->nik', Ra, Rb)                      # Ra^T Rb
                t_est = np.einsum('nji,nj->ni', Ra, tb - ta)                   # Ra^T (tb - ta)
                R_err = np.einsum('nji,njk->nik', R_ab, R_est)                 # R_ab^T R_est
                t_err = np.einsum('nji,nj->ni', R_ab, t_est - t_ab)
                r_rot = Rotation.from_matrix(R_err).as_rotvec() * w_r[:, None]
                out.append(np.hstack([r_rot, t_err * w_t[:, None]]).reshape(-1))
            if s.pose_priors:
                R_err = np.einsum('nji,njk->nik', R_pr, Rk)                    # R_pr^T Rk
                r_rot = Rotation.from_matrix(R_err).as_rotvec() / s.pose_sd_rot
                r_tr = (tk - t_pr) / s.pose_sd
                out.append(np.hstack([r_rot, r_tr]).reshape(-1))
            return np.concatenate(out)

        npar = x0.size
        nres = 8 * len(obs) + 6 * len(odo) + (6 * nk if s.pose_priors else 0)
        sp = lil_matrix((nres, npar), dtype=np.int8)

        def kcols(k):
            j = k - k_off
            return slice(6 * j, 6 * j + 6) if j >= 0 else None

        for n, (k, m, _) in enumerate(obs):
            rows = slice(8 * n, 8 * n + 8)
            if kcols(k) is not None:
                sp[rows, kcols(k)] = 1
            sp[rows, 6 * nkp + 6 * m:6 * nkp + 6 * m + 6] = 1
        base = 8 * len(obs)
        for n, (a, _, _, _) in enumerate(odo):
            rows = slice(base + 6 * n, base + 6 * n + 6)
            for k in (a, a + 1):
                if kcols(k) is not None:
                    sp[rows, kcols(k)] = 1
        base += 6 * len(odo)
        if s.pose_priors:
            for k in range(nk):
                sp[base + 6 * k:base + 6 * k + 6, kcols(k)] = 1
        sol = least_squares(residuals, x0, jac_sparsity=sp, loss=s.loss, f_scale=s.huber_px,
                            x_scale='jac', max_nfev=max_nfev, method='trf', ftol=1e-6, xtol=1e-6,
                            tr_solver='lsmr', tr_options={'atol': 1e-10, 'btol': 1e-10, 'maxiter': 3000})
        x = sol.x
        r = residuals(x)
        proj = r[:8 * len(obs)].reshape(len(obs), 8) * s.pixel_sd
        Rk, tk, Rm, tm = unpack(x)
        markers = {mid: make_T(Rm[mi[mid]], tm[mi[mid]]) for mid in ids}
        poses = [make_T(Rk[k], tk[k]) for k in range(nk)]
        extra = None
        if s.track_sd and any(kf.T_ref_base is not None for kf in kfs):
            # the reference track the result is aligned to anchors every keyframe
            extra = np.zeros(npar)
            for k, kf in enumerate(kfs):
                if kf.T_ref_base is not None and kcols(k) is not None:
                    c = kcols(k)
                    extra[c.start + 3:c.start + 6] = 1.0 / s.track_sd ** 2
        # the corners' noise as measured: a robust spread of the reprojection residuals,
        # corrected for the parameters fitted to them; the covariance uses it instead of
        # the assumed pixel_sd (which only weighs the corners against the other factors)
        npix = 8 * len(obs)
        dof = max(npix - npar, npix // 2)
        noise = float(1.4826 * np.median(np.abs(r[:npix])) * math.sqrt(npix / dof)) * s.pixel_sd
        cov_px = max(noise, s.min_pixel_sd)
        from scipy.sparse import csr_matrix, diags
        w = np.ones(nres)
        w[:npix] = s.pixel_sd / cov_px
        jac = diags(w) @ csr_matrix(sol.jac)
        Fk = None
        if self.T_cam_base is not None:
            Fk = floor_jacobian(x[:6 * nkp].reshape(-1, 6), self.T_cam_base, nk)
        joint = marker_covariances(jac, 6 * nkp, nm, extra, Fk)
        self.joint_cov = (ids, joint)
        stats = {}
        for mid in ids:
            sel = o_m == mi[mid]
            rms = float(np.sqrt(np.mean(proj[sel] ** 2))) if sel.any() else float('nan')
            stats[mid] = {'views': int(sel.sum()), 'reproj_rms_px': rms,
                          'cov_world': None if joint is None else
                          joint[6 * mi[mid]:6 * mi[mid] + 6, 6 * mi[mid]:6 * mi[mid] + 6]}
        info = {'keyframes': nk, 'observations': len(obs), 'cost': float(sol.cost),
                'reproj_rms_px': float(np.sqrt(np.mean(proj ** 2))), 'status': str(sol.message),
                'pixel_noise_px': noise,
                'factors': ('pose priors' if s.pose_priors else '')
                + (' + ' if s.pose_priors and odo else '') + ('motion' if odo else '')}
        return markers, poses, stats, info


FLOOR_PARAMS = 6      # the floor in the joint covariance: mean base z axis, mean base origin


def floor_vector(T_world_base) -> np.ndarray:
    """The floor's parameters (FLOOR_PARAMS): the mean z axis of the base poses (not
    normalised) and their mean origin; frames.floor_from_base_poses makes the floor."""
    return np.concatenate([np.mean([T[:3, 2] for T in T_world_base], axis=0),
                           np.mean([T[:3, 3] for T in T_world_base], axis=0)])


def floor_jacobian(kp: np.ndarray, T_cam_base: np.ndarray, nk: int, eps: float = 1e-6):
    """d floor_vector / d keyframe parameters (FLOOR_PARAMS x 6 len(kp)); `kp` are the
    free keyframes' camera poses (rotation vector, translation) out of `nk`."""
    from scipy.spatial.transform import Rotation
    zc, tc = T_cam_base[:3, 2], T_cam_base[:3, 3]
    n = len(kp)
    F = np.zeros((FLOOR_PARAMS, 6 * n))
    R0 = Rotation.from_rotvec(kp[:, :3]).as_matrix()
    for j in range(3):
        d = kp[:, :3].copy()
        d[:, j] += eps
        dR = (Rotation.from_rotvec(d).as_matrix() - R0) / eps          # n x 3 x 3
        F[:3, j::6] = (dR @ zc).T / nk
        F[3:, j::6] = (dR @ tc).T / nk
    for j in range(3):
        F[3 + j, 3 + j::6] = 1.0 / nk
    return F


def marker_covariances(jac, off: int, nm: int, extra_diag=None, Fk=None):
    """Joint covariance (6 nm x 6 nm; per marker rotvec then translation) of all markers,
    from the Gauss-Newton normal matrix J^T J (plus `extra_diag`, information added
    on the diagonal); None if it is singular. The markers' parameters come last
    (from `off`); the keyframes are eliminated (Schur complement), which keeps the
    sparse factorisation small: the keyframe block is banded. With `Fk` (q x off),
    linear functions of the keyframe parameters (the floor), their q rows and columns
    follow the markers'."""
    from scipy.sparse import csc_matrix, diags
    from scipy.sparse.linalg import splu
    J = csc_matrix(jac)
    A = (J.T @ J).tocsc()
    if extra_diag is not None:
        A = (A + diags(np.asarray(extra_diag, float))).tocsc()
    Amm = A[off:, off:].toarray()
    try:
        if off > 0:
            Akk = A[:off, :off].tocsc()
            Akm = A[:off, off:].toarray()
            lu = splu(Akk)
            X = lu.solve(Akm)
            S = Amm - Akm.T @ X
        else:
            S = Amm
        C = np.linalg.inv(S)
        if Fk is not None and off > 0:
            FX = Fk @ X
            Cfm = -FX @ C
            Cff = Fk @ lu.solve(np.ascontiguousarray(Fk.T)) + FX @ C @ FX.T
            C = np.block([[C, Cfm.T], [Cfm, Cff]])
    except (RuntimeError, np.linalg.LinAlgError):          # singular
        return None
    if not np.all(np.isfinite(C)):
        return None
    return 0.5 * (C + C.T)
