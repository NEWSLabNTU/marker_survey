"""Rigid transforms as 4x4 matrices. No ROS.

T_a_b maps points of frame b into frame a. Rotations are 3x3 matrices;
rotation vectors (axis times angle, radians) parameterise them in the solver.
"""

import math

import numpy as np


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rotvec_to_matrix(v) -> np.ndarray:
    v = np.asarray(v, float)
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * Kx + (1 - math.cos(th)) * (Kx @ Kx)


def matrix_to_rotvec(R) -> np.ndarray:
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_rotvec()


def make_T(R, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def inv_T(T) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def T_from_params(p) -> np.ndarray:
    return make_T(rotvec_to_matrix(p[:3]), p[3:6])


def params_from_T(T) -> np.ndarray:
    return np.concatenate([matrix_to_rotvec(T[:3, :3]), T[:3, 3]])


def quat_to_matrix(x, y, z, w) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(R) -> np.ndarray:
    """(x, y, z, w)."""
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()


def interpolate_T(Ta, Tb, f: float) -> np.ndarray:
    """Pose a fraction f of the way from Ta to Tb (slerp + lerp)."""
    from scipy.spatial.transform import Rotation, Slerp
    rots = Rotation.from_matrix(np.stack([Ta[:3, :3], Tb[:3, :3]]))
    R = Slerp([0.0, 1.0], rots)([f]).as_matrix()[0]
    return make_T(R, (1 - f) * Ta[:3, 3] + f * Tb[:3, 3])


def pose_at_stamp(stamps, poses, t: float, tolerance: float, max_gap: float):
    """The pose at stamp t from a track of (sorted) stamps and poses: interpolated
    between its neighbours when they are at most max_gap apart (a lost message
    leaves a gap of two intervals), else the nearest one within tolerance; None
    when there is none (the track has not reached t yet, or a long gap)."""
    ts = np.asarray(stamps, float)
    if len(ts) == 0:
        return None
    i = int(np.searchsorted(ts, t))
    if 0 < i < len(ts):
        t0, t1 = ts[i - 1], ts[i]
        if t1 - t0 <= max(max_gap, 2 * tolerance):
            return interpolate_T(poses[i - 1], poses[i], (t - t0) / max(t1 - t0, 1e-9))
    near = [j for j in (i - 1, i) if 0 <= j < len(ts)]
    j = min(near, key=lambda j: abs(ts[j] - t))
    return poses[j] if abs(ts[j] - t) <= tolerance else None


def rotation_angle(Ra, Rb) -> float:
    """Angle [rad] of the rotation between two orientations."""
    c = (np.trace(np.asarray(Ra).T @ np.asarray(Rb)) - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, c)))


def umeyama(src, dst):
    """Rigid transform (R, t) minimising sum |R src_i + t - dst_i|^2, no scale.

    src, dst: N x 3. Returns (T_dst_src, rms residual)."""
    a = np.asarray(src, float)
    b = np.asarray(dst, float)
    if len(a) < 3:
        raise ValueError(f'a rigid fit needs at least 3 points, got {len(a)}')
    ma, mb = a.mean(axis=0), b.mean(axis=0)
    H = (a - ma).T @ (b - mb)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d if d != 0 else 1.0])
    R = Vt.T @ D @ U.T
    t = mb - R @ ma
    res = a @ R.T + t - b
    return make_T(R, t), float(np.sqrt(np.mean(np.sum(res ** 2, axis=1))))
