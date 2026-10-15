"""The frame a survey's result is expressed in. No ROS.

| mode            | result frame                                                         |
|-----------------|----------------------------------------------------------------------|
| origin_marker   | origin on the floor below a chosen marker, x the way it faces, z up  |
|                 | (the floor normal from the base frame's poses)                       |
| pose_frame      | the pose source's own frame (for example `map` while a localizer     |
|                 | runs); nothing is moved                                               |
| align_to_track  | the survey runs in the odometry frame; the solved base trajectory is |
|                 | then fitted rigidly (Umeyama, no scale) to a reference trajectory    |
|                 | recorded at the same stamps (a localizer's poses)                    |

Each function returns T_target_world: survey-frame points into the result
frame.
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

from marker_survey.core.se3 import make_T, umeyama

FRAME_MODES = ('origin_marker', 'pose_frame', 'align_to_track')


@dataclass
class FrameSettings:
    mode: str = 'origin_marker'
    origin_marker: Optional[int] = None   # origin_marker mode
    frame_id: str = 'map'                 # the result frame's name, written to the map
    floor_offset: float = 0.0             # [m] the floor lies this far below the base frame

    def check(self):
        if self.mode not in FRAME_MODES:
            raise ValueError(f'frame mode {self.mode!r} is not one of {", ".join(FRAME_MODES)}')
        if self.mode == 'origin_marker' and (self.origin_marker is None or self.origin_marker < 0):
            raise ValueError('frame mode origin_marker needs frame.origin_marker: the marker '
                             'that defines the frame')


def floor_from_base_poses(T_world_base: Sequence[np.ndarray], floor_offset: float = 0.0):
    """The floor: mean base z axis as the normal, mean base origin on it, moved down by
    floor_offset (a vehicle on a stand has its wheels in the air)."""
    zs = np.array([T[:3, 2] for T in T_world_base])
    n = zs.mean(axis=0)
    n /= np.linalg.norm(n)
    p = np.mean([T[:3, 3] for T in T_world_base], axis=0) - float(floor_offset) * n
    return p, n


def origin_marker_frame(T_world_origin: np.ndarray, floor_point, floor_normal) -> np.ndarray:
    """T_world_target: z = floor normal; origin = the marker's centre dropped onto the
    floor; x = the horizontal part of the direction the marker faces (for a marker lying
    flat, of its image x axis)."""
    u = np.asarray(floor_normal, float)
    c = T_world_origin[:3, 3]
    origin = c - ((c - floor_point) @ u) * u
    facing = T_world_origin[:3, 2]
    xh = facing - (facing @ u) * u
    if np.linalg.norm(xh) < 0.2:
        ax = T_world_origin[:3, 0]
        xh = ax - (ax @ u) * u
    xh /= np.linalg.norm(xh)
    y = np.cross(u, xh)
    return make_T(np.column_stack([xh, y, u]), origin)


def align_to_track(T_world_base: List[np.ndarray], T_ref_base: List[Optional[np.ndarray]]):
    """T_target_world (the reference frame from the survey frame) fitting the solved base
    positions onto the reference positions at the same stamps; and the fit's per-point
    residuals [m]."""
    pairs = [(a[:3, 3], b[:3, 3]) for a, b in zip(T_world_base, T_ref_base) if b is not None]
    if len(pairs) < 3:
        raise ValueError(f'align_to_track needs a reference pose for at least 3 keyframes, '
                         f'got {len(pairs)}: is the reference frame in TF?')
    src = np.array([p[0] for p in pairs])
    dst = np.array([p[1] for p in pairs])
    span = np.ptp(src, axis=0)
    if np.sort(span)[-2] < 0.5:
        raise ValueError('align_to_track: the trajectory is (nearly) a straight line, so its '
                         'rotation about that line is unknown; drive a loop or a turn')
    T, _ = umeyama(src, dst)
    res = np.linalg.norm(src @ T[:3, :3].T + T[:3, 3] - dst, axis=1)
    return T, res


def correlation_time(x: np.ndarray) -> float:
    """Integrated autocorrelation time of a series (1 for independent samples, about
    2 L / step for errors that hold over a length L), by Geyer's initial positive
    sequence: autocorrelations summed in pairs while the pairs stay positive."""
    x = np.asarray(x, float) - np.mean(x)
    n = len(x)
    v = float(x @ x) / n
    if n < 4 or v <= 0:
        return 1.0
    f = np.fft.rfft(x, 2 * n)
    acf = np.fft.irfft(f * np.conj(f))[:n] / (n * v)
    tau = -1.0
    for k in range(0, n - 1, 2):
        pair = acf[k] + acf[k + 1]
        if pair <= 0:
            break
        tau += 2.0 * pair
    return float(min(max(tau, 1.0), n))


def alignment_covariance(T_world_base: List[np.ndarray], T_ref_base: List[Optional[np.ndarray]],
                         T_target_world: np.ndarray, n_eff: Optional[float] = None):
    """How well align_to_track placed the survey in the reference frame: the 6 x 6
    covariance (rotation vector, translation; target frame) of the fitted transform,
    about the reference positions' centroid; the centroid; and the number of
    independent samples the residuals held.

    The fit's residuals (the reference track's error, and what is left of the
    odometry's) are not independent from keyframe to keyframe: a localizer's error
    holds over metres. Their correlation along the track, measured from the residuals
    themselves (correlation_time), divides the number of keyframes into a number of
    independent samples (n_eff, at least 1, unless given), and the covariance is the
    least-squares one for that many samples of the residuals' spread. Errors that
    hold over the whole drive count once: they do not shrink with more keyframes."""
    pairs = [(a[:3, 3], b[:3, 3]) for a, b in zip(T_world_base, T_ref_base) if b is not None]
    src = np.array([p[0] for p in pairs])
    dst = np.array([p[1] for p in pairs])
    R, t = T_target_world[:3, :3], T_target_world[:3, 3]
    q = src @ R.T + t
    e = dst - q
    n = len(e)
    c = dst.mean(axis=0)
    d = q - c
    if n_eff is None:
        taus = [correlation_time(e[:, j]) for j in range(3) if np.std(e[:, j]) > 1e-6]
        n_eff = n / max(taus) if taus else float(n)
    n_eff = float(min(max(n_eff, 1.0), n))
    var = np.maximum(np.mean(e ** 2, axis=0) * n / max(n - 6, 1), 1e-12)   # per axis
    A = np.zeros((3 * n, 6))
    W = np.tile(1.0 / var, n)
    for k in range(n):
        A[3 * k:3 * k + 3, :3] = -np.array([[0, -d[k, 2], d[k, 1]],
                                            [d[k, 2], 0, -d[k, 0]],
                                            [-d[k, 1], d[k, 0], 0]])
        A[3 * k:3 * k + 3, 3:] = np.eye(3)
    N = (A.T * W) @ A
    try:
        C = np.linalg.inv(N) * (n / n_eff)
    except np.linalg.LinAlgError:
        C = np.linalg.pinv(N) * (n / n_eff)
    return 0.5 * (C + C.T), c, n_eff
