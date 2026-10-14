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
