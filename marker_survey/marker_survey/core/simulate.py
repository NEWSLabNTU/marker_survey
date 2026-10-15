"""A synthetic survey: a robot driving a route among markers. No ROS.

For tests and for predicting a survey's accuracy before going on site. The
robot follows a route; every pose is a frame with

- the true base pose in the site's map frame;
- an odometry pose: the true relative motion with noise integrated along
  the drive, in an odometry frame that starts at the robot's first pose;
- a localizer's map pose: the true pose with noise, independent per frame
  (a LiDAR localizer's jitter) or, with map_corr_m, wandering slowly along
  the drive (a localizer's error that holds over metres, as a real one's
  does).

The corner-level simulation projects the marker corners with Gaussian pixel
noise (no image); render.py draws the images instead.
"""

from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from marker_survey.core.detection import Detection, marker_object_points
from marker_survey.core.se3 import inv_T, make_T, rot_z, rotvec_to_matrix
from marker_survey.core.survey import Survey, SurveySettings


@dataclass
class Camera:
    K: np.ndarray
    width: int
    height: int
    T_base_cam: np.ndarray          # base -> camera optical frame


def pinhole_K(width: int, height: int, hfov_deg: float) -> np.ndarray:
    f = width / 2 / math.tan(math.radians(hfov_deg / 2))
    return np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1.0]])


# optical axes (x right, y down, z forward) in the base frame
R_BASE_FORWARD = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)   # looking ahead
R_BASE_UPWARD = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], float)     # looking up, image top ahead


def forward_camera(width=1920, height=1200, hfov_deg=102.0, x=0.20, z=0.25) -> Camera:
    """A wide forward camera, 0.20 m ahead of the base and 0.25 m up."""
    return Camera(pinhole_K(width, height, hfov_deg), width, height,
                  make_T(R_BASE_FORWARD, [x, 0.0, z]))


def upward_camera(width=640, height=480, hfov_deg=70.0, x=-0.12, z=0.30) -> Camera:
    """A webcam looking straight up, 0.30 m above the floor."""
    return Camera(pinhole_K(width, height, hfov_deg), width, height,
                  make_T(R_BASE_UPWARD, [x, 0.0, z]))


def drive(path_xy: Sequence[Tuple[float, float]], step=0.05, sweeps: Dict[int, float] = None):
    """Base poses along a polyline, every `step` m; at the listed waypoint indices the
    robot swings its heading by +-amp rad (turned on the spot)."""
    poses = []
    sweeps = sweeps or {}
    for i in range(len(path_xy) - 1):
        (x0, y0), (x1, y1) = path_xy[i], path_xy[i + 1]
        yaw = math.atan2(y1 - y0, x1 - x0)
        n = max(1, int(round(math.hypot(x1 - x0, y1 - y0) / step)))
        for k in range(n):
            f = k / n
            poses.append((x0 + f * (x1 - x0), y0 + f * (y1 - y0), yaw))
        if i + 1 in sweeps:
            amp = sweeps[i + 1]
            for a in (list(np.linspace(0, amp, 15)) + list(np.linspace(amp, -amp, 30))
                      + list(np.linspace(-amp, 0, 15))):
                poses.append((x1, y1, yaw + a))
    return [make_T(rot_z(yaw), [x, y, 0.0]) for x, y, yaw in poses]


def route(path_xy: Sequence[Tuple[float, float]], laps: int = 1, closed: bool = True,
          step: float = 0.05, sweeps: int = 0, sweep_deg: float = 70.0):
    """Base poses for `laps` laps of a route; `sweeps` evenly spaced turns on the spot
    per lap."""
    pts = list(path_xy)
    if closed and pts[0] != pts[-1]:
        pts.append(pts[0])
    loop = []
    for lap in range(laps):
        loop += pts if lap == 0 else pts[1:]
    n = len(pts) - 1
    sw = {}
    if sweeps:
        for lap in range(laps):
            for i in np.linspace(0, n, sweeps + 1)[:-1]:
                sw[int(i) + lap * n] = math.radians(sweep_deg)
    return drive(loop, step=step, sweeps=sw)


@dataclass
class Frame:
    stamp: float
    T_map_base: np.ndarray           # truth
    T_odom_base: np.ndarray          # odometry, drifting; odom = the first pose
    T_mapmeas_base: np.ndarray       # a localizer's map pose, noisy


@dataclass
class Noise:
    drift: float = 0.01              # odometry translation sd per metre
    yaw_drift_deg_per_m: float = 0.3
    map_sd: float = 0.0              # [m] localizer noise per horizontal axis
    map_sd_yaw_deg: float = 0.0
    map_corr_m: float = 0.0          # [m] correlation length of that noise along the drive; 0: white
    pixel_sd: float = 0.5
    seed: int = 1


def make_frames(base_poses: List[np.ndarray], noise: Noise, rate: float = 10.0) -> List[Frame]:
    rng_o = np.random.default_rng(noise.seed)
    rng_m = np.random.default_rng(noise.seed + 1000)
    T_odo = np.eye(4)
    out = []
    z = rng_m.normal(0, 1, 3) if noise.map_corr_m > 0 else None   # correlated map noise, in sds
    for k, Tb in enumerate(base_poses):
        if k > 0:
            rel = inv_T(base_poses[k - 1]) @ Tb
            d = np.linalg.norm(rel[:3, 3])
            nt = rng_o.normal(0, noise.drift * d, 3)
            nr = rng_o.normal(0, math.radians(noise.yaw_drift_deg_per_m) * d + 1e-5, 3) \
                * np.array([0.2, 0.2, 1.0])
            T_odo = T_odo @ make_T(rel[:3, :3] @ rotvec_to_matrix(nr), rel[:3, 3] + nt)
        if noise.map_corr_m > 0:            # Ornstein-Uhlenbeck along the distance driven
            if k > 0:
                a = math.exp(-d / noise.map_corr_m)
                z = a * z + math.sqrt(1.0 - a * a) * rng_m.normal(0, 1, 3)
            e = make_T(rot_z(math.radians(z[2] * noise.map_sd_yaw_deg)),
                       [z[0] * noise.map_sd, z[1] * noise.map_sd, 0.0])
        else:
            e = make_T(rot_z(math.radians(rng_m.normal(0, noise.map_sd_yaw_deg)))
                       if noise.map_sd_yaw_deg else np.eye(3),
                       [rng_m.normal(0, noise.map_sd), rng_m.normal(0, noise.map_sd), 0.0]
                       if noise.map_sd else [0.0, 0.0, 0.0])
        out.append(Frame(k / rate, Tb, T_odo.copy(), Tb @ e))
    return out


def visible(T_world_cam, Tm, size, camera: Camera, max_angle_deg=65.0, min_depth=0.3,
            margin=2.0) -> Optional[np.ndarray]:
    """Pixel corners of a marker if all of it is in view and it faces the camera."""
    obj = marker_object_points(size)
    Xw = obj @ Tm[:3, :3].T + Tm[:3, 3]
    T_cw = inv_T(T_world_cam)
    Xc = Xw @ T_cw[:3, :3].T + T_cw[:3, 3]
    if np.any(Xc[:, 2] < min_depth):
        return None
    v = T_world_cam[:3, 3] - Tm[:3, 3]
    if (v / np.linalg.norm(v)) @ Tm[:3, 2] < math.cos(math.radians(max_angle_deg)):
        return None
    uv = Xc @ camera.K.T
    uv = uv[:, :2] / uv[:, 2:]
    if uv.min() < margin or uv[:, 0].max() > camera.width - margin or \
            uv[:, 1].max() > camera.height - margin:
        return None
    return uv


def project(markers: Dict[int, np.ndarray], sizes: Dict[int, float], camera: Camera,
            T_world_cam: np.ndarray, rng, pixel_sd: float) -> List[Detection]:
    dets = []
    for mid, Tm in markers.items():
        uv = visible(T_world_cam, Tm, sizes[mid], camera)
        if uv is not None:
            dets.append(Detection(mid, uv + rng.normal(0, pixel_sd, uv.shape)))
    return dets


def survey_pose(f: Frame, mode: str) -> np.ndarray:
    """The pose source's base pose a survey in `mode` sees."""
    return f.T_mapmeas_base if mode == 'pose_frame' else f.T_odom_base


def simulate(markers: Dict[int, np.ndarray], sizes: Dict[int, float], frames: List[Frame],
             camera: Camera, settings: SurveySettings, mode: str = 'origin_marker',
             noise: Noise = None) -> Survey:
    """A survey fed with corner-level detections of every frame."""
    noise = noise or Noise()
    rng = np.random.default_rng(noise.seed + 2000)
    sv = Survey(settings, camera.K)
    sv.T_cam_base = inv_T(camera.T_base_cam)
    for f in frames:
        dets = project(markers, sizes, camera, f.T_map_base @ camera.T_base_cam, rng, noise.pixel_sd)
        T_wc = survey_pose(f, mode) @ camera.T_base_cam
        sv.add(f.stamp, T_wc, dets, f.T_mapmeas_base if mode == 'align_to_track' else None)
    return sv


def settings_for(mode: str, marker_size: float, sizes: Dict[int, float] = None,
                 pose_sd: float = 0.05, **kw) -> SurveySettings:
    """Survey settings for a frame mode: pose priors in pose_frame, motion otherwise."""
    s = SurveySettings(marker_size=marker_size, marker_sizes=dict(sizes or {}), **kw)
    if mode == 'pose_frame':
        s.motion_factors, s.pose_priors, s.pose_sd = False, True, pose_sd
    return s
