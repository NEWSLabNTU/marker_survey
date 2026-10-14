"""Marker detection for ArUco and AprilTag families, OpenCV 4.5 to 4.10. No ROS.

Settings per family, each one measured:

- Every family: the corners are refined with cornerSubPix and a half-window
  of half a cell, 2 or 3 pixels. OpenCV's own CORNER_REFINE_SUBPIX uses a
  fixed 5 pixel window, which on small markers reaches the bits inside and
  pulls the corner onto them (up to 3 px off on 25 px markers in rendered
  images; 0.2 px median with the small window).
- AprilTag families (tag36h11, ...): OpenCV 4.6 decodes them reliably only
  with CORNER_REFINE_APRILTAG. That refinement leaves the corners about half
  a pixel inside the black square, and cornerSubPix moves them onto its edge
  (in simulation, 8 cm pose error before, 1 cm after).
- ArUco dictionaries (DICT_5X5_1000, ...): CORNER_REFINE_NONE, then
  cornerSubPix as above.
- Last, every family: refine_edges() fits a line to each edge of the black
  square and intersects them. cornerSubPix leaves the corners 0.15 to 0.2 px
  inside the square, which scales distances by about 1 % on 30 px markers (a
  4 cm error at 5 m in rendered surveys); the edge fit is unbiased (median
  error 0.02 px on 70 px markers, 0.1 px on 40 px markers).
- A marker cut by the image border still decodes, but its hidden corner is
  guessed: markers with a corner within `border_margin` pixels of the border
  are dropped.
- Markers whose shortest edge is under `min_marker_px` are dropped: their
  corners are too coarse to place them.

API differences: OpenCV 4.7 added the ArucoDetector class and dropped
DetectorParameters_create(); 4.5 and 4.6 have only the function API, and on
4.6 calling DetectorParameters() crashes, so DetectorParameters_create() is
used wherever it exists.

Conventions. A marker's frame is OpenCV's: x to the viewer's right, y up
(towards the top of the printed image), z out of the marker towards the
viewer. Corners come top-left, top-right, bottom-right, bottom-left of the
printed image.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from marker_survey.core.se3 import make_T

# Short names, as a marker map's `dictionary` may give them, to OpenCV names.
DICTIONARY_ALIASES = {
    'tag16h5': 'DICT_APRILTAG_16h5',
    'tag25h9': 'DICT_APRILTAG_25h9',
    'tag36h10': 'DICT_APRILTAG_36h10',
    'tag36h11': 'DICT_APRILTAG_36h11',
    'aruco_mip_36h12': 'DICT_ARUCO_MIP_36h12',
    'aruco_original': 'DICT_ARUCO_ORIGINAL',
    'aruco': 'DICT_ARUCO_ORIGINAL',
}


def opencv_dictionary_name(name: str) -> str:
    """The OpenCV DICT_* name for a dictionary name (DICT_* or a short alias)."""
    if name.startswith('DICT_'):
        return name
    cv_name = DICTIONARY_ALIASES.get(name.lower())
    if cv_name is None:
        raise ValueError(f'unknown dictionary {name!r}: use an OpenCV name such as '
                         f'DICT_5X5_1000 or one of {", ".join(sorted(DICTIONARY_ALIASES))}')
    return cv_name


def is_apriltag(name: str) -> bool:
    return opencv_dictionary_name(name).startswith('DICT_APRILTAG')


def opencv_dictionary(name: str):
    """cv2.aruco dictionary for a dictionary name."""
    import cv2
    cv_name = opencv_dictionary_name(name)
    if not hasattr(cv2.aruco, cv_name):
        raise ValueError(f'dictionary {name!r} ({cv_name}) is not available in this OpenCV '
                         f'({cv2.__version__}); DICT_ARUCO_MIP_36h12 needs OpenCV >= 4.7')
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cv_name))


def marker_object_points(size: float) -> np.ndarray:
    """The four corners in the marker's frame, in OpenCV's order."""
    h = size / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


@dataclass
class Detection:
    id: int
    corners: np.ndarray              # 4 x 2 px, TL TR BR BL
    rejected: Optional[str] = None   # None if usable, else why not ('small', 'border')

    def min_edge(self) -> float:
        c = self.corners
        return float(min(np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)))

    def area(self) -> float:
        c = self.corners
        return float(abs(np.cross(c[2] - c[0], c[3] - c[1])) / 2)


class Detector:
    """Detects one dictionary with the settings that work for its family."""

    def __init__(self, dictionary: str, min_marker_px: float = 20.0, border_margin: float = 5.0,
                 edges: bool = True):
        import cv2
        self.cv2 = cv2
        self.name = dictionary
        self.dictionary = opencv_dictionary(dictionary)
        self.apriltag = is_apriltag(dictionary)
        self.min_marker_px = float(min_marker_px)
        self.border_margin = float(border_margin)
        self.edges = bool(edges)
        if hasattr(cv2.aruco, 'DetectorParameters_create'):        # OpenCV 4.5, 4.6
            self.params = cv2.aruco.DetectorParameters_create()
        else:                                                      # OpenCV 4.7 and newer
            self.params = cv2.aruco.DetectorParameters()
        self.params.cornerRefinementMethod = (
            cv2.aruco.CORNER_REFINE_APRILTAG if self.apriltag else cv2.aruco.CORNER_REFINE_NONE)
        self.cells = self.dictionary.markerSize + 2      # bits plus the black border
        self.legacy = not hasattr(cv2.aruco, 'ArucoDetector')
        self.detector = None if self.legacy else cv2.aruco.ArucoDetector(self.dictionary, self.params)
        self.subpix = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)

    def detect_all(self, image) -> List[Detection]:
        """Every decoded marker; `rejected` says why one is not to be used."""
        cv2 = self.cv2
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if self.legacy:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        else:
            corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return []
        h, w = gray.shape[:2]
        b = self.border_margin
        out = []
        for c, i in zip(corners, ids.ravel()):
            c = c.reshape(4, 2).astype(np.float32)
            if c.min() < b or (c[:, 0] > w - 1 - b).any() or (c[:, 1] > h - 1 - b).any():
                out.append(Detection(int(i), c.astype(float), 'border'))
                continue
            c = c.copy()
            cv2.cornerSubPix(gray, c, self.window(c), (-1, -1), self.subpix)
            fine = refine_edges(gray, c, self.cells) if self.edges else None
            d = Detection(int(i), (fine if fine is not None else c).astype(float))
            if d.min_edge() < self.min_marker_px:
                d.rejected = 'small'
            out.append(d)
        return out

    def window(self, corners) -> Tuple[int, int]:
        """cornerSubPix half-window: half a cell, 2 or 3 px; a larger window reaches the
        bits inside a small marker and pulls the corner onto them."""
        edge = min(np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4))
        w = int(min(3, max(2, round(0.5 * edge / self.cells))))
        return (w, w)

    def detect(self, image) -> List[Detection]:
        """The usable markers only."""
        return [d for d in self.detect_all(image) if d.rejected is None]


def refine_edges(gray, corners, cells: int, width: float = 0.7, ends: int = 4) -> Optional[np.ndarray]:
    """Corners as the intersections of the four edges of the black square, each edge a
    line fitted to points where the intensity crosses half way from the black border to
    the white margin, sampled across the edge away from the corners. Unbiased for any
    symmetric blur; cornerSubPix alone leaves the corners 0.15 to 0.2 px inside the
    square (a rounded apex), which makes markers look smaller and farther away.
    None if an edge has too little contrast."""
    import cv2
    c = np.asarray(corners, float)
    ctr = c.mean(axis=0)
    edge = min(np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4))
    r = float(np.clip(width * edge / cells, 1.0, 4.0))   # part of a cell each way, 1 to 4 px
    offs = np.arange(-r, r + 1e-9, 0.25)
    g = gray.astype(np.float32)
    lines = []
    for i in range(4):
        a, b = c[i], c[(i + 1) % 4]
        length = np.linalg.norm(b - a)
        t = (b - a) / length
        n = np.array([-t[1], t[0]])
        if ((a + b) / 2 - ctr) @ n < 0:                   # n points out of the square
            n = -n
        s = np.linspace(0.15, 0.85, max(5, int(length)))
        P = a[None, :] + (s * length)[:, None] * t[None, :]
        Q = P[:, None, :] + offs[None, :, None] * n[None, None, :]
        prof = cv2.remap(g, Q[..., 0].astype(np.float32), Q[..., 1].astype(np.float32),
                         cv2.INTER_LINEAR)
        third = max(1, len(offs) // ends)
        pts = []
        for j in range(len(P)):
            p = prof[j]
            lo, hi = p[:third].mean(), p[-third:].mean()
            if hi - lo < 20:
                continue
            mid = (lo + hi) / 2
            above = np.nonzero(p > mid)[0]
            if not len(above) or above[0] == 0:
                continue
            k = above[0]
            f = (mid - p[k - 1]) / (p[k] - p[k - 1])
            pts.append(P[j] + (offs[k - 1] + f * (offs[k] - offs[k - 1])) * n)
        if len(pts) < 3:
            return None
        pts = np.array(pts)
        mu = pts.mean(axis=0)
        _, _, vt = np.linalg.svd(pts - mu)
        lines.append((mu, vt[0]))
    out = []
    for i in range(4):
        (p1, d1), (p0, d0) = lines[i], lines[i - 1]
        try:
            st = np.linalg.solve(np.array([d0, -d1]).T, p1 - p0)
        except np.linalg.LinAlgError:
            return None
        out.append(p0 + st[0] * d0)
    out = np.array(out)
    if np.abs(out - c).max() > 2.0:                       # not where the detector saw it
        return None
    return out


def marker_pose_in_camera(det: Detection, K, size: float, dist=None) -> Tuple[np.ndarray, float]:
    """T_cam_marker from one detection and its RMS reprojection error [px].

    Candidates: both solutions of IPPE_SQUARE and of the general planar IPPE, the
    iterative solver and SQPnP; the one that reprojects best is refined
    (Levenberg-Marquardt). IPPE alone is not enough: on OpenCV 4.5 it returns a
    wrong rotation (20 px reprojection error) or NaN for a marker whose centre is
    at the camera's height, which is common for wall boards seen from a vehicle."""
    import cv2
    obj = marker_object_points(size)
    img = np.asarray(det.corners, float).reshape(-1, 1, 2)
    K = np.asarray(K, float)
    dist = np.zeros(5) if dist is None else np.asarray(dist, float)
    cands = []
    flags = [cv2.SOLVEPNP_IPPE_SQUARE, cv2.SOLVEPNP_IPPE, cv2.SOLVEPNP_ITERATIVE]
    if hasattr(cv2, 'SOLVEPNP_SQPNP'):
        flags.append(cv2.SOLVEPNP_SQPNP)
    for flag in flags:
        try:
            n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, img, K, dist, flags=flag)
        except cv2.error:
            continue
        cands += [(r, t) for r, t in zip(rvecs[:n], tvecs[:n])
                  if np.all(np.isfinite(r)) and np.all(np.isfinite(t)) and t.ravel()[2] > 0]

    def rms(r, t):
        proj, _ = cv2.projectPoints(obj, r, t, K, dist)
        return float(np.sqrt(np.mean(np.sum((proj - img) ** 2, axis=2))))

    if not cands:
        return np.full((4, 4), np.nan), float('nan')
    r, t = min(cands, key=lambda c: rms(*c))
    r, t = cv2.solvePnPRefineLM(obj, img, K, dist, r.copy(), t.copy())
    R, _ = cv2.Rodrigues(r)
    return make_T(R, t.ravel()), rms(r, t)
