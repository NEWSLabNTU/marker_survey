"""Render what a camera sees of a set of markers: real marker images warped onto
their corners. No ROS.

A plain grey scene with every marker in view drawn with a white margin of
one cell. Good enough to exercise detection and corner refinement; no
lighting, blur or lens distortion.
"""

from typing import Dict

import numpy as np

from marker_survey.core.detection import marker_object_points
from marker_survey.core.se3 import inv_T

CELL_PX = 40


def _marker_image(dictionary, mid: int):
    import cv2
    cells = dictionary.markerSize + 2
    side = cells * CELL_PX
    gen = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    mk = gen(dictionary, int(mid), side)
    q = CELL_PX
    big = np.full((side + 2 * q, side + 2 * q), 255, np.uint8)
    big[q:q + side, q:q + side] = mk
    # the black square's outer edges, in pixel-centre coordinates (OpenCV's)
    a, b = q - 0.5, q + side - 0.5
    src = np.float32([[a, a], [b, a], [b, b], [a, b]])
    return big, src


class Renderer:
    """Draws markers as a camera with intrinsics K sees them. Each marker is warped at
    `supersample` times the resolution inside its bounding box and the image is then
    averaged down (INTER_AREA), which gives anti-aliased edges like a real lens and
    sensor; without it, the edges of small markers jump by up to a pixel."""

    def __init__(self, dictionary, K, width: int, height: int, background: int = 160,
                 supersample: int = 3):
        self.dictionary = dictionary
        self.K = np.asarray(K, float)
        self.W, self.H = int(width), int(height)
        self.background = background
        self.ss = max(1, int(supersample))
        self._cache = {}

    def render(self, markers: Dict[int, np.ndarray], sizes: Dict[int, float],
               T_world_cam: np.ndarray) -> np.ndarray:
        import cv2
        W, H, ss = self.W, self.H, self.ss
        img = np.full((H * ss, W * ss), self.background, np.uint8)
        T_cw = inv_T(T_world_cam)
        order = []
        for mid, Tm in markers.items():
            obj = marker_object_points(sizes[mid])
            Xw = obj @ Tm[:3, :3].T + Tm[:3, 3]
            Xc = Xw @ T_cw[:3, :3].T + T_cw[:3, 3]
            if np.any(Xc[:, 2] < 0.3):
                continue
            v = T_world_cam[:3, 3] - Tm[:3, 3]
            if (v / np.linalg.norm(v)) @ Tm[:3, 2] < 0.2:
                continue
            uv = Xc @ self.K.T
            uv = uv[:, :2] / uv[:, 2:]
            if uv[:, 0].max() < 0 or uv[:, 0].min() > W or uv[:, 1].max() < 0 or uv[:, 1].min() > H:
                continue
            order.append((float(Xc[:, 2].mean()), mid, uv))
        for _, mid, uv in sorted(order, reverse=True):      # far first, near on top
            if mid not in self._cache:
                self._cache[mid] = _marker_image(self.dictionary, mid)
            big, src = self._cache[mid]
            # pixel centres: image (x, y) is supersampled (ss x + (ss - 1) / 2, ...)
            dst = uv * ss + (ss - 1) / 2.0
            M = cv2.getPerspectiveTransform(src, np.float32(dst))
            # the white margin's outline bounds the warp
            h_, w_ = big.shape
            box = cv2.perspectiveTransform(
                np.float32([[[-0.5, -0.5]], [[w_ - 0.5, -0.5]], [[w_ - 0.5, h_ - 0.5]],
                            [[-0.5, h_ - 0.5]]]), M).reshape(-1, 2)
            x0 = max(0, int(np.floor(box[:, 0].min())) - 1)
            y0 = max(0, int(np.floor(box[:, 1].min())) - 1)
            x1 = min(W * ss, int(np.ceil(box[:, 0].max())) + 2)
            y1 = min(H * ss, int(np.ceil(box[:, 1].max())) + 2)
            if x1 <= x0 or y1 <= y0:
                continue
            Ms = np.array([[1, 0, -x0], [0, 1, -y0], [0, 0, 1]], float) @ M
            size = (x1 - x0, y1 - y0)
            warped = cv2.warpPerspective(big, Ms, size, flags=cv2.INTER_LINEAR, borderValue=0)
            mask = cv2.warpPerspective(np.full_like(big, 255), Ms, size, flags=cv2.INTER_LINEAR,
                                       borderValue=0)
            roi = img[y0:y1, x0:x1]
            a = mask.astype(np.float32) / 255.0
            roi[:] = (a * warped + (1 - a) * roi).astype(np.uint8)
        if ss > 1:
            img = cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)
        return img
