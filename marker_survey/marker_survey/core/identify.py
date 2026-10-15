"""Which marker is this? Read the IDs from a photo of a sheet or a ceiling. No ROS.

A printed marker carries its own ID: if the label is cut off, the sheet is
mounted label side up or the list is lost, a photo tells. With the
dictionary unknown (`auto`), each common family is tried and the one that
finds the most markers wins (ties: the order of COMMON, tag36h11 first).

Phone photos are large; they are reduced to `max_side` pixels on the long
side before detection, and the corners scaled back.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from marker_survey.core.detection import Detector, opencv_dictionary_name

COMMON = ['tag36h11', 'DICT_4X4_1000', 'DICT_5X5_1000', 'DICT_6X6_1000', 'DICT_7X7_1000',
          'aruco_original', 'aruco_mip_36h12', 'tag25h9', 'tag16h5']


@dataclass
class Found:
    id: int
    dictionary: str
    corners: np.ndarray            # 4 x 2, full-image pixels, TL TR BR BL of the printed image

    def as_dict(self) -> dict:
        return {'id': self.id, 'dictionary': self.dictionary,
                'corners': [[round(float(x), 1), round(float(y), 1)] for x, y in self.corners]}


def _available(name: str) -> bool:
    import cv2
    return hasattr(cv2.aruco, opencv_dictionary_name(name))


def identify(image: np.ndarray, dictionary: str = 'auto', max_side: int = 2000) -> List[Found]:
    """Every marker in the image, with its ID, in one dictionary."""
    import cv2
    if image is None or image.size == 0:
        raise ValueError('the image is empty or could not be read')
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape[:2]
    scale = min(1.0, max_side / float(max(h, w)))
    small = gray if scale >= 1.0 else cv2.resize(gray, None, fx=scale, fy=scale,
                                                 interpolation=cv2.INTER_AREA)
    names = [n for n in COMMON if _available(n)] if dictionary == 'auto' else [dictionary]
    best: List[Found] = []
    for name in names:
        det = Detector(name, min_marker_px=8.0, border_margin=0.0, edges=False)
        found = [Found(d.id, name, (d.corners + 0.5) / scale - 0.5) for d in det.detect_all(small)]
        if len(found) > len(best):
            best = found
    return sorted(best, key=lambda f: f.id)


def annotate(image: np.ndarray, found: List[Found]) -> np.ndarray:
    """A copy of the image with each marker outlined and its ID in large print."""
    import cv2
    out = image.copy() if image.ndim == 3 else cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    for f in found:
        c = f.corners.astype(np.float32)
        edge = float(min(np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)))
        thick = max(2, int(edge / 40))
        cv2.polylines(out, [c.astype(np.int32)], True, (40, 200, 40), thick, cv2.LINE_AA)
        cv2.circle(out, tuple(int(v) for v in c[0]), thick * 2, (40, 40, 220), -1, cv2.LINE_AA)
        text = str(f.id)
        font_scale = max(0.8, edge / 160.0)
        (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thick * 2)
        cx, cy = c.mean(axis=0)
        org = (int(cx - tw / 2), int(cy + th / 2))
        pad = int(th * 0.3)
        cv2.rectangle(out, (org[0] - pad, org[1] - th - pad), (org[0] + tw + pad, org[1] + base + pad),
                      (255, 255, 255), -1)
        cv2.putText(out, text, org, cv2.FONT_HERSHEY_SIMPLEX, font_scale, (20, 20, 200), thick * 2,
                    cv2.LINE_AA)
    return out


def summary(found: List[Found]) -> Tuple[Optional[str], List[int]]:
    """(dictionary, IDs) of a result."""
    if not found:
        return None, []
    return found[0].dictionary, [f.id for f in found]
