import math

import numpy as np
import pytest

from marker_survey.core import marker_map as mm
from marker_survey.core import simulate as sim
from marker_survey.core.detection import Detector, is_apriltag, opencv_dictionary
from marker_survey.core.render import Renderer
from marker_survey.core.se3 import make_T


def ceiling_marker(x, y, z, rot):
    return mm.MapMarker(0, x, y, z, 'ceiling', {'rotation_deg': rot}).pose()


def view(dictionary, markers, size, T_world_cam, camera):
    r = Renderer(opencv_dictionary(dictionary), camera.K, camera.width, camera.height)
    return r.render(markers, {k: size for k in markers}, T_world_cam)


@pytest.mark.parametrize('dictionary', ['tag36h11', 'DICT_5X5_1000', 'DICT_APRILTAG_25h9'])
def test_rendered_corners_are_found_within_a_fraction_of_a_pixel(dictionary):
    cam = sim.upward_camera()
    markers = {3: ceiling_marker(0.4, 0.3, 3.0, 20.0), 8: ceiling_marker(-0.5, -0.2, 3.0, -50.0)}
    T_wc = make_T(np.eye(3), [0, 0, 0]) @ cam.T_base_cam
    img = view(dictionary, markers, 0.45, T_wc, cam)
    dets = Detector(dictionary).detect(img)
    assert sorted(d.id for d in dets) == [3, 8]
    err = np.concatenate([np.linalg.norm(d.corners - sim.visible(T_wc, markers[d.id], 0.45, cam), axis=1)
                          for d in dets])
    assert np.median(err) < 0.3 and err.max() < 0.8, err


def test_apriltag_families_get_their_settings():
    import cv2
    assert is_apriltag('tag36h11') and not is_apriltag('DICT_5X5_1000')
    assert Detector('tag36h11').params.cornerRefinementMethod == cv2.aruco.CORNER_REFINE_APRILTAG
    assert Detector('DICT_5X5_1000').params.cornerRefinementMethod == cv2.aruco.CORNER_REFINE_NONE


def test_a_marker_cut_by_the_border_is_rejected():
    cam = sim.upward_camera()
    T_wc = cam.T_base_cam
    # the marker straddles the image edge: x far enough that one side leaves the view
    edge = 2.7 * math.tan(math.radians(35))       # half the view width at the ceiling
    markers = {5: ceiling_marker(0.0, -edge, 3.0, 0.0)}
    img = view('tag36h11', markers, 0.45, T_wc, cam)
    det = Detector('tag36h11')
    every = det.detect_all(img)
    assert [d.rejected for d in every if d.id == 5] in ([], ['border'])
    assert det.detect(img) == []


def test_a_small_marker_is_rejected():
    cam = sim.upward_camera()
    markers = {5: ceiling_marker(0.0, 0.0, 3.0, 0.0)}
    img = view('tag36h11', markers, 0.18, cam.T_base_cam, cam)    # about 30 px on the image
    det = Detector('tag36h11', min_marker_px=40.0)
    every = det.detect_all(img)
    assert [d.rejected for d in every] == ['small']
    assert det.detect(img) == []
    assert [d.id for d in Detector('tag36h11', min_marker_px=10.0).detect(img)] == [5]
