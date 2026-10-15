"""The camera-only survey (pose_source markers): a handheld camera walked under ceiling
markers, every frame placed on the markers already mapped."""
import math

import numpy as np
import pytest

from marker_survey.core import marker_map as mm
from marker_survey.core import simulate as sim
from marker_survey.core.frames import FrameSettings, ceiling_vertical, origin_marker_frame
from marker_survey.core.se3 import inv_T, rotation_angle
from marker_survey.core.solve import finish
from marker_survey.core.survey import SurveySettings
from marker_survey.core.tracking import MarkerTracker, place_camera

SIZE = 0.40
# a 3 x 3 grid at 1.4 m under a 2.9 m ceiling, marker 0 in the middle
GRID = [(x, y) for y in (-1.4, 0.0, 1.4) for x in (-1.4, 0.0, 1.4)]
ROTATIONS = [35.0, -120.0, 80.0, 170.0, 0.0, -45.0, 100.0, -10.0, 60.0]
# each row, then each column; the camera tilted away for 3 s on the middle column
ROUTE = [(0.0, 0.0), (-1.4, 0.0), (-1.4, -1.4), (1.4, -1.4), (1.4, 1.4), (-1.4, 1.4), (-1.4, -1.4),
         (0.0, -1.4), (0.0, 1.4), (1.4, 1.4), (1.4, 0.0), (0.0, 0.0)]


def truth():
    out = {}
    for n, ((x, y), rot) in enumerate(zip(GRID, ROTATIONS)):
        mid = 0 if (x, y) == (0.0, 0.0) else n + 1
        out[mid] = mm.MapMarker(mid, x, y, 2.9, 'ceiling', {'rotation_deg': rot}).pose()
    return out


def quiet(*a):
    pass


@pytest.fixture(scope='module')
def handheld():
    markers = truth()
    hand = sim.Hand(tilt_at=36.0, seed=2)
    poses = sim.walk(ROUTE, speed=0.5, rate=5.0, hand=hand)
    sizes = {k: SIZE for k in markers}
    sv, tr = sim.simulate_markers(markers, sizes, poses, sim.handheld_camera(),
                                  SurveySettings(marker_size=SIZE), origin_marker=0,
                                  floor_offset=1.0, vertical='ceiling', seed=2)
    frame = FrameSettings('origin_marker', 0, floor_offset=1.0, vertical='ceiling')
    return markers, poses, hand, sv, tr, finish(sv, frame, 'tag36h11', None, log=quiet)


def test_every_marker_is_mapped_and_good(handheld):
    markers, _, _, sv, _, res = handheld
    assert {m.id for m in res['markers']} == set(markers)
    for m in res['markers']:
        assert m.verdict == 'good', (m.id, m.flags)


def test_markers_land_within_two_centimetres_of_the_truth(handheld):
    markers, _, _, _, _, res = handheld
    Ti = inv_T(origin_marker_frame(markers[0], np.zeros(3), np.array([0, 0, 1.0])))
    err = [np.linalg.norm(m.T[:3, 3] - (Ti @ markers[m.id])[:3, 3]) for m in res['markers']]
    assert np.median(err) < 0.015 and max(err) < 0.03, err


def test_the_floor_is_the_camera_height_below_the_camera(handheld):
    _, _, _, _, _, res = handheld
    o = next(m for m in res['markers'] if m.id == 0)
    assert (o.x, o.y) == pytest.approx((0.0, 0.0), abs=1e-6)
    assert o.z == pytest.approx(2.9, abs=0.02)


def test_no_motion_factor_and_a_reported_uncertainty_that_holds(handheld):
    markers, _, _, sv, _, res = handheld
    assert 'motion' not in res['info']['factors']
    Ti = inv_T(origin_marker_frame(markers[0], np.zeros(3), np.array([0, 0, 1.0])))
    for m in res['markers']:
        d = m.T[:3, 3] - (Ti @ markers[m.id])[:3, 3]
        if m.id != 0:
            assert np.hypot(d[0], d[1]) < 4 * m.frame_sd_xy + 0.003, (m.id, d, m.frame_sd_xy)
            assert abs(d[2]) < 4 * m.frame_sd_z + 0.003, (m.id, d, m.frame_sd_z)
        # a ceiling marker's angle sd stays small: the solve's own rotation vectors are used
        assert m.sd_angle_deg < 1.0, (m.id, m.sd_angle_deg)


def test_frames_in_the_tilted_stretch_are_not_placed_and_tracking_resumes(handheld):
    _, poses, hand, sv, tr, _ = handheld
    st = tr.status()
    assert st['unplaced'] >= int(2.0 * 5)          # at least the 2.4 s fully tilted away
    assert st['tracking'] == 'ok' and st['lost_since'] is None
    t0, t1 = sim.tilt_window(hand)
    stamps = [kf.stamp for kf in sv.keyframes]
    assert not any(t0 <= t <= t1 for t in stamps)
    assert any(t > t1 for t in stamps)


def test_lost_while_no_mapped_marker_is_in_view():
    markers = truth()
    sv = sim.Survey(SurveySettings(marker_size=SIZE), sim.handheld_camera().K)
    sv.T_cam_base = np.eye(4)
    tr = MarkerTracker(sv)
    assert tr.status()['tracking'] == 'ok' and tr.status()['last_known_marker'] is None
    cam = sim.handheld_camera()
    rng = np.random.default_rng(0)
    up = sim.walk([(0.0, 0.0), (0.1, 0.0)], hand=sim.Hand(shake_deg=0.0, yaw_shake_deg=0.0, bob=0.0))[0]
    dets = sim.project(markers, {k: SIZE for k in markers}, cam, up, rng, 0.3)
    T = tr.track(0.0, dets)
    assert T is not None
    sv.add(0.0, T, dets)
    assert tr.status()['last_known_marker'] == 0
    assert tr.track(0.2, []) is None
    assert tr.track(1.2, []) is None
    st = tr.status()
    assert st['tracking'] == 'lost' and st['lost_since'] == pytest.approx(1.2)
    assert st['last_known_marker'] == 0 and st['unplaced'] == 2
    assert tr.track(1.4, dets) is not None
    assert tr.status()['tracking'] == 'ok'


def test_place_camera_on_the_mapped_markers():
    markers = truth()
    cam = sim.handheld_camera(hfov_deg=110.0)         # wide: several markers in view
    rng = np.random.default_rng(1)
    T_true = sim.walk([(0.7, 0.05), (0.8, 0.05)], hand=sim.Hand(seed=4))[0]
    dets = sim.project(markers, {k: SIZE for k in markers}, cam, T_true, rng, 0.3)
    assert len(dets) >= 3
    pl = place_camera(dets, markers, lambda i: SIZE, cam.K)
    assert np.linalg.norm(pl.T_world_cam[:3, 3] - T_true[:3, 3]) < 0.02
    assert math.degrees(rotation_angle(pl.T_world_cam[:3, :3], T_true[:3, :3])) < 0.5
    # a marker moved since it was mapped is left out
    moved = dict(markers)
    moved[dets[0].id] = markers[dets[0].id] @ sim.make_T(np.eye(3), [0.3, 0.0, 0.0])
    pl = place_camera(dets, moved, lambda i: SIZE, cam.K)
    assert dets[0].id in pl.dropped
    assert np.linalg.norm(pl.T_world_cam[:3, 3] - T_true[:3, 3]) < 0.03
    assert place_camera(dets, {}, lambda i: SIZE, cam.K) is None


def test_ceiling_vertical_from_the_markers_plane():
    markers = truth()
    tilt = sim.make_T(sim.rotvec_to_matrix([0.02, -0.01, 0.0]), [0, 0, 0])
    up = ceiling_vertical([tilt @ T for T in markers.values()])
    assert up == pytest.approx(tilt[:3, 2], abs=1e-9)
    with pytest.raises(ValueError, match='not all on the ceiling'):
        wall = mm.MapMarker(9, 0, 0, 1, 'wall', {}).pose()
        ceiling_vertical(list(markers.values()) + [wall])
