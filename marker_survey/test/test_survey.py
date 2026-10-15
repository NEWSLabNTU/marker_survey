import math
import os

import numpy as np
import pytest
import yaml

from marker_survey.core import marker_map as mm
from marker_survey.core import simulate as sim
from marker_survey.core.detection import Detector, opencv_dictionary
from marker_survey.core.frames import FrameSettings, origin_marker_frame
from marker_survey.core.report import surveyed_markers
from marker_survey.core.se3 import inv_T
from marker_survey.core.solve import finish, load_dump, save_dump

# a small room: six 0.38 m boards around a 6 m x 3 m loop: (x, y, facing_deg)
BOARDS = {
    24: (8.0, 1.5, 180.0),
    25: (6.0, -2.0, 120.0),
    26: (2.0, -2.0, 120.0),
    27: (-2.0, 1.5, 0.0),
    28: (2.0, 5.0, -60.0),
    29: (6.0, 5.0, -120.0),
}
LOOP = [(0, 0), (6, 0), (7.5, 1.5), (6, 3), (0, 3), (-1, 1.5), (0, 0)]
# boards standing on the floor but not quite upright: (lean_deg, turn_deg)
TILT = {25: (6.9, 0.0), 27: (0.0, -5.7), 29: (-3.4, 2.9)}
SIZE = 0.38


def board(x, y, z, facing, lean=0.0, turn=0.0):
    return mm.MapMarker(0, x, y, z, 'wall', {'facing_deg': facing, 'lean_deg': lean,
                                             'turn_deg': turn}).pose()


def run(truth, poses, mode='origin_marker', seed=3, **kw):
    noise = sim.Noise(seed=seed)
    frames = sim.make_frames(poses, noise)
    sizes = {k: SIZE for k in truth}
    return sim.simulate(truth, sizes, frames, sim.forward_camera(), sim.settings_for(mode, SIZE),
                        mode, noise)


def quiet(*a):
    pass


@pytest.fixture(scope='module')
def result():
    truth = {mid: board(x, y, 0.40, f, *TILT.get(mid, (0.0, 0.0)))
             for mid, (x, y, f) in BOARDS.items()}
    poses = sim.drive(LOOP, step=0.05, sweeps={2: math.radians(50), 5: math.radians(50)})
    sv = run(truth, poses)
    return truth, sv, finish(sv, FrameSettings('origin_marker', 24), 'DICT_5X5_1000', None, log=quiet)


def test_every_marker_is_found(result):
    _, _, res = result
    assert {m.id for m in res['markers']} == set(BOARDS)


def test_markers_land_within_a_few_centimetres(result):
    truth, _, res = result
    Ti = inv_T(origin_marker_frame(truth[24], np.zeros(3), np.array([0, 0, 1.0])))
    for m in res['markers']:
        Tt = Ti @ truth[m.id]
        assert np.linalg.norm(m.T[:3, 3] - Tt[:3, 3]) < 0.03, m
        got, want = m.angles(), mm.angles_from_rotation('wall', Tt[:3, :3])
        for k in ('facing_deg', 'lean_deg', 'turn_deg'):
            d = (got[k] - want[k] + 180) % 360 - 180
            assert abs(d) < 1.0, (m.id, k, got[k], want[k])
        assert m.verdict == 'good', (m.id, m.flags)


def test_the_origin_marker_defines_the_frame(result):
    _, _, res = result
    o = next(m for m in res['markers'] if m.id == 24)
    assert (o.x, o.y, o.angles()['facing_deg']) == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)
    assert o.z == pytest.approx(0.40, abs=0.02)


def test_the_start_pose_is_where_the_drive_began(result):
    _, _, res = result
    # the drive starts at the world origin heading +x; marker 24 at (8, 1.5) faces -x,
    # so the frame's x axis is world -x and its y axis world -y
    sp = res['start_pose']
    assert sp['x'] == pytest.approx(8.0, abs=0.05)
    assert sp['y'] == pytest.approx(1.5, abs=0.05)
    assert abs((sp['yaw_deg'] - 180 + 180) % 360 - 180) < 1.0


def test_writes_a_marker_map_and_a_report(result, tmp_path):
    truth, sv, _ = result
    res = finish(sv, FrameSettings('origin_marker', 24), 'DICT_5X5_1000', str(tmp_path), log=quiet)
    m = mm.load(str(tmp_path / 'marker_map.yaml'))
    assert set(m.ids()) == set(BOARDS)
    assert m.dictionary == 'DICT_5X5_1000' and m.marker_size == SIZE and m.mount == 'wall'
    assert m.header[0].startswith('Surveyed ')
    Ti = inv_T(origin_marker_frame(truth[24], np.zeros(3), np.array([0, 0, 1.0])))
    for mid in BOARDS:   # the map carries the measured tilt
        n_true = (Ti @ truth[mid])[:3, 2]
        assert m.pose_of(mid)[:3, 2] @ n_true > math.cos(math.radians(1.0)), mid
    rep = yaml.safe_load((tmp_path / 'survey_report.yaml').read_text())
    assert rep['frame']['mode'] == 'origin_marker' and rep['frame']['origin_marker'] == 24
    assert {r['id'] for r in rep['markers']} == set(BOARDS)
    assert all(r['verdict'] in ('good', 'pass again', 'bad') for r in rep['markers'])
    assert len(res['written']) == 2


def test_a_new_survey_keeps_the_comments_of_the_old_map(result, tmp_path):
    _, sv, _ = result
    finish(sv, FrameSettings('origin_marker', 24), 'DICT_5X5_1000', str(tmp_path), log=quiet)
    p = tmp_path / 'marker_map.yaml'
    text = p.read_text().replace('# Surveyed', '# Boards of the hall.\n# Surveyed')
    lines = text.splitlines()
    i = next(n for n, ln in enumerate(lines) if ln.strip().startswith('- {id: 27,'))
    lines[i] += '   # by the door'
    p.write_text('\n'.join(lines) + '\n')
    finish(sv, FrameSettings('origin_marker', 24), 'DICT_5X5_1000', str(tmp_path), log=quiet)
    m = mm.load(str(p))
    assert m.by_id()[27].comment == 'by the door'
    assert 'Boards of the hall.' in m.header
    assert sum(1 for h in m.header if h and h.startswith('Surveyed ')) == 1


def test_dump_round_trip_solves_the_same(result, tmp_path):
    _, sv, res = result
    p = tmp_path / 'survey_data.npz'
    save_dump(str(p), sv, 'DICT_5X5_1000', FrameSettings('origin_marker', 24))
    sv2, dic, frame = load_dump(str(p))
    assert (dic, frame.mode, frame.origin_marker) == ('DICT_5X5_1000', 'origin_marker', 24)
    assert sv2.s.marker_size_sd == sv.s.marker_size_sd
    res2 = finish(sv2, frame, dic, None, log=quiet)
    a = {m.id: m.T[:3, 3] for m in res['markers']}
    b = {m.id: m.T[:3, 3] for m in res2['markers']}
    for k in a:
        assert a[k] == pytest.approx(b[k], abs=0.005)


def test_an_unseen_origin_is_refused(result):
    _, sv, _ = result
    with pytest.raises(ValueError, match='never seen'):
        finish(sv, FrameSettings('origin_marker', 99), 'DICT_5X5_1000', None, log=quiet)


def test_per_marker_sizes_are_used():
    truth = {mid: board(x, y, 0.40, f) for mid, (x, y, f) in BOARDS.items()}
    sizes = {k: (0.25 if k == 26 else SIZE) for k in truth}
    poses = sim.drive(LOOP, step=0.05, sweeps={2: math.radians(50)})
    noise = sim.Noise(seed=4)
    frames = sim.make_frames(poses, noise)
    settings = sim.settings_for('origin_marker', SIZE, {26: 0.25})
    sv = sim.simulate(truth, sizes, frames, sim.forward_camera(), settings, 'origin_marker', noise)
    res = finish(sv, FrameSettings('origin_marker', 24), 'DICT_5X5_1000', None, log=quiet)
    Ti = inv_T(origin_marker_frame(truth[24], np.zeros(3), np.array([0, 0, 1.0])))
    m26 = next(m for m in res['markers'] if m.id == 26)
    assert np.linalg.norm(m26.T[:3, 3] - (Ti @ truth[26])[:3, 3]) < 0.03
    assert res['map'].by_id()[26].size == 0.25


def test_dictionaries():
    opencv_dictionary('DICT_5X5_1000')
    opencv_dictionary('tag36h11')
    with pytest.raises(ValueError):
        opencv_dictionary('ALL_DICTS')


def test_the_detector_reads_a_rendered_marker():
    import cv2
    d = opencv_dictionary('DICT_5X5_1000')
    gen = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    img = np.full((400, 400), 255, np.uint8)
    img[100:300, 100:300] = gen(d, 24, 200)
    dets = Detector('DICT_5X5_1000').detect(img)
    assert [x.id for x in dets] == [24]
    c = dets[0].corners          # TL, TR, BR, BL
    assert c[0][0] < c[1][0] and c[0][1] < c[3][1]


def test_the_detector_works_without_arucodetector(monkeypatch):
    """OpenCV 4.5 and 4.6 have no ArucoDetector: the function API must serve."""
    import cv2
    real = cv2.aruco
    calls = {}

    class Legacy:
        pass

    legacy = Legacy()
    for name in dir(real):
        if name in ('ArucoDetector', 'DetectorParameters', 'DetectorParameters_create'):
            continue
        setattr(legacy, name, getattr(real, name))
    make_params = getattr(real, 'DetectorParameters_create', None) or real.DetectorParameters

    def detect_markers(gray, dictionary, parameters=None):
        calls['params'] = parameters
        if hasattr(real, 'ArucoDetector'):
            return real.ArucoDetector(dictionary, parameters).detectMarkers(gray)
        return real.detectMarkers(gray, dictionary, parameters=parameters)

    legacy.DetectorParameters_create = make_params
    legacy.detectMarkers = detect_markers
    monkeypatch.setattr(cv2, 'aruco', legacy)
    gen = getattr(real, 'generateImageMarker', None) or real.drawMarker
    img = np.full((400, 400), 255, np.uint8)
    img[100:300, 100:300] = gen(real.getPredefinedDictionary(real.DICT_5X5_1000), 24, 200)
    det = Detector('DICT_5X5_1000')
    assert det.legacy
    assert [d.id for d in det.detect(img)] == [24]
    assert calls['params'].cornerRefinementMethod == real.CORNER_REFINE_NONE


def test_a_single_keyframe_is_saved_without_adjustment(tmp_path):
    truth = {24: board(3.0, 0.0, 0.40, 180.0), 25: board(3.0, 1.2, 0.40, 180.0)}
    sv = run(truth, [sim.drive([(0, 0), (0.05, 0)], step=0.05)[0]], seed=2)
    assert len(sv.keyframes) == 1
    with pytest.raises(ValueError, match='not enough data'):
        sv.solve()
    logs = []
    res = finish(sv, FrameSettings('origin_marker', 24), 'DICT_5X5_1000', str(tmp_path),
                 log=logs.append)
    assert any('single keyframe' in str(x) and 'quick look' in str(x) for x in logs)
    assert res['info']['status'] == 'single keyframe, no bundle adjustment'
    assert all(m.sd_xy is None and m.sd_angle_deg is None for m in res['markers'])
    rep = yaml.safe_load((tmp_path / 'survey_report.yaml').read_text())
    assert rep['solve']['status'] == 'single keyframe, no bundle adjustment'
    assert rep['markers'][0]['sd_xy_cm'] is None
    m = {x.id: x for x in res['markers']}
    assert m[24].x == pytest.approx(0.0, abs=0.02) and m[24].z == pytest.approx(0.40, abs=0.03)
    assert abs(m[25].y) == pytest.approx(1.2, abs=0.05)


def _marker_list(Ts, origin=24):
    T_tw = inv_T(origin_marker_frame(Ts[origin], np.zeros(3), np.array([0, 0, 1.0])))
    out = surveyed_markers(Ts, {}, T_tw, {k: SIZE for k in Ts}, np.zeros(3), np.array([0, 0, 1.0]))
    return {m.id: m for m in out}


def test_rotating_a_board_in_its_plane_is_not_a_tilt():
    Ts = {24: board(0, 0, 0.4, 0.0), 30: board(2, 1, 0.4, 40.0, 0.0, 90.0),
          31: board(2, -1, 0.4, -23.0, 0.0, -90.0)}
    out = _marker_list(Ts)
    for mid in (30, 31):
        assert out[mid].tilt_deg == pytest.approx(0.0, abs=1e-6)
        assert not any('tilted' in f for f in out[mid].flags)
    assert out[30].angles()['turn_deg'] == pytest.approx(90.0, abs=1e-6)
    assert out[30].angles()['lean_deg'] == pytest.approx(0.0, abs=1e-6)


def test_tilt_is_the_lean_of_the_normal():
    Ts = {24: board(0, 0, 0.4, 0.0), 30: board(2, 1, 0.4, 40.0, 12.0, 90.0),
          31: board(2, -1, 0.4, -23.0, -12.0, 37.0)}
    out = _marker_list(Ts)
    assert out[30].tilt_deg == pytest.approx(12.0, abs=1e-6)
    assert out[31].tilt_deg == pytest.approx(12.0, abs=1e-6)
    assert any('tilted' in f for f in out[30].flags) and out[30].verdict == 'bad'
    assert out[30].angles()['lean_deg'] == pytest.approx(12.0, abs=1e-6)
    assert out[30].angles()['turn_deg'] == pytest.approx(90.0, abs=1e-6)


# ------------------------------------------------------------------ a vehicle on a stand

def _stand_survey(h):
    truth = {24: board(3.0, 0.0, 0.25, 180.0), 25: board(3.0, 1.2, 0.25, 180.0)}
    pose = sim.drive([(0, 0), (0.05, 0)], step=0.05)[0].copy()
    pose[2, 3] = h                                   # base h above the floor: wheels off
    return run(truth, [pose], seed=2)


def test_floor_offset_corrects_a_vehicle_on_a_stand(tmp_path):
    logs = []
    res = finish(_stand_survey(0.15), FrameSettings('origin_marker', 24), 'DICT_5X5_1000',
                 str(tmp_path), log=logs.append)
    z = {m.id: m.z for m in res['markers']}
    assert z[24] == pytest.approx(0.10, abs=0.03)           # true 0.25 - 0.15
    assert all(any('half the marker size' in f for f in m.flags) for m in res['markers'])
    assert any('floor_offset' in str(x) for x in logs)
    fixed = finish(_stand_survey(0.15), FrameSettings('origin_marker', 24, floor_offset=0.15),
                   'DICT_5X5_1000', None, log=quiet)
    for m in fixed['markers']:
        assert m.z == pytest.approx(0.25, abs=0.03)
        assert not any('half the marker size' in f for f in m.flags)
    for a, b in zip(res['markers'], fixed['markers']):   # x, y and facing do not change
        assert (a.x, a.y) == pytest.approx((b.x, b.y), abs=1e-6)


def test_a_marker_below_the_floor_is_bad():
    res = finish(_stand_survey(0.40), FrameSettings('origin_marker', 24), 'DICT_5X5_1000', None,
                 log=quiet)
    assert all(m.z < 0 and m.verdict == 'bad' and any('BELOW the floor' in f for f in m.flags)
               for m in res['markers'])


def test_a_lost_odometry_message_does_not_lose_the_frame():
    """The camera pose at an image's stamp comes from the odometry around it: across a
    lost message (a gap of two intervals) it is interpolated; only a long gap, or a
    stamp the odometry has not reached, gives none (the frame then waits)."""
    from marker_survey.core.se3 import make_T, pose_at_stamp, rot_z
    ts = [k / 9.0 for k in range(20)]
    Ts = [make_T(rot_z(0.1 * t), [0.3 * t, 0.0, 0.0]) for t in ts]
    del ts[9], Ts[9]                                    # one message lost
    t = 9.4 / 9.0
    T = pose_at_stamp(ts, Ts, t, 0.1, 0.5)
    assert T is not None and abs(T[0, 3] - 0.3 * t) < 1e-9
    assert pose_at_stamp(ts, Ts, 1.0, 0.1, 0.0) is None          # the old rule: dropped
    assert pose_at_stamp(ts, Ts, ts[-1] + 0.05, 0.1, 0.5) is Ts[-1]
    assert pose_at_stamp(ts, Ts, ts[-1] + 0.5, 0.1, 0.5) is None  # not there yet: wait
    del ts[5:12], Ts[5:12]                               # a long gap
    assert pose_at_stamp(ts, Ts, 8.0 / 9.0, 0.1, 0.5) is None
    assert pose_at_stamp([], [], 1.0, 0.1, 0.5) is None


def test_correlation_time_counts_independent_samples():
    from marker_survey.core.frames import correlation_time
    rng = np.random.default_rng(3)
    assert correlation_time(rng.normal(size=4000)) == pytest.approx(1.0, abs=0.3)
    x = np.zeros(4000)
    for k in range(1, len(x)):                  # AR(1), rho 0.9: (1 + rho) / (1 - rho) = 19
        x[k] = 0.9 * x[k - 1] + rng.normal()
    assert correlation_time(x) == pytest.approx(19.0, rel=0.3)
    assert correlation_time(np.ones(10)) == 1.0


def test_the_alignment_covariance_does_not_shrink_with_correlated_residuals():
    """The fit's 1-sigma for a track error that is independent per keyframe falls as
    1 / sqrt(N); for the same error held over long stretches it stays near the error."""
    from marker_survey.core.frames import align_to_track, alignment_covariance
    from marker_survey.core.se3 import make_T, rot_z
    rng = np.random.default_rng(4)
    n = 600
    a = np.linspace(0, 4 * np.pi, n)
    base = [make_T(rot_z(t), [4 * np.cos(t), 2 * np.sin(t), 0.0]) for t in a]
    white = rng.normal(0, 0.05, (n, 2))
    slow = np.zeros((n, 2))
    for k in range(1, n):
        slow[k] = 0.99 * slow[k - 1] + math.sqrt(1 - 0.99 ** 2) * rng.normal(0, 0.05, 2)
    sds = {}
    for name, e in (('white', white), ('slow', slow)):
        refs = [T.copy() for T in base]
        for T, d in zip(refs, e):
            T[:2, 3] += d
        T_tw, res = align_to_track(base, refs)
        C, c, n_eff = alignment_covariance(base, refs, T_tw)
        assert C.shape == (6, 6) and np.allclose(C, C.T)
        sds[name] = (math.sqrt(C[3, 3]), n_eff)
    assert sds['white'][1] > 300 and sds['white'][0] < 0.005, sds
    assert sds['slow'][1] < 30 and sds['slow'][0] > 0.01, sds


def test_marker_size_sd_widens_the_frame_uncertainty_only():
    """marker_size_sd adds the size's error to the 1-sigma in the frame (mostly the
    distance from the camera: the height of a ceiling marker), not to the relative one
    the verdicts use."""
    m = mm.load(os.path.join(os.path.dirname(__file__), '..', '..', 'examples', 'ceiling', 'sites',
                             'office', 'marker_map.yaml'))
    route = [(-4.0, -2.0), (4.0, -2.0), (4.0, 2.0), (-4.0, 2.0)]
    noise = sim.Noise(pixel_sd=0.3, seed=2)
    frames = sim.make_frames(sim.route(route, laps=1, step=0.1), noise)
    out = {}
    for sd in (0.0, 0.01):
        s = sim.settings_for('origin_marker', m.marker_size, marker_size_sd=sd)
        sv = sim.simulate(m.poses(), m.sizes(), frames, sim.upward_camera(), s, 'origin_marker', noise)
        res = finish(sv, FrameSettings('origin_marker', 5), m.dictionary, None, log=lambda *a: None)
        out[sd] = {x.id: x for x in res['markers']}
    for mid, x in out[0.01].items():
        x0 = out[0.0][mid]
        assert x.sd_xy == pytest.approx(x0.sd_xy) and x.sd_z == pytest.approx(x0.sd_z)
        assert x.verdict == x0.verdict
        if mid != 5:
            # 1 % of the 2.7 m from the camera, in height
            assert x.frame_sd_z ** 2 - x0.frame_sd_z ** 2 > 0.01 ** 2, (mid, x.frame_sd_z)


def test_the_solve_tool_takes_marker_size_sd(result, tmp_path):
    from marker_survey.tools import solve as solve_tool
    _, sv, _ = result
    p = tmp_path / 'survey_data.npz'
    save_dump(str(p), sv, 'DICT_5X5_1000', FrameSettings('origin_marker', 24))
    sds = []
    for x in ('0', '0.02'):
        out = tmp_path / x
        assert solve_tool.main([str(p), '--output-dir', str(out), '--marker-size-sd', x]) == 0
        rep = yaml.safe_load((out / 'survey_report.yaml').read_text())
        sds.append(max(m['frame_sd_xy_cm'] for m in rep['markers']))
    assert sds[1] > sds[0], sds


def test_the_solve_tool_solves_again_with_a_measured_size(result, tmp_path):
    """--marker-size: the same recording solved with a size 1 % larger puts every
    board about 1 % farther from the camera."""
    from marker_survey.tools import solve as solve_tool
    _, sv, res = result
    p = tmp_path / 'survey_data.npz'
    save_dump(str(p), sv, 'DICT_5X5_1000', FrameSettings('origin_marker', 24))
    size = sv.s.marker_size * 1.01
    out = tmp_path / 'out'
    assert solve_tool.main([str(p), '--output-dir', str(out), '--marker-size', str(size)]) == 0
    got = mm.load(str(out / 'marker_map.yaml'))
    assert got.marker_size == pytest.approx(size)
    a = {m.id: m.T[:3, 3] for m in res['markers']}
    moved = [np.linalg.norm(got.pose_of(k)[:3, 3] - a[k]) for k in a]
    assert max(moved) > 0.005, moved
