"""Turn an accumulated survey into a marker map: solve, choose the frame, write. No ROS.

Shared by the survey node (on save, on exit) and the offline re-solver
(marker_survey_solve), which reads the survey's own recording
(survey_data.npz), so a recording can be solved again with other settings
without the robot.

Files written to the output directory:

    marker_map.yaml       the markers (see marker_map.py); comments of an
                          existing file are kept for the same marker ids
    survey_report.yaml    per marker: views, reprojection, uncertainty, tilt,
                          flags, verdict; the solve and the frame fit
"""

import datetime
import math
import os
from typing import Optional

import numpy as np

from marker_survey.core import frames as fr
from marker_survey.core import marker_map as mm
from marker_survey.core import report as rp
from marker_survey.core.detection import Detection, marker_pose_in_camera
from marker_survey.core.se3 import (T_from_params, inv_T, make_T, matrix_to_rotvec, params_from_T,
                                    rotvec_to_matrix)
from marker_survey.core.survey import (ALIGN_PARAMS, FLOOR_PARAMS, Keyframe, Survey, SurveySettings,
                                       floor_vector)

MAP_FILE = 'marker_map.yaml'
REPORT_FILE = 'survey_report.yaml'
DATA_FILE = 'survey_data.npz'


def finish(survey: Survey, frame: fr.FrameSettings, dictionary: str, output_dir: Optional[str],
           log=print, max_nfev: int = 300, report_settings: rp.ReportSettings = None) -> dict:
    frame.check()
    if frame.mode == 'origin_marker' and frame.origin_marker not in survey.markers:
        raise ValueError(f'origin marker {frame.origin_marker} was never seen '
                         f'(seen: {sorted(survey.markers)})')
    if survey.T_cam_base is None:
        raise ValueError('no camera -> base frame transform: the floor cannot be found')
    if len(survey.keyframes) == 1:
        markers, poses, stats, info = _single_keyframe(survey)
        log("WARNING: single keyframe, no bundle adjustment: the markers are this one view's "
            'estimates, with no uncertainty. A quick look, not a survey: drive past the '
            'markers for the real thing.')
    else:
        log(f'solving: {len(survey.keyframes)} keyframes, {len(survey.markers)} markers ...')
        # a reference track anchors the uncertainty only when the result is aligned to it
        # (the survey node records it whenever there is one, since the mode can change)
        track_sd = survey.s.track_sd
        if frame.mode != 'align_to_track':
            survey.s.track_sd = 0.0
        try:
            markers, poses, stats, info = survey.solve(
                max_nfev=max_nfev, align_gauge=frame.mode == 'align_to_track')
        finally:
            survey.s.track_sd = track_sd
        log(f'solved: reprojection RMS {info["reproj_rms_px"]:.2f} px, corner noise '
            f'{info["pixel_noise_px"]:.2f} px ({info["status"]})')
    base = [T @ survey.T_cam_base for T in poses]
    floor_p, floor_n = fr.floor_from_base_poses(base, frame.floor_offset)
    frame_info = {'mode': frame.mode, 'frame_id': frame.frame_id}
    align = None
    if frame.mode == 'origin_marker':
        T_tw = inv_T(fr.origin_marker_frame(markers[frame.origin_marker], floor_p, floor_n))
        frame_info['origin_marker'] = int(frame.origin_marker)
        frame_info['definition'] = ('origin on the floor below the origin marker centre, '
                                    'x the way it faces, z up')
    elif frame.mode == 'pose_frame':
        T_tw = np.eye(4)
        frame_info['definition'] = "the pose source's frame"
    else:
        refs = [kf.T_ref_base for kf in survey.keyframes]
        T_tw, res = fr.align_to_track(base, refs)
        C_align, c_align, n_eff = fr.alignment_covariance(base, refs, T_tw)
        align = (C_align, c_align)
        frame_info['definition'] = ('survey in the odometry frame, fitted rigidly to the '
                                    'reference trajectory')
        frame_info['fit_points'] = int(len(res))
        frame_info['fit_rms_m'] = round(float(np.sqrt(np.mean(res ** 2))), 4)
        frame_info['fit_max_m'] = round(float(res.max()), 4)
        frame_info['fit_samples'] = round(n_eff, 1)
        frame_info['fit_sd_xy_cm'] = round(100 * math.sqrt(max(
            np.linalg.eigvalsh(C_align[3:5, 3:5]).max(), 0.0)), 2)
        frame_info['fit_sd_yaw_deg'] = round(math.degrees(math.sqrt(max(C_align[2, 2], 0.0))), 3)
        log(f'aligned to the reference track: {len(res)} keyframes, residual RMS '
            f'{frame_info["fit_rms_m"] * 100:.1f} cm, max {frame_info["fit_max_m"] * 100:.1f} cm; '
            f'as independent as {n_eff:.0f} samples: the frame is known to '
            f'{frame_info["fit_sd_xy_cm"]:.1f} cm, {frame_info["fit_sd_yaw_deg"]:.2f} deg (1-sigma)')
    if frame.floor_offset:
        frame_info['floor_offset'] = float(frame.floor_offset)
    sizes = {mid: survey.s.size_of(mid) for mid in markers}
    exact_size = []         # the frame's 1-sigma with the size taken as exact
    for mid, (C, Cr, Cx) in target_covariances(survey, markers, T_tw, frame, base, align).items():
        stats[mid]['cov_target'] = C
        stats[mid]['cov_relative'] = Cr
        exact_size.append(rp.sds_of(Cx)[0])
    surveyed = rp.surveyed_markers(markers, stats, T_tw, sizes, floor_p, floor_n, report_settings)
    frame_sds = [m.frame_sd_xy for m in surveyed if m.frame_sd_xy is not None]
    if frame_sds:
        frame_info['frame_sd_xy_max_cm'] = round(max(frame_sds) * 100, 2)
        rel = [m.sd_xy for m in surveyed if m.sd_xy is not None]
        if frame.mode == 'origin_marker' and rel and exact_size and max(exact_size) > 2 * max(rel):
            log(f'in the frame of origin marker {frame.origin_marker}, the markers are known to '
                f'{max(exact_size) * 100:.1f} cm (1-sigma, the size taken as exact), among each other to '
                f'{max(rel) * 100:.1f} cm: the origin marker\'s facing turns the far ones. '
                'Harmless for localizing on this map; to tie the frame tighter, pass the origin '
                'marker from more places, or choose one in the middle of the site')
    start = rp.start_pose_in(T_tw, base[0])
    log(rp.format_table(surveyed))
    for m in surveyed:
        if m.verdict != rp.GOOD:
            log(f'marker {m.id}: {m.verdict}: {"; ".join(m.flags)}' + (f' ({m.hint})' if m.hint else ''))
    stamp = datetime.datetime.now().astimezone().isoformat(timespec='seconds')
    header = [f'Surveyed {stamp} by marker_survey, frame mode {frame.mode}'
              + (f' (origin marker {frame.origin_marker})' if frame.mode == 'origin_marker' else '')
              + '.']
    previous = None
    map_path = os.path.join(output_dir, MAP_FILE) if output_dir else None
    if map_path and os.path.exists(map_path):
        try:
            previous = mm.load(map_path)
        except mm.MarkerMapError as e:
            log(f'WARNING: the existing {map_path} does not load ({e}); its comments are lost')
    the_map = rp.build_map(surveyed, dictionary, survey.s.marker_size, frame.frame_id, header,
                           previous)
    report = rp.report_dict(surveyed, info, frame_info, dictionary, survey.s.marker_size, start)
    written = []
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        mm.write(the_map, map_path)
        rpath = os.path.join(output_dir, REPORT_FILE)
        rp.write_report(rpath, report)
        written = [map_path, rpath]
        for w in written:
            log(f'wrote {w}')
    return {'markers': surveyed, 'map': the_map, 'report': report, 'info': info,
            'frame': frame_info, 'T_target_world': T_tw, 'poses': poses, 'start_pose': start,
            'written': written}


def target_covariances(survey: Survey, markers, T_tw, frame: fr.FrameSettings, base_poses,
                       align=None):
    """6x6 covariances (rotation, translation) of every marker in the result frame:
    {id: (in the frame, relative, in the frame with the size taken as exact)}.

    In the frame: what the frame's definition adds included. In origin_marker mode
    the frame moves with the origin marker and with the floor (the base poses), so a
    small error of the origin marker's facing turns every far marker about it. In
    align_to_track mode (`align`: the fit's covariance and centroid,
    frames.alignment_covariance) the markers' covariance relative to the solved
    trajectory as a whole (the rigid motion that the fit would undo taken out), plus
    the fit's own: the survey's frame is the reference track's only as well as the
    fit placed it.

    Relative: the same with a common shift and turn about the vertical of the whole
    map taken out (an S-transformation to the markers' centroid): how well each marker
    sits among the others, what driving past it again improves, and what a robot
    localizing on the map feels. The verdicts use this one."""
    if survey.joint_cov is None or survey.joint_cov[1] is None:
        return {}
    ids, C = survey.joint_cov
    nm = len(ids)
    idx = {m: i for i, m in enumerate(ids)}
    rows = list(survey.joint_rows)
    if C.shape[0] != 6 * nm + 6 * len(rows):          # a covariance from elsewhere
        rows = ['floor'] if C.shape[0] == 6 * nm + FLOOR_PARAMS else []
    has_floor = 'floor' in rows
    a0 = 6 * nm + 6 * rows.index('align') if 'align' in rows and align is not None else None
    f0 = 6 * nm + 6 * rows.index('floor') if has_floor else None
    origin = frame.origin_marker if frame.mode == 'origin_marker' else None
    y0 = np.concatenate([params_from_T(markers[m]) for m in ids]
                        + [floor_vector(base_poses) if r == 'floor' else np.zeros(ALIGN_PARAMS)
                           for r in rows])

    def frame_of(y):
        if a0 is not None:
            # the trajectory moved by y's alignment part (about its centroid): the frame
            # moves with it, as the fit to the reference track would
            a = y[a0:a0 + ALIGN_PARAMS]
            c = survey.align_centroid
            Ra = rotvec_to_matrix(a[:3])
            return T_tw @ inv_T(make_T(Ra, c + a[3:] - Ra @ c))
        if origin is None:
            return T_tw
        o = T_from_params(y[6 * idx[origin]:6 * idx[origin] + 6])
        if has_floor:
            f = y[f0:f0 + FLOOR_PARAMS]
            n = f[:3] / np.linalg.norm(f[:3])
            p = f[3:] - float(frame.floor_offset) * n
        else:
            p, n = fr.floor_from_base_poses(base_poses, frame.floor_offset)
        return inv_T(fr.origin_marker_frame(o, p, n))

    F0 = frame_of(y0)
    T_hat = {m: F0 @ markers[m] for m in ids}

    def g(y, F, i):
        m = ids[i]
        T = F @ T_from_params(y[6 * i:6 * i + 6])
        return np.concatenate([matrix_to_rotvec(T_hat[m][:3, :3].T @ T[:3, :3]), T[:3, 3]])

    g0 = np.concatenate([g(y0, F0, i) for i in range(nm)])
    G = np.zeros((6 * nm, len(y0)))
    eps = 1e-6
    frame_cols = set()
    if origin is not None:
        frame_cols = set(range(6 * idx[origin], 6 * idx[origin] + 6))
        if has_floor:
            frame_cols |= set(range(f0, f0 + FLOOR_PARAMS))
    if a0 is not None:
        frame_cols |= set(range(a0, a0 + ALIGN_PARAMS))
    for j in range(len(y0)):
        y = y0.copy()
        y[j] += eps
        if j in frame_cols:
            F = frame_of(y)
            G[:, j] = (np.concatenate([g(y, F, i) for i in range(nm)]) - g0) / eps
        elif j < 6 * nm:
            i = j // 6
            G[6 * i:6 * i + 6, j] = (g(y, F0, i) - g0[6 * i:6 * i + 6]) / eps
    # the relative covariance (the verdicts): without the trajectory's placement and
    # without the size's error, which driving past again does not improve; with fewer
    # than three markers, the frame's without them
    Gm = G.copy()
    if a0 is not None:
        Gm[:, a0:a0 + ALIGN_PARAMS] = 0.0
    Cm = Gm @ C @ Gm.T
    Cr = relative_covariance(Cm, [T_hat[m] for m in ids])
    if Cr is None:
        Cr = Cm
    Ct = G @ C @ G.T
    Cx = Ct
    for v in getattr(survey, 'joint_bias', []):
        if len(v) == C.shape[0]:        # a fixed quantity's error (the size): in the frame only
            gv = G @ v
            Ct = Ct + np.outer(gv, gv)
    if align is not None:
        C_align, c = align
        H = np.zeros((6 * nm, 6))
        for i, m in enumerate(ids):
            T = T_hat[m]
            v = T[:3, 3] - c
            H[6 * i:6 * i + 3, :3] = T[:3, :3].T
            H[6 * i + 3:6 * i + 6, :3] = -np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]],
                                                    [-v[1], v[0], 0]])
            H[6 * i + 3:6 * i + 6, 3:] = np.eye(3)
        Ct = Ct + H @ C_align @ H.T
        Cx = Cx + H @ C_align @ H.T
    return {m: tuple(X[6 * i:6 * i + 6, 6 * i:6 * i + 6] for X in (Ct, Cr, Cx))
            for i, m in enumerate(ids)}


def relative_covariance(C, poses):
    """The joint covariance `C` (6 per marker: local rotation vector, translation) with
    a common shift and turn about z of all markers taken out: the S-transformation to
    the minimum trace of the positions. None with fewer than three markers."""
    nm = len(poses)
    if nm < 3:
        return None
    H = np.zeros((6 * nm, 4))
    c = np.mean([T[:3, 3] for T in poses], axis=0)
    z = np.array([0.0, 0.0, 1.0])
    for i, T in enumerate(poses):
        H[6 * i + 3:6 * i + 6, :3] = np.eye(3)
        H[6 * i + 3:6 * i + 6, 3] = np.cross(z, T[:3, 3] - c)
        H[6 * i:6 * i + 3, 3] = T[:3, :3].T @ z
    W = np.zeros(6 * nm)
    for i in range(nm):
        W[6 * i + 3:6 * i + 6] = 1.0
    HtW = H.T * W
    try:
        S = np.eye(6 * nm) - H @ np.linalg.solve(HtW @ H, HtW)
    except np.linalg.LinAlgError:
        return None
    out = S @ C @ S.T
    return 0.5 * (out + out.T)


def _single_keyframe(survey: Survey):
    """The solve()-shaped result for a stationary recording: no bundle adjustment, each
    marker as its one view placed it."""
    kf = survey.keyframes[0]
    markers, stats, sq = {}, {}, []
    for det in kf.detections:
        T_cm, err = marker_pose_in_camera(det, survey.K, survey.s.size_of(det.id))
        markers[det.id] = kf.T_world_cam @ T_cm
        stats[det.id] = {'views': 1, 'reproj_rms_px': err}
        sq.append(err ** 2)
    info = {'keyframes': 1, 'status': 'single keyframe, no bundle adjustment',
            'reproj_rms_px': float(np.sqrt(np.mean(sq))) if sq else 0.0}
    return markers, [kf.T_world_cam], stats, info


# ------------------------------------------------------------------ the recording

def save_dump(path: str, survey: Survey, dictionary: str, frame: fr.FrameSettings) -> None:
    kfs = survey.keyframes
    kf_T = np.stack([k.T_world_cam for k in kfs]) if kfs else np.zeros((0, 4, 4))
    kf_ref = np.full((len(kfs), 4, 4), np.nan)
    for i, k in enumerate(kfs):
        if k.T_ref_base is not None:
            kf_ref[i] = k.T_ref_base
    rows = [(i, d.id, *d.corners.ravel()) for i, k in enumerate(kfs) for d in k.detections]
    s = survey.s
    sizes = np.array(sorted(s.marker_sizes.items()), dtype=float).reshape(-1, 2)
    np.savez_compressed(
        path, K=survey.K,
        T_cam_base=survey.T_cam_base if survey.T_cam_base is not None else np.eye(4),
        has_base=survey.T_cam_base is not None, kf_T=kf_T, kf_ref=kf_ref,
        kf_stamp=np.array([k.stamp for k in kfs]),
        detections=np.array(rows, dtype=float).reshape(-1, 10),
        marker_size=s.marker_size, marker_sizes=sizes, dictionary=dictionary,
        frame_mode=frame.mode, origin_marker=-1 if frame.origin_marker is None else frame.origin_marker,
        frame_id=frame.frame_id, floor_offset=frame.floor_offset,
        motion_factors=s.motion_factors, pose_priors=s.pose_priors, pose_sd=s.pose_sd,
        pose_sd_rot=s.pose_sd_rot, odom_drift=s.odom_drift, marker_size_sd=s.marker_size_sd)


def load_dump(path: str, settings: Optional[SurveySettings] = None):
    """(survey, dictionary, frame settings) from a recording."""
    d = np.load(path, allow_pickle=False)
    if settings is None:
        settings = SurveySettings(marker_size=float(d['marker_size']))
        for k in ('motion_factors', 'pose_priors'):
            if k in d:
                setattr(settings, k, bool(d[k]))
        for k in ('pose_sd', 'pose_sd_rot', 'odom_drift', 'marker_size_sd'):
            if k in d:
                setattr(settings, k, float(d[k]))
    settings.marker_size = float(d['marker_size'])
    if 'marker_sizes' in d:
        settings.marker_sizes = {int(a): float(b) for a, b in d['marker_sizes']}
    sv = Survey(settings, d['K'])
    if bool(d['has_base']):
        sv.T_cam_base = d['T_cam_base']
    dets = d['detections']
    refs = d['kf_ref'] if 'kf_ref' in d else np.full((len(d['kf_T']), 4, 4), np.nan)
    for i, (T, stamp) in enumerate(zip(d['kf_T'], d['kf_stamp'])):
        rows = dets[dets[:, 0] == i]
        ref = None if np.isnan(refs[i]).any() else refs[i]
        sv.keyframes.append(Keyframe(float(stamp), T, [
            Detection(int(r[1]), r[2:].reshape(4, 2)) for r in rows], ref))
    sv.rebuild_live()
    om = int(d['origin_marker']) if 'origin_marker' in d else -1
    frame = fr.FrameSettings(mode=str(d['frame_mode']) if 'frame_mode' in d else 'origin_marker',
                             origin_marker=None if om < 0 else om,
                             frame_id=str(d['frame_id']) if 'frame_id' in d else 'map',
                             floor_offset=float(d['floor_offset']) if 'floor_offset' in d else 0.0)
    return sv, str(d['dictionary']), frame
