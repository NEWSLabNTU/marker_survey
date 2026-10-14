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
import os
from typing import Optional

import numpy as np

from marker_survey.core import frames as fr
from marker_survey.core import marker_map as mm
from marker_survey.core import report as rp
from marker_survey.core.detection import Detection, marker_pose_in_camera
from marker_survey.core.se3 import T_from_params, inv_T, matrix_to_rotvec, params_from_T
from marker_survey.core.survey import Keyframe, Survey, SurveySettings

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
        markers, poses, stats, info = survey.solve(max_nfev=max_nfev)
        log(f'solved: reprojection RMS {info["reproj_rms_px"]:.2f} px ({info["status"]})')
    base = [T @ survey.T_cam_base for T in poses]
    floor_p, floor_n = fr.floor_from_base_poses(base, frame.floor_offset)
    frame_info = {'mode': frame.mode, 'frame_id': frame.frame_id}
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
        frame_info['definition'] = ('survey in the odometry frame, fitted rigidly to the '
                                    'reference trajectory')
        frame_info['fit_points'] = int(len(res))
        frame_info['fit_rms_m'] = round(float(np.sqrt(np.mean(res ** 2))), 4)
        frame_info['fit_max_m'] = round(float(res.max()), 4)
        log(f'aligned to the reference track: {len(res)} keyframes, residual RMS '
            f'{frame_info["fit_rms_m"] * 100:.1f} cm, max {frame_info["fit_max_m"] * 100:.1f} cm')
    if frame.floor_offset:
        frame_info['floor_offset'] = float(frame.floor_offset)
    sizes = {mid: survey.s.size_of(mid) for mid in markers}
    for mid, C in target_covariances(survey, markers, T_tw, frame, floor_p, floor_n).items():
        stats[mid]['cov_target'] = C
    surveyed = rp.surveyed_markers(markers, stats, T_tw, sizes, floor_p, floor_n, report_settings)
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


def target_covariances(survey: Survey, markers, T_tw, frame: fr.FrameSettings, floor_p, floor_n):
    """6x6 covariance (rotation, translation) of every marker in the result frame. In
    origin_marker mode the frame moves with the origin marker, so the uncertainty is
    relative to it (the floor, from many poses, is taken as exact)."""
    if survey.joint_cov is None or survey.joint_cov[1] is None:
        return {}
    ids, C = survey.joint_cov
    idx = {m: i for i, m in enumerate(ids)}
    origin = frame.origin_marker if frame.mode == 'origin_marker' else None

    def frame_of(xo):
        if origin is None:
            return T_tw
        return inv_T(fr.origin_marker_frame(T_from_params(xo), floor_p, floor_n))

    out = {}
    eps = 1e-6
    for mid in ids:
        xm = params_from_T(markers[mid])
        xo = params_from_T(markers[origin]) if origin is not None else None
        T_hat = frame_of(xo) @ markers[mid]

        def f(x):
            m = T_from_params(x[:6])
            o = x[6:] if len(x) > 6 else (x[:6] if mid == origin else xo)
            T = frame_of(o) @ m
            return np.concatenate([matrix_to_rotvec(T_hat[:3, :3].T @ T[:3, :3]), T[:3, 3]])

        blocks = [idx[mid]] + ([idx[origin]] if origin is not None and mid != origin else [])
        x0 = np.concatenate([xm] + ([xo] if len(blocks) > 1 else []))
        f0 = f(x0)
        J = np.zeros((6, len(x0)))
        for j in range(len(x0)):
            dx = np.zeros(len(x0))
            dx[j] = eps
            J[:, j] = (f(x0 + dx) - f0) / eps
        sel = np.concatenate([np.arange(6 * b, 6 * b + 6) for b in blocks])
        out[mid] = J @ C[np.ix_(sel, sel)] @ J.T
    return out


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
        pose_sd_rot=s.pose_sd_rot, odom_drift=s.odom_drift)


def load_dump(path: str, settings: Optional[SurveySettings] = None):
    """(survey, dictionary, frame settings) from a recording."""
    d = np.load(path, allow_pickle=False)
    if settings is None:
        settings = SurveySettings(marker_size=float(d['marker_size']))
        for k in ('motion_factors', 'pose_priors'):
            if k in d:
                setattr(settings, k, bool(d[k]))
        for k in ('pose_sd', 'pose_sd_rot', 'odom_drift'):
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
