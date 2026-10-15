"""marker_localizer_score: the localizer's published poses against the truth.

Reads a rosbag2 recording of the localizer's output topic
(geometry_msgs/PoseWithCovarianceStamped) and the truth_poses.csv that
marker_survey_synthetic writes beside its recording, and prints the error of
every fix. Every fix must carry exactly the stamp of the image it came from
(the capture time); a stamp that matches no image fails the check.

    marker_localizer_score OUT/poses OUT/truth_poses.csv --mode planar --max-mean-cm 1 --max-cm 3

--gap T0 T1 (stamps, s): no fix may carry a stamp between them (the camera
looked away from the markers: a fix there is a false one), and the first fix
after T1 is reported with its error and its delay; --max-first-cm limits that
error (the localizer, with no prior, must land at once where the camera is).
"""

import argparse
import math
import sys

import numpy as np

from marker_survey.core.se3 import make_T, quat_to_matrix, rotation_angle


def load_truth(path):
    out = {}
    with open(path) as fh:
        for line in fh:
            if line.startswith('#') or not line.strip():
                continue
            parts = line.strip().split(',')
            v = [float(x) for x in parts[1:]]
            out[parts[0]] = make_T(quat_to_matrix(*v[3:7]), v[0:3])
    return out


def read_poses(bag, topic=None):
    import rclpy.serialization as ser
    import rosbag2_py
    from geometry_msgs.msg import PoseWithCovarianceStamped
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = []
    while r.has_next():
        name, data, _ = r.read_next()
        if types.get(name) != 'geometry_msgs/msg/PoseWithCovarianceStamped' or \
                (topic and name != topic):
            continue
        m = ser.deserialize_message(data, PoseWithCovarianceStamped)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        out.append((f'{m.header.stamp.sec}.{m.header.stamp.nanosec:09d}', m.header.frame_id,
                    make_T(quat_to_matrix(q.x, q.y, q.z, q.w), [p.x, p.y, p.z]),
                    np.array(m.pose.covariance).reshape(6, 6)))
    return out


def score(poses, truth, mode):
    rows, unmatched = [], []
    for stamp, frame, T, C in poses:
        Tt = truth.get(stamp)
        if Tt is None:
            unmatched.append(stamp)
            continue
        d = T[:3, 3] - Tt[:3, 3]
        e = float(np.hypot(d[0], d[1])) if mode == 'planar' else float(np.linalg.norm(d))
        yaw = math.degrees(abs((math.atan2(T[1, 0], T[0, 0]) - math.atan2(Tt[1, 0], Tt[0, 0])
                                + math.pi) % (2 * math.pi) - math.pi))
        rot = math.degrees(rotation_angle(T[:3, :3], Tt[:3, :3]))
        nees = float(d[:2] @ np.linalg.solve(C[:2, :2], d[:2]))
        rows.append({'stamp': stamp, 'err': e, 'yaw_deg': yaw, 'rot_deg': rot, 'nees_xy': nees,
                     'frame': frame})
    return rows, unmatched


def gap_check(rows, t0, t1, max_first_cm=None, fail=None):
    """Lines describing the fixes around a gap [t0, t1] (stamps, s) in which none may
    be published; failures are appended to `fail`."""
    fail = [] if fail is None else fail
    out = []
    inside = [r for r in rows if t0 <= float(r['stamp']) <= t1]
    out.append(f'gap {t0:.3f} to {t1:.3f} s ({t1 - t0:.1f} s): {len(inside)} fixes in it')
    if inside:
        fail.append(f'{len(inside)} fixes inside the gap (first {inside[0]["stamp"]})')
    after = sorted((r for r in rows if float(r['stamp']) > t1), key=lambda r: float(r['stamp']))
    if not after:
        fail.append('no fix after the gap')
        return out
    r = after[0]
    last = max((float(x['stamp']) for x in rows if float(x['stamp']) < t0), default=None)
    out.append(f'first fix after the gap: {float(r["stamp"]) - t1:.2f} s after it, error '
               f'{100 * r["err"]:.2f} cm'
               + (f'; the last before it {t0 - last:.2f} s before it' if last is not None else ''))
    if max_first_cm is not None and 100 * r['err'] > max_first_cm:
        fail.append(f'first fix after the gap {100 * r["err"]:.2f} cm off, over {max_first_cm}')
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('bag', help='rosbag2 recording of the output topic')
    ap.add_argument('truth', help='truth_poses.csv')
    ap.add_argument('--topic', help='the output topic, if the bag has several pose topics')
    ap.add_argument('--mode', choices=('planar', '3d'), default='planar',
                    help='planar: error in x, y; 3d: in x, y, z')
    ap.add_argument('--frame-id', default='map')
    ap.add_argument('--min-fixes', type=int, default=1)
    ap.add_argument('--max-mean-cm', type=float)
    ap.add_argument('--max-cm', type=float)
    ap.add_argument('--max-yaw-deg', type=float)
    ap.add_argument('--gap', type=float, nargs=2, metavar=('T0', 'T1'),
                    help='[s] stamps between which no fix may be published')
    ap.add_argument('--max-first-cm', type=float,
                    help='with --gap: the largest error of the first fix after it')
    a = ap.parse_args(argv)
    truth = load_truth(a.truth)
    poses = read_poses(a.bag, a.topic)
    rows, unmatched = score(poses, truth, a.mode)
    fail = []
    if unmatched:
        fail.append(f'{len(unmatched)} fixes carry a stamp that is no image\'s capture time '
                    f'(first {unmatched[0]})')
    frames = {r['frame'] for r in rows}
    if frames - {a.frame_id}:
        fail.append(f'frame_id {sorted(frames)} is not {a.frame_id}')
    if len(rows) < a.min_fixes:
        fail.append(f'{len(rows)} fixes, expected at least {a.min_fixes}')
    if rows:
        e = np.array([r['err'] for r in rows]) * 100
        yaw = max(r['yaw_deg'] for r in rows)
        nees = np.mean([r['nees_xy'] < 9.21 for r in rows])
        print(f'{len(rows)} fixes of {len(truth)} images, every stamp an image\'s capture time: '
              f'{"yes" if not unmatched else "NO"}')
        print(f'{a.mode} error: mean {e.mean():.2f} cm, median {np.median(e):.2f} cm, '
              f'p95 {np.percentile(e, 95):.2f} cm, max {e.max():.2f} cm; yaw max {yaw:.2f} deg; '
              f'within the 99 % ellipse of its covariance: {100 * nees:.1f} %')
        if a.max_mean_cm is not None and e.mean() > a.max_mean_cm:
            fail.append(f'mean {e.mean():.2f} cm over {a.max_mean_cm}')
        if a.max_cm is not None and e.max() > a.max_cm:
            fail.append(f'max {e.max():.2f} cm over {a.max_cm}')
        if a.max_yaw_deg is not None and yaw > a.max_yaw_deg:
            fail.append(f'yaw {yaw:.2f} deg over {a.max_yaw_deg}')
    if a.gap:
        for line in gap_check(rows, a.gap[0], a.gap[1], a.max_first_cm, fail):
            print(line)
    for f in fail:
        print(f'FAIL: {f}')
    return 1 if fail else 0


if __name__ == '__main__':
    sys.exit(main())
