"""marker_survey_solve: solve a saved survey recording again, without the robot.

    marker_survey_solve OUT/survey_data.npz [--output-dir DIR] [--frame-mode MODE]
                        [--origin-marker ID] [--floor-offset M] [--odom-drift X]

Without --output-dir it prints the result only. The frame settings default
to the ones the recording was made with.
"""
import argparse

from marker_survey.core.frames import FRAME_MODES
from marker_survey.core.solve import finish, load_dump


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('dump', help='survey_data.npz written by the survey node')
    ap.add_argument('--output-dir', help='where to write marker_map.yaml and survey_report.yaml')
    ap.add_argument('--frame-mode', choices=FRAME_MODES)
    ap.add_argument('--origin-marker', type=int, help='the marker that defines the frame')
    ap.add_argument('--frame-id', help='frame name written to the map')
    ap.add_argument('--floor-offset', type=float, help='metres the floor lies below the base frame')
    ap.add_argument('--odom-drift', type=float, help='odometry drift, sd per metre')
    ap.add_argument('--max-nfev', type=int, default=300, help='solver iteration cap')
    args = ap.parse_args(argv)
    sv, dictionary, frame = load_dump(args.dump)
    if args.odom_drift is not None:
        sv.s.odom_drift = args.odom_drift
    if args.frame_mode:
        frame.mode = args.frame_mode
        sv.s.pose_priors = frame.mode == 'pose_frame'
        sv.s.motion_factors = frame.mode != 'pose_frame'
    if args.origin_marker is not None:
        frame.origin_marker = args.origin_marker
    if args.frame_id:
        frame.frame_id = args.frame_id
    if args.floor_offset is not None:
        frame.floor_offset = args.floor_offset
    try:
        finish(sv, frame, dictionary, args.output_dir, max_nfev=args.max_nfev)
    except ValueError as e:
        print(f'not solved: {e}')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
