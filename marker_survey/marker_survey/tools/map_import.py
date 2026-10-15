"""marker_map_import: convert an existing site's marker list into marker_map.yaml.

    marker_map_import course course.yaml marker_map.yaml --dictionary DICT_5X5_1000
    marker_map_import ceiling-list markers.yaml marker_map.yaml

A one-off migration; core/importers.py describes the two source formats.
Comments next to markers are copied; the source's header and footer
comments only with --keep-header (otherwise one line names the source).
The converted map is loaded back and compared with the source, pose by pose.
"""
import argparse
import sys

import numpy as np

from marker_survey.core import importers
from marker_survey.core import marker_map as mm


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('kind', choices=['course', 'ceiling-list'])
    ap.add_argument('source')
    ap.add_argument('output', help='marker_map.yaml to write')
    ap.add_argument('--dictionary', help='printed family, if the source does not name it')
    ap.add_argument('--frame-id', default='map')
    ap.add_argument('--keep-header', action='store_true',
                    help="also copy the source's header and footer comments")
    args = ap.parse_args(argv)
    with open(args.source) as f:
        text = f.read()
    try:
        if args.kind == 'course':
            m = importers.from_course(text, args.source, args.dictionary, args.frame_id,
                                      args.keep_header)
        else:
            m = importers.from_ceiling_list(text, args.source, args.frame_id, args.keep_header)
    except (mm.MarkerMapError, ValueError) as e:
        print(f'ERROR: {e}')
        return 1
    mm.write(m, args.output)
    back = mm.load(args.output)
    want = importers.source_poses('course' if args.kind == 'course' else 'ceiling', text)
    worst = max(float(np.abs(back.pose_of(i) - T).max()) for i, T in want.items())
    print(f'wrote {args.output}: {len(back.markers)} markers; largest difference from the '
          f'source pose {worst:.1e} (angles rounded by at most '
          f'{importers.ANGLE_ROUNDING_DEG} deg)')
    return 0 if worst < POSE_TOLERANCE else 1


# the rounding of the angles, as a rotation matrix element, and a margin
POSE_TOLERANCE = 2e-4


if __name__ == '__main__':
    sys.exit(main())
