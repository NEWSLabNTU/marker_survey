"""The survey node's parameters, read and checked. No ROS: the checks are unit-tested
without a running node."""

import math

from marker_survey.core.frames import FRAME_MODES, FrameSettings
from marker_survey.core.survey import SurveySettings

POSE_SOURCES = {'odometry': 'odometry', 'odom': 'odometry', 'tf': 'tf'}


def parse_sizes(items) -> dict:
    """['12:0.30', '13:0.30'] -> {12: 0.30, 13: 0.30}."""
    out = {}
    for it in items or []:
        if not str(it).strip():
            continue
        try:
            k, v = str(it).split(':')
            out[int(k)] = float(v)
        except ValueError:
            raise ValueError(f'marker_sizes entry {it!r}: expected "ID:SIZE", e.g. "12:0.30"') from None
    return out


class SurveyParams:
    """The node's parameters, read and checked; kept separate from the node so the
    checks are testable."""

    def __init__(self, get, check_frame: bool = True):
        self.output_dir = str(get('output_dir'))
        if not self.output_dir:
            raise ValueError('set output_dir: where marker_map.yaml is written')
        self.dictionary = str(get('dictionary'))
        self.frame = FrameSettings(mode=str(get('frame.mode')),
                                   origin_marker=int(get('frame.origin_marker')),
                                   frame_id=str(get('frame.frame_id')),
                                   floor_offset=float(get('floor_offset')))
        if self.frame.mode not in FRAME_MODES:
            raise ValueError(f'frame.mode {self.frame.mode!r} is not one of {", ".join(FRAME_MODES)}')
        if self.frame.mode != 'origin_marker':
            self.frame.origin_marker = None
        elif self.frame.origin_marker is not None and self.frame.origin_marker < 0:
            self.frame.origin_marker = None
        if check_frame:
            self.frame.check()
        src = str(get('pose_source'))
        if src not in POSE_SOURCES:
            raise ValueError(f'pose_source {src!r} is not one of odometry, tf')
        self.pose_source = POSE_SOURCES[src]
        self.pose_frame = str(get('pose_frame'))
        if self.pose_source == 'tf' and not self.pose_frame:
            raise ValueError('pose_source tf needs pose_frame: the frame of the camera poses')
        self.reference_frame = str(get('frame.reference_frame'))
        if self.frame.mode == 'align_to_track' and not self.reference_frame:
            raise ValueError('frame.mode align_to_track needs frame.reference_frame: the TF frame '
                             'of the reference trajectory (for example map)')
        self.base_frame = str(get('base_frame'))
        size = float(get('marker_size'))
        if size <= 0:
            raise ValueError('marker_size must be positive: the edge of the black square [m]')
        self.settings = SurveySettings(
            marker_size=size, marker_sizes=parse_sizes(get('marker_sizes')),
            keyframe_distance=float(get('keyframe_distance')),
            keyframe_angle=math.radians(float(get('keyframe_angle_deg'))),
            odom_drift=float(get('odom_drift')), min_marker_px=float(get('min_marker_px')),
            pose_sd=float(get('pose_sd')), pose_sd_rot=math.radians(float(get('pose_sd_deg'))))
        if self.frame.mode == 'pose_frame':
            # a map pose jitters frame to frame: absolute priors, not its relative motion
            self.settings.motion_factors = bool(get('pose_frame_motion'))
            self.settings.pose_priors = True
        self.min_marker_px = self.settings.min_marker_px
        self.border_margin = float(get('border_margin_px'))
        self.result_frame_id = self.frame.frame_id
        if self.frame.mode == 'pose_frame' and self.pose_source == 'tf':
            self.result_frame_id = self.pose_frame
        elif self.frame.mode == 'align_to_track':
            self.result_frame_id = self.reference_frame
        self.frame.frame_id = self.result_frame_id


PARAMETERS = {
    'output_dir': '',
    'dictionary': 'DICT_5X5_1000',
    'marker_size': 0.38,
    'marker_sizes': [''],
    'image_topic': '/camera/image',
    'camera_info_topic': '/camera/camera_info',
    'compressed': False,
    'pose_source': 'odometry',
    'odom_topic': '/odom',
    'pose_frame': '',
    'base_frame': 'base_link',
    'frame.mode': 'origin_marker',
    'frame.origin_marker': -1,
    'frame.frame_id': 'map',
    'frame.reference_frame': '',
    'floor_offset': 0.0,
    'rate': 5.0,
    'process_all': False,
    'pose_tolerance': 0.1,
    'save_on_exit': True,
    'keyframe_distance': 0.15,
    'keyframe_angle_deg': 5.0,
    'odom_drift': 0.01,
    'pose_sd': 0.05,
    'pose_sd_deg': 1.0,
    'pose_frame_motion': False,
    'min_marker_px': 20.0,
    'border_margin_px': 5.0,
    'debug_width': 960,
    'start_paused': False,
    'keep_history': True,
}
