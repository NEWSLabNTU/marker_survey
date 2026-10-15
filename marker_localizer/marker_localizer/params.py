"""The localizer node's parameters, read and checked. No ROS: the checks are unit-tested
without a running node."""

from marker_localizer.core.localize import MODES, LocalizerSettings

ROLL_PITCH = ('zero', 'tf', 'imu')
PRIOR_TYPES = ('pose', 'odometry')


class LocalizerParams:
    def __init__(self, get):
        self.marker_map = str(get('marker_map'))
        if not self.marker_map:
            raise ValueError('set marker_map: the site\'s marker_map.yaml')
        self.mode = str(get('mode'))
        if self.mode not in MODES:
            raise ValueError(f'mode {self.mode!r} is not one of {", ".join(MODES)}')
        self.base_frame = str(get('base_frame'))
        self.image_topic = str(get('image_topic'))
        self.camera_info_topic = str(get('camera_info_topic'))
        self.compressed = bool(get('compressed'))
        self.pose_topic = str(get('pose_topic'))
        self.prior_topic = str(get('prior_topic'))
        self.prior_type = str(get('prior_type'))
        if self.prior_type not in PRIOR_TYPES:
            raise ValueError(f'prior_type {self.prior_type!r} is not one of {", ".join(PRIOR_TYPES)}')
        self.prior_tolerance = float(get('prior_tolerance'))
        self.require_prior = bool(get('require_prior'))
        self.enabled = bool(get('enabled'))
        self.process_all = bool(get('process_all'))
        self.roll_pitch = str(get('roll_pitch'))
        if self.roll_pitch not in ROLL_PITCH:
            raise ValueError(f'roll_pitch {self.roll_pitch!r} is not one of {", ".join(ROLL_PITCH)}')
        self.roll_pitch_frame = str(get('roll_pitch_frame'))
        if self.roll_pitch == 'tf' and not self.roll_pitch_frame:
            raise ValueError('roll_pitch tf needs roll_pitch_frame: a level frame such as odom')
        self.imu_topic = str(get('imu_topic'))
        self.min_marker_px = float(get('min_marker_px'))
        self.border_margin = float(get('border_margin_px'))
        self.detect_scale = float(get('detect_scale'))
        self.quad_decimate = float(get('quad_decimate'))
        self.min_contrast = int(get('min_contrast'))
        self.debug_width = int(get('debug_width'))
        self.settings = LocalizerSettings(
            mode=self.mode, pixel_sd=float(get('pixel_sd')), robust_px=float(get('robust_px')),
            max_rms_px=float(get('max_rms_px')), outlier_px=float(get('outlier_px')),
            max_distance=float(get('max_distance')),
            max_position_sd=float(get('max_position_sd')), min_sd=float(get('min_sd')),
            min_sd_deg=float(get('min_sd_deg')), tilt_sd_deg=float(get('tilt_sd_deg')),
            base_height=float(get('base_height')), gate_distance=float(get('gate_distance')),
            gate_angle_deg=float(get('gate_angle_deg')))
        self.settings.check()


PARAMETERS = {
    'marker_map': '',
    'mode': 'planar',
    'base_frame': 'base_link',
    'image_topic': '/camera/image',
    'camera_info_topic': '/camera/camera_info',
    'compressed': False,
    'pose_topic': '~/pose',
    'prior_topic': '',
    'prior_type': 'pose',
    'prior_tolerance': 0.5,
    'require_prior': False,
    'gate_distance': 1.0,
    'gate_angle_deg': 15.0,
    'enabled': True,
    'process_all': False,
    'roll_pitch': 'zero',
    'roll_pitch_frame': '',
    'imu_topic': '/imu',
    'min_marker_px': 20.0,
    'border_margin_px': 5.0,
    'detect_scale': 1.0,
    'quad_decimate': -1.0,
    'min_contrast': 30,
    'max_distance': 0.0,
    'max_position_sd': 0.25,
    'pixel_sd': 0.5,
    'robust_px': 1.0,
    'max_rms_px': 2.0,
    'outlier_px': 3.0,
    'min_sd': 0.01,
    'min_sd_deg': 0.3,
    'tilt_sd_deg': 0.2,
    'base_height': 0.0,
    'debug_width': 960,
}
