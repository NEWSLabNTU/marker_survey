import math

import pytest

from marker_survey.params import PARAMETERS, SurveyParams, parse_sizes


def params(**over):
    p = dict(PARAMETERS)
    p.update({'output_dir': '/tmp/site', 'frame.origin_marker': 10})
    p.update(over)
    return SurveyParams(lambda k: p[k])


def test_defaults_survey_in_the_frame_of_an_origin_marker():
    p = params()
    assert p.frame.mode == 'origin_marker' and p.frame.origin_marker == 10
    assert p.pose_source == 'odometry'
    assert p.settings.motion_factors and not p.settings.pose_priors
    assert p.settings.keyframe_angle == pytest.approx(math.radians(5.0))
    assert p.frame.frame_id == 'map'


def test_output_dir_and_origin_are_required():
    with pytest.raises(ValueError, match='output_dir'):
        params(output_dir='')
    with pytest.raises(ValueError, match='origin_marker'):
        params(**{'frame.origin_marker': -1})


def test_pose_frame_uses_priors_and_the_pose_frame_name():
    p = params(**{'frame.mode': 'pose_frame', 'pose_source': 'tf', 'pose_frame': 'map',
                  'pose_sd': 0.06})
    assert p.settings.pose_priors and not p.settings.motion_factors
    assert p.settings.pose_sd == 0.06
    assert p.frame.frame_id == 'map' and p.frame.origin_marker is None


def test_align_to_track_needs_a_reference_frame():
    with pytest.raises(ValueError, match='reference_frame'):
        params(**{'frame.mode': 'align_to_track'})
    p = params(**{'frame.mode': 'align_to_track', 'frame.reference_frame': 'map'})
    assert p.frame.frame_id == 'map' and p.settings.motion_factors


def test_bad_values_are_refused():
    with pytest.raises(ValueError, match='frame.mode'):
        params(**{'frame.mode': 'nowhere'})
    with pytest.raises(ValueError, match='pose_source'):
        params(pose_source='gps')
    with pytest.raises(ValueError, match='pose_frame'):
        params(pose_source='tf')
    with pytest.raises(ValueError, match='marker_size'):
        params(marker_size=0.0)
    with pytest.raises(ValueError, match='ID:SIZE'):
        params(marker_sizes=['12=0.3'])


def test_odom_is_an_alias_of_odometry():
    assert params(pose_source='odom').pose_source == 'odometry'


def test_marker_sizes():
    assert parse_sizes(['12:0.30', ' ', '7:0.2']) == {12: 0.30, 7: 0.2}
    assert params(marker_sizes=['12:0.30']).settings.size_of(12) == 0.30


def test_the_launch_file_passes_its_arguments_with_the_node_defaults():
    """Every parameter survey.launch.xml sets comes from an argument with the node's
    default (or a required one); param_file is read last."""
    import os
    import xml.etree.ElementTree as ET
    path = os.path.join(os.path.dirname(__file__), '..', 'launch', 'survey.launch.xml')
    root = ET.parse(path).getroot()
    args = {a.get('name'): a.get('default') for a in root.iter('arg')}
    params = [(p.get('name'), p.get('value'), p.get('from')) for p in root.find('node').iter('param')]
    assert params[-1] == (None, None, '$(var param_file)') and 'param_file' in args
    for name, value, _ in params[:-1]:
        assert value.startswith('$(var ') and value.endswith(')'), name
        arg = value[6:-1]
        assert arg in args, name
        if name not in PARAMETERS:
            assert name == 'use_sim_time'
            continue
        default = PARAMETERS[name]
        if args[arg] is not None and default not in ('', None):
            want = str(default).lower() if isinstance(default, bool) else str(default)
            assert args[arg] == want or float(args[arg]) == float(default), (name, args[arg], default)


def test_pose_source_markers_links_keyframes_by_the_markers_only():
    p = params(pose_source='markers', base_frame='', vertical='ceiling', floor_offset=1.0)
    assert p.pose_source == 'markers'
    assert not p.settings.motion_factors and not p.settings.pose_priors
    assert p.frame.vertical == 'ceiling' and p.frame.floor_offset == 1.0
    q = params(pose_source='markers', **{'frame.mode': 'pose_frame'})
    assert not q.settings.pose_priors and q.frame.frame_id == 'map'


def test_vertical_must_be_known():
    with pytest.raises(ValueError, match='vertical'):
        params(vertical='wall')
