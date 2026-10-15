import pytest

from marker_localizer.params import PARAMETERS, LocalizerParams


def params(**kw):
    p = dict(PARAMETERS, marker_map='site/marker_map.yaml')
    p.update(kw)
    return LocalizerParams(lambda k: p[k])


def test_defaults_are_valid():
    p = params()
    assert p.mode == 'planar' and p.settings.gate_distance == 1.0 and p.enabled


@pytest.mark.parametrize('kw, msg', [
    ({'marker_map': ''}, 'marker_map'),
    ({'mode': '6dof'}, 'mode'),
    ({'roll_pitch': 'gyro'}, 'roll_pitch'),
    ({'roll_pitch': 'tf'}, 'roll_pitch_frame'),
    ({'prior_type': 'twist'}, 'prior_type'),
    ({'gate_distance': 0.0}, 'gate_distance'),
    ({'gate_sigmas': -1.0}, 'gate_sigmas'),
])
def test_errors_name_the_parameter(kw, msg):
    with pytest.raises(ValueError, match=msg):
        params(**kw)


def test_the_launch_file_passes_its_arguments_with_the_node_defaults():
    """Every argument of localizer.launch.xml sets the parameter of its name, with the
    node's default; param_file is read last."""
    import os
    import xml.etree.ElementTree as ET
    path = os.path.join(os.path.dirname(__file__), '..', 'launch', 'localizer.launch.xml')
    root = ET.parse(path).getroot()
    args = {a.get('name'): a.get('default') for a in root.iter('arg')}
    node = root.find('node')
    params = [(p.get('name'), p.get('value'), p.get('from')) for p in node.iter('param')]
    assert params[-1] == (None, None, '$(var param_file)')
    for name, value, _ in params[:-1]:
        assert value == f'$(var {name})', name
        assert name in PARAMETERS or name == 'use_sim_time', name
        if name in PARAMETERS and args.get(name) is not None:
            default = PARAMETERS[name]
            want = str(default).lower() if isinstance(default, bool) else str(default)
            assert args[name] == want, (name, args[name], default)
    for name in ('max_distance', 'min_sd', 'min_sd_deg', 'max_position_sd', 'require_prior',
                 'prior_tolerance', 'param_file', 'gate_sigmas', 'node_name'):
        assert name in args, name
