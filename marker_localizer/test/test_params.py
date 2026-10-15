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
])
def test_errors_name_the_parameter(kw, msg):
    with pytest.raises(ValueError, match=msg):
        params(**kw)
