"""marker_survey_compare: the error against the survey's own 1-sigma, and --require-good."""
import yaml

from marker_survey.tools.compare import main

TRUTH = """version: 1
frame_id: map
dictionary: tag36h11
marker_size: 0.40
mount: ceiling
markers:
  - {id: 0, x: 0.0, y: 0.0, z: 2.9, rotation_deg: 0.0}
  - {id: 1, x: 1.4, y: 0.0, z: 2.9, rotation_deg: 0.0}
"""


def site(tmp_path, verdict='good'):
    (tmp_path / 'truth.yaml').write_text(TRUTH)
    moved = TRUTH.replace('x: 1.4, y: 0.0, z: 2.9', 'x: 1.43, y: 0.04, z: 2.92')
    (tmp_path / 'marker_map.yaml').write_text(moved)
    rep = {'markers': [{'id': 0, 'verdict': 'good', 'frame_sd_xy_cm': 0.5, 'frame_sd_z_cm': 1.0},
                       {'id': 1, 'verdict': verdict, 'frame_sd_xy_cm': 2.5, 'frame_sd_z_cm': 1.0}]}
    (tmp_path / 'survey_report.yaml').write_text(yaml.safe_dump(rep))
    return str(tmp_path / 'marker_map.yaml'), str(tmp_path / 'truth.yaml')


def test_error_in_units_of_the_frame_sigma(tmp_path, capsys):
    got, want = site(tmp_path)
    assert main([got, want, '--require-good']) == 0
    out = capsys.readouterr().out
    assert 'h/sd_xy  dz/sd_z  verdict' in out
    assert '    2.00     2.00  good' in out                   # 5 cm / 2.5 cm, 2 cm / 1 cm
    assert 'error / frame 1-sigma: horizontal median 1.00, max 2.00; height median 1.00, max 2.00' in out


def test_require_good(tmp_path, capsys):
    got, want = site(tmp_path, verdict='pass again')
    assert main([got, want, '--require-good']) == 1
    assert 'FAIL: not good: [1]' in capsys.readouterr().out
    (tmp_path / 'survey_report.yaml').unlink()
    assert main([got, want]) == 0                              # no report: no column, no check
    assert main([got, want, '--require-good']) == 1
