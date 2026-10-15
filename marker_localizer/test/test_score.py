"""marker_localizer_score's gap check: no fix while the camera looks away, the first
fix after it in the right place."""
from marker_localizer.tools.score import gap_check


def rows(stamps, err=0.01):
    return [{'stamp': f'{t:.9f}', 'err': err} for t in stamps]


def test_no_fix_in_the_gap_and_a_good_first_fix_after_it():
    fail = []
    out = gap_check(rows([9.8, 10.0, 13.4, 13.6]), 10.1, 13.3, 5.0, fail)
    assert not fail
    assert out[0].endswith('0 fixes in it')
    assert out[1].startswith('first fix after the gap: 0.10 s after it, error 1.00 cm')


def test_a_fix_in_the_gap_or_a_bad_first_one_fails():
    fail = []
    gap_check(rows([10.0, 11.0, 13.4]), 10.1, 13.3, None, fail)
    assert fail and 'inside the gap' in fail[0]
    fail = []
    gap_check(rows([10.0, 13.4], err=0.2), 10.1, 13.3, 5.0, fail)
    assert fail and 'first fix after the gap 20.00 cm off' in fail[0]
    fail = []
    gap_check(rows([10.0]), 10.1, 13.3, 5.0, fail)
    assert fail == ['no fix after the gap']
