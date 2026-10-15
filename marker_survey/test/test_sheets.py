"""Marker sheets: units, ID choice, layout, the PDF, and printed sizes read back."""
import os
import re
import shutil
import subprocess
import tempfile

import numpy as np
import pytest

from marker_survey.core import sheets
from marker_survey.core.identify import annotate, identify
from marker_survey.core.sheets import SheetError, SheetJob, plan_job


def test_parse_length_needs_a_unit():
    assert sheets.parse_length('450mm') == 450
    assert sheets.parse_length('45 cm') == 450
    assert sheets.parse_length('0.45m') == pytest.approx(450)
    assert sheets.parse_length('1in') == pytest.approx(25.4)
    for bad in ('45', 'abc', '-3cm', '0cm'):
        with pytest.raises(SheetError):
            sheets.parse_length(bad)


def test_parse_paper_and_ids():
    assert sheets.parse_paper('a1') == ('A1', (594, 841))
    assert sheets.parse_paper('exact') == ('exact', None)
    assert sheets.parse_paper('60x85cm') == ('600x850mm', (600, 850))
    with pytest.raises(SheetError):
        sheets.parse_paper('huge')
    assert sheets.parse_ids('0-3, 8,2') == [0, 1, 2, 3, 8]
    with pytest.raises(SheetError):
        sheets.parse_ids('5-2')


def test_size_text():
    assert sheets.size_text(450) == '45cm'
    assert sheets.size_text(375) == '37.5cm'
    assert sheets.size_text(123.4) == '123.4mm'


def test_ids_are_deterministic_distinct_and_far_apart():
    fam = sheets.family('tag36h11')
    a, da = sheets.select_ids(fam, 24)
    b, db = sheets.select_ids(fam, 24)
    assert a == b and da == db
    assert len(set(a)) == 24 and a[0] == 0
    assert da == sheets.min_distance(fam, a)
    assert da >= 11                        # tag36h11's own guarantee, rotations included
    # the first picks are farther apart than the plain 0..23
    assert da >= sheets.min_distance(fam, list(range(24)))


def test_exclusions_are_kept_out_and_kept_apart():
    fam = sheets.family('tag36h11')
    used = [0, 405, 240]
    ids, d = sheets.select_ids(fam, 5, used)
    assert not set(ids) & set(used)
    assert d == sheets.min_distance(fam, ids + used)


def test_too_many_markers():
    with pytest.raises(SheetError, match='too many'):
        plan_job(SheetJob(size_mm=100, count=40, dictionary='tag16h5'))


def test_auto_paper_and_the_error_naming_a_paper_that_fits():
    assert plan_job(SheetJob(size_mm=450, count=2)).paper_name == 'A1'
    assert plan_job(SheetJob(size_mm=120, count=2)).paper_name == 'A4'
    with pytest.raises(SheetError, match='A1 fits'):
        plan_job(SheetJob(size_mm=450, count=2, paper='A2'))


def test_several_small_markers_share_a_page():
    p = plan_job(SheetJob(size_mm=60, count=12, paper='A4'))
    assert p.per_page == 4 and len(p.pages) == 3
    assert plan_job(SheetJob(size_mm=60, count=12, paper='A4', per_page='1')).per_page == 1


def test_exact_page_is_marker_plus_border():
    p = plan_job(SheetJob(size_mm=450, count=1, paper='exact', border_mm=60))
    assert p.pages[0].width == pytest.approx(570) and p.pages[0].height == pytest.approx(570)
    default = plan_job(SheetJob(size_mm=450, count=1, paper='exact'))
    assert default.border_mm == pytest.approx(450 / 8)       # one cell
    assert plan_job(SheetJob(size_mm=450, count=1, paper='exact', border_mm=20)).warnings


def test_pdf_says_the_id_of_every_page():
    p = plan_job(SheetJob(size_mm=450, count=3, back_labels=True))
    pdf = sheets.outputs(p)[0][1]
    assert pdf.startswith(b'%PDF-1.4')
    for mid in p.ids:
        assert f'/P (id {mid})'.encode() in pdf            # page label
        assert f'/Title (id {mid})'.encode() in pdf        # bookmark
        assert f'/P (id {mid} back)'.encode() in pdf
    assert pdf.count(b'/Type /Page ') == len(p.pages) + len(p.contact)
    assert sheets.outputs(plan_job(SheetJob(size_mm=450, count=3, back_labels=True)))[0][1] == pdf


def test_split_files_are_named_after_their_id():
    p = plan_job(SheetJob(size_mm=450, count=3, split=True))
    names = [n for n, _ in sheets.outputs(p)]
    for mid in p.ids:
        assert f'tag36h11_45cm_id{mid:03d}.pdf' in names
    assert 'contact_sheet.pdf' in names and 'markers.csv' in names
    rows = sheets.manifest_csv(p).strip().splitlines()
    assert rows[0].startswith('id,dictionary') and len(rows) == 4


def test_svg_preview_draws_the_same_cells():
    p = plan_job(SheetJob(size_mm=120, count=1))
    svg = sheets.page_svg(p.pages[0])
    black = len(re.findall(r'<rect[^>]*fill="#000000"', svg))
    assert black == sum(1 for prim in p.pages[0].prims if prim.kind == 'rect' and prim.args[4] == 0.0)


def _rendered(job, dpi):
    """Every marker page of a job rasterized: [(page ids, grey image)]."""
    p = plan_job(job)
    tmp = tempfile.mkdtemp()
    try:
        path = os.path.join(tmp, 'm.pdf')
        with open(path, 'wb') as f:
            f.write(sheets.pdf_bytes(p.pages, 't'))
        subprocess.run(['pdftoppm', '-r', str(dpi), '-gray', '-png', path, os.path.join(tmp, 'p')],
                       check=True)
        import cv2
        files = sorted(n for n in os.listdir(tmp) if n.endswith('.png'))
        return p, [(page.ids, cv2.imread(os.path.join(tmp, n), cv2.IMREAD_GRAYSCALE))
                   for page, n in zip(p.pages, files)]
    finally:
        shutil.rmtree(tmp)


@pytest.mark.skipif(shutil.which('pdftoppm') is None, reason='needs pdftoppm (poppler-utils)')
@pytest.mark.parametrize('job, dpi', [
    (SheetJob(size_mm=450, count=3), 25),                       # A1, one per page
    (SheetJob(size_mm=60, count=8, paper='A4'), 150),           # four per page
    (SheetJob(size_mm=450, count=2, paper='exact'), 25),
    (SheetJob(size_mm=100, count=4, dictionary='DICT_5X5_1000', paper='A4'), 120),
])
def test_printed_pages_read_back_with_their_ids_and_size(job, dpi):
    plan, pages = _rendered(job, dpi)
    px_per_mm = dpi / 25.4
    for ids, img in pages:
        found = identify(img, plan.family.name)
        assert sorted(f.id for f in found) == sorted(ids)
        for f in found:
            edge = np.mean([np.linalg.norm(f.corners[i] - f.corners[(i + 1) % 4]) for i in range(4)])
            # the black square's edge, within one pixel
            assert abs(edge / px_per_mm - job.size_mm) * px_per_mm < 1.5


def test_identify_any_family_and_rotation():
    import cv2
    fam = sheets.family('tag36h11')
    cells = sheets.marker_cells(fam, 17)
    img = np.full((400, 400), 255, np.uint8)
    marker = np.kron(1 - cells, np.ones((25, 25), np.uint8)) * 255
    img[100:300, 100:300] = marker
    rot = cv2.warpAffine(img, cv2.getRotationMatrix2D((200, 200), 30, 1.0), (400, 400),
                         borderValue=255)
    found = identify(rot)
    assert [(f.id, f.dictionary) for f in found] == [(17, 'tag36h11')]
    out = annotate(rot, found)
    assert out.shape == (400, 400, 3)
