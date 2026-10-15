"""Print-ready marker sheets: choose the IDs, lay out the pages, write PDF and SVG. No ROS.

A job says how big the black square is and how many distinct markers are
needed; everything else has a default:

- dictionary: tag36h11;
- IDs: chosen deterministically (below) unless given;
- paper: the smallest of A4, A3, A2, A1, A0 that holds one marker with its
  labels; or a named size (A0 to A6, B1 to B5, Letter, Legal, Tabloid), a
  custom `WxH` with a unit, or `exact`: a page exactly the marker plus its
  white border;
- white border: one cell of the marker, the least a detector needs.

ID selection. Starting from the lowest ID that is not excluded, each next ID
is the one whose bit pattern differs most from every ID already chosen, and
from every excluded one (IDs already on the site), in any of the four
rotations (largest minimum Hamming distance); ties go to the lower ID. The
same dictionary, count and exclusions always give the same IDs. Markers that
differ in many bits are the least likely to be read as one another.

Every page says which marker it holds, in as many places as the mode
allows: the PDF bookmark and page label ("id 17"), the file name in split
mode, the ID on the sheet (large, outside the white border; in exact mode
small and grey in the border's bottom-right corner, which also marks the
bottom of the marker), an optional back page with the ID in large print,
and a contact sheet and a CSV manifest listing every ID. Once printed, the
pattern itself is the ID: core/identify.py reads it from a photo.

Pages are lists of drawing primitives in millimetres (origin top left, y
down). pdf_bytes() and page_svg() draw the same primitives, so the web
preview is the PDF.
"""

import csv
import io
import math
import re
import zipfile
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set, Tuple

import numpy as np

from marker_survey.core.detection import opencv_dictionary, opencv_dictionary_name

DEFAULT_DICTIONARY = 'tag36h11'
PT_PER_MM = 72.0 / 25.4

# ISO 216 and North American sizes, portrait, mm.
PAPERS: Dict[str, Tuple[float, float]] = {
    'A0': (841, 1189), 'A1': (594, 841), 'A2': (420, 594), 'A3': (297, 420),
    'A4': (210, 297), 'A5': (148, 210), 'A6': (105, 148),
    'B1': (707, 1000), 'B2': (500, 707), 'B3': (353, 500), 'B4': (250, 353), 'B5': (176, 250),
    'LETTER': (215.9, 279.4), 'LEGAL': (215.9, 355.6), 'TABLOID': (279.4, 431.8),
}
AUTO_PAPERS = ['A4', 'A3', 'A2', 'A1', 'A0']

MARGIN = 10.0          # mm kept free at the page edge (printers cannot print there)
FOOTER = 20.0          # mm for the scale bar and the printing hint
GAP = 8.0              # mm between markers on a shared page

UNITS = {'mm': 1.0, 'cm': 10.0, 'm': 1000.0, 'in': 25.4}


class SheetError(ValueError):
    """A job that cannot be printed; the message says what to change."""


# ------------------------------------------------------------------ units

def parse_length(text: str, what: str = 'length') -> float:
    """'450mm', '45 cm', '0.45m', '17.7in' -> millimetres. A unit is required."""
    m = re.fullmatch(r'\s*([0-9]*\.?[0-9]+)\s*(mm|cm|m|in)\s*', str(text).lower())
    if not m:
        raise SheetError(f'{what} {text!r}: give a number and a unit, such as 450mm, 45cm or 0.45m')
    value = float(m.group(1)) * UNITS[m.group(2)]
    if value <= 0:
        raise SheetError(f'{what} {text!r} must be more than zero')
    return value


def parse_paper(text: str) -> Tuple[str, Optional[Tuple[float, float]]]:
    """'auto', 'exact', a paper name, or 'WxH<unit>' -> (kind, (w, h) mm or None)."""
    t = str(text).strip()
    low = t.lower()
    if low in ('auto', 'exact'):
        return low, None
    if t.upper() in PAPERS:
        return t.upper(), PAPERS[t.upper()]
    m = re.fullmatch(r'\s*([0-9]*\.?[0-9]+)\s*[x*]\s*([0-9]*\.?[0-9]+)\s*(mm|cm|m|in)\s*', low)
    if m:
        u = UNITS[m.group(3)]
        w, h = float(m.group(1)) * u, float(m.group(2)) * u
        return f'{_num(w)}x{_num(h)}mm', (w, h)
    raise SheetError(f'paper {text!r}: use auto, exact, one of {", ".join(sorted(PAPERS))}, '
                     'or a size such as 600x850mm')


def parse_ids(text: str) -> List[int]:
    """'0-9', '3,7,12', '0-3, 8' -> IDs in the order given, without repeats."""
    out: List[int] = []
    for part in str(text).replace(' ', '').split(','):
        if not part:
            continue
        m = re.fullmatch(r'(\d+)(?:-(\d+))?', part)
        if not m:
            raise SheetError(f'IDs {text!r}: use numbers and ranges such as 0-9 or 3,7,12')
        a = int(m.group(1))
        b = int(m.group(2)) if m.group(2) is not None else a
        if b < a:
            raise SheetError(f'IDs {part!r}: the range goes down')
        for i in range(a, b + 1):
            if i not in out:
                out.append(i)
    if not out:
        raise SheetError('no IDs given')
    return out


def _num(v: float) -> str:
    """A length without trailing zeros: 450.0 -> '450', 37.5 -> '37.5'."""
    return f'{v:.2f}'.rstrip('0').rstrip('.')


def size_label(mm: float) -> str:
    """How a size reads on a sheet: '45 cm', '37.5 cm', '123.4 mm'."""
    t = size_text(mm)
    return t[:-2] + ' ' + t[-2:]


def size_text(mm: float) -> str:
    """How a size reads on a sheet and in a file name: whole cm as cm, else mm."""
    return f'{_num(mm / 10)}cm' if abs(mm / 10 - round(mm / 10, 1)) < 1e-9 else f'{_num(mm)}mm'


# ------------------------------------------------------------------ dictionary

@dataclass
class Family:
    name: str              # as the user gave it, e.g. tag36h11
    cv_name: str           # DICT_APRILTAG_36h11
    bits: int              # data bits per side (6 for tag36h11)
    count: int             # IDs in the dictionary
    dictionary: object     # cv2.aruco dictionary

    @property
    def cells(self) -> int:
        """Cells per side of the black square: data bits plus the black border."""
        return self.bits + 2


def family(name: str = DEFAULT_DICTIONARY) -> Family:
    d = opencv_dictionary(name)
    return Family(name, opencv_dictionary_name(name), int(d.markerSize), int(len(d.bytesList)), d)


def marker_cells(fam: Family, marker_id: int) -> np.ndarray:
    """The marker as a square array, 1 = black, black border included."""
    import cv2
    if not 0 <= marker_id < fam.count:
        raise SheetError(f'ID {marker_id} is not in {fam.name} (IDs 0 to {fam.count - 1})')
    side = fam.cells
    if hasattr(cv2.aruco, 'generateImageMarker'):
        img = cv2.aruco.generateImageMarker(fam.dictionary, marker_id, side)
    else:
        img = cv2.aruco.drawMarker(fam.dictionary, marker_id, side)
    return (np.asarray(img) < 128).astype(np.uint8)


def _all_bits(fam: Family) -> np.ndarray:
    """(count, 4, bits*bits): the data bits of every ID in its four rotations."""
    out = np.zeros((fam.count, 4, fam.bits * fam.bits), np.uint8)
    for i in range(fam.count):
        inner = marker_cells(fam, i)[1:-1, 1:-1]
        for k in range(4):
            out[i, k] = np.rot90(inner, k).ravel()
    return out


def select_ids(fam: Family, count: int, exclude: Sequence[int] = ()) -> Tuple[List[int], int]:
    """`count` IDs, deterministic, far apart in bits (module docstring); and the smallest
    distance between any two of them (or to an excluded ID)."""
    excluded = sorted({int(i) for i in exclude if 0 <= int(i) < fam.count})
    free = fam.count - len(excluded)
    if count < 1:
        raise SheetError('ask for at least one marker')
    if count > free:
        raise SheetError(f'{fam.name} has {fam.count} IDs and {len(excluded)} are excluded: '
                         f'{count} distinct markers are too many; use a larger dictionary')
    bits = _all_bits(fam)
    base = bits[:, 0, :]                                   # each candidate, unrotated
    taken = np.zeros(fam.count, bool)
    mind = np.full(fam.count, np.iinfo(np.int32).max, np.int64)

    def add(i):
        taken[i] = True
        # distance of every candidate to i, the smallest over i's four rotations
        d = (base[:, None, :] != bits[i][None, :, :]).sum(axis=2).min(axis=1)
        np.minimum(mind, d, out=mind)

    for i in excluded:
        add(i)
    chosen: List[int] = []
    smallest = None
    for _ in range(count):
        if not chosen and not excluded:
            pick = 0
        else:
            cand = np.where(~taken, mind, -1)
            pick = int(np.argmax(cand))                    # argmax: the lowest ID on ties
            smallest = int(cand[pick]) if smallest is None else min(smallest, int(cand[pick]))
        chosen.append(pick)
        add(pick)
    return chosen, (smallest if smallest is not None else 0)


def min_distance(fam: Family, ids: Sequence[int]) -> int:
    """Smallest bit distance between two of these IDs, over rotations; 0 for one ID."""
    if len(ids) < 2:
        return 0
    bits = _all_bits(fam)
    best = None
    for a_i, a in enumerate(ids):
        for b in ids[a_i + 1:]:
            d = int((bits[a, 0][None, :] != bits[b]).sum(axis=1).min())
            best = d if best is None else min(best, d)
    return best


# ------------------------------------------------------------------ the job

@dataclass
class SheetJob:
    size_mm: float                         # edge of the black square
    count: Optional[int] = None            # distinct markers, if IDs are chosen here
    ids: Optional[List[int]] = None        # explicit IDs
    dictionary: str = DEFAULT_DICTIONARY
    paper: str = 'auto'
    border_mm: Optional[float] = None      # white margin; None: one cell
    per_page: str = 'auto'                 # 'auto' (several if they fit) or '1'
    labels: bool = True
    back_labels: bool = False
    exclude: Set[int] = field(default_factory=set)
    split: bool = False
    basename: str = 'markers'              # one-PDF mode: <basename>.pdf


@dataclass
class Prim:
    kind: str                              # rect, line, text, poly
    args: tuple


@dataclass
class Page:
    width: float                           # mm
    height: float
    prims: List[Prim]
    label: str                             # PDF page label, e.g. 'id 17'
    bookmark: Optional[str] = None
    ids: List[int] = field(default_factory=list)


@dataclass
class Plan:
    job: SheetJob
    family: Family
    ids: List[int]
    min_distance: int
    paper_name: str                        # A1, exact, 600x850mm
    paper: Tuple[float, float]             # page size, mm
    border_mm: float
    cell_mm: float
    per_page: int
    pages: List[Page]                      # the marker pages (and back pages), in order
    contact: List[Page]                    # the contact sheet
    warnings: List[str]

    def summary(self) -> dict:
        marker_pages = sum(1 for p in self.pages if p.ids)
        return {
            'dictionary': self.family.name, 'ids': self.ids, 'count': len(self.ids),
            'min_distance_bits': self.min_distance,
            'size_mm': self.job.size_mm, 'border_mm': round(self.border_mm, 3),
            'cell_mm': round(self.cell_mm, 3),
            'sheet_mm': round(self.job.size_mm + 2 * self.border_mm, 3),
            'paper': self.paper_name, 'paper_mm': [round(v, 1) for v in self.paper],
            'per_page': self.per_page, 'marker_pages': marker_pages,
            'pages': len(self.pages) + len(self.contact),
            'split': self.job.split, 'warnings': self.warnings,
        }


def plan_job(job: SheetJob) -> Plan:
    """Check the job and lay out every page; SheetError says what to change."""
    fam = family(job.dictionary)
    warnings: List[str] = []
    size = float(job.size_mm)
    cell = size / fam.cells
    border = float(job.border_mm) if job.border_mm is not None else cell
    if border < cell - 1e-9:
        warnings.append(f'white border {_num(border)} mm is under one cell ({_num(cell)} mm): '
                        'detectors may miss the marker')
    if cell < 2.0:
        warnings.append(f'cells of {_num(cell)} mm: hard to print sharply; use a larger size')

    if job.ids:
        ids = list(job.ids)
        for i in ids:
            if not 0 <= i < fam.count:
                raise SheetError(f'ID {i} is not in {fam.name} (IDs 0 to {fam.count - 1})')
        clash = sorted(set(ids) & set(job.exclude))
        if clash:
            warnings.append(f'IDs already used at the site: {", ".join(map(str, clash))}')
        if job.count is not None and job.count != len(ids):
            raise SheetError(f'{job.count} markers asked for but {len(ids)} IDs given')
        dist = min_distance(fam, sorted(set(ids) | set(job.exclude)) if job.exclude else ids)
    else:
        if job.count is None:
            raise SheetError('say how many markers (count) or which IDs')
        ids, dist = select_ids(fam, int(job.count), sorted(job.exclude))
        ids = sorted(ids)                   # pages in ID order; the choice stays the same
    if len(ids) > 1 and dist < 3:
        warnings.append(f'two of these IDs differ in only {dist} bits: a misread could turn one '
                        'into the other')

    kind, wh = parse_paper(job.paper)
    sheet = size + 2 * border
    if kind == 'exact':
        paper_name, paper, per_page = 'exact', (sheet, sheet), 1
    else:
        candidates = AUTO_PAPERS if kind == 'auto' else [kind]
        paper_name = None
        for name in candidates:
            pw, ph = PAPERS.get(name, wh)
            if _fits(sheet, size, pw, ph, job.labels):
                paper_name, paper = name, (pw, ph)
                break
        if paper_name is None:
            need = _needed(sheet, size, job.labels)
            bigger = [n for n in AUTO_PAPERS if _fits(sheet, size, *PAPERS[n], job.labels)]
            hint = (f'; {bigger[0]} fits' if bigger else
                    '; no standard paper fits: use --paper exact or a custom size')
            raise SheetError(f'a {size_label(size)} marker with its border and labels needs '
                             f'{_num(need[0])} x {_num(need[1])} mm; {kind if kind != "auto" else "A0"} '
                             f'is too small{hint}')
        cols, rows = _grid(sheet, size, paper[0], paper[1], job.labels)
        per_page = 1 if str(job.per_page) == '1' or job.split else cols * rows

    pages: List[Page] = []
    for start in range(0, len(ids), per_page):
        group = ids[start:start + per_page]
        if kind == 'exact':
            pages.append(_exact_page(fam, group[0], size, border, job.labels))
        else:
            pages.append(_paper_page(fam, group, size, border, paper, job.labels))
        if job.back_labels:
            for mid in group:
                pages.append(_back_page(fam, mid, size, paper))
    plan = Plan(job, fam, ids, dist, paper_name, paper, border, cell, per_page, pages, [], warnings)
    plan.contact = _contact_pages(plan)
    return plan


def _label_heights(size: float) -> Tuple[float, float, float]:
    """(ID text, detail text, top arrow) heights in mm, by marker size."""
    big = min(max(size * 0.06, 6.0), 30.0)
    return big, min(max(big * 0.4, 3.0), 8.0), min(max(size * 0.03, 4.0), 12.0)


def _block(sheet: float, size: float, labels: bool) -> Tuple[float, float]:
    """Width and height of one marker with its white border and labels."""
    if not labels:
        return sheet, sheet
    big, small, arrow = _label_heights(size)
    return sheet, arrow * 1.6 + sheet + big * 1.5 + small * 1.6


def _needed(sheet: float, size: float, labels: bool) -> Tuple[float, float]:
    bw, bh = _block(sheet, size, labels)
    return bw + 2 * MARGIN, bh + 2 * MARGIN + FOOTER


def _fits(sheet, size, pw, ph, labels) -> bool:
    nw, nh = _needed(sheet, size, labels)
    return nw <= pw + 1e-9 and nh <= ph + 1e-9


def _grid(sheet, size, pw, ph, labels) -> Tuple[int, int]:
    bw, bh = _block(sheet, size, labels)
    cols = int((pw - 2 * MARGIN + GAP) // (bw + GAP))
    rows = int((ph - 2 * MARGIN - FOOTER + GAP) // (bh + GAP))
    return max(cols, 1), max(rows, 1)


# ------------------------------------------------------------------ drawing

def _marker_prims(fam: Family, mid: int, x: float, y: float, size: float) -> List[Prim]:
    """Black cells as rectangles, runs along each row merged."""
    cells = marker_cells(fam, mid)
    n = cells.shape[0]
    c = size / n
    out = []
    for r in range(n):
        col = 0
        while col < n:
            if cells[r, col]:
                start = col
                while col < n and cells[r, col]:
                    col += 1
                out.append(Prim('rect', (x + start * c, y + r * c, (col - start) * c, c, 0.0)))
            else:
                col += 1
    return out


def _crop_marks(x: float, y: float, w: float, h: float) -> List[Prim]:
    """Short grey lines just outside the corners of a sheet, for trimming."""
    out, off, ln = [], 2.0, 6.0
    for cx, sx in ((x, -1), (x + w, 1)):
        for cy, sy in ((y, -1), (y + h, 1)):
            out.append(Prim('line', (cx + sx * off, cy, cx + sx * (off + ln), cy, 0.6, 0.2, None)))
            out.append(Prim('line', (cx, cy + sy * off, cx, cy + sy * (off + ln), 0.6, 0.2, None)))
    return out


def _detail(fam: Family, size: float) -> str:
    return f'{fam.name}, black square {size_label(size)}'


def _paper_page(fam, group, size, border, paper, labels) -> Page:
    pw, ph = paper
    sheet = size + 2 * border
    bw, bh = _block(sheet, size, labels)
    cols, rows = _grid(sheet, size, pw, ph, labels)
    n = len(group)
    used_cols = min(cols, n)
    used_rows = math.ceil(n / used_cols)
    total_w = used_cols * bw + (used_cols - 1) * GAP
    total_h = used_rows * bh + (used_rows - 1) * GAP
    x0 = (pw - total_w) / 2
    y0 = MARGIN + max(0.0, (ph - 2 * MARGIN - FOOTER - total_h) / 2)
    prims: List[Prim] = []
    big, small, arrow = _label_heights(size)
    for k, mid in enumerate(group):
        bx = x0 + (k % used_cols) * (bw + GAP)
        by = y0 + (k // used_cols) * (bh + GAP)
        sy = by + (arrow * 1.6 if labels else 0.0)            # top of the white sheet
        prims += _crop_marks(bx, sy, sheet, sheet)
        prims += _marker_prims(fam, mid, bx + border, sy + border, size)
        if labels:
            cx = bx + sheet / 2
            prims.append(Prim('poly', ([(cx, by + arrow * 0.15), (cx - arrow * 0.5, by + arrow),
                                        (cx + arrow * 0.5, by + arrow)], 0.35)))
            prims.append(Prim('text', (cx + arrow * 0.7, by + arrow, arrow * 0.6, 'TOP', 0.35, True,
                                       'start')))
            ty = sy + sheet + big * 1.15
            prims.append(Prim('text', (cx, ty, big, f'ID {mid}', 0.0, True, 'middle')))
            prims.append(Prim('text', (cx, ty + small * 1.5, small, _detail(fam, size), 0.3, False,
                                       'middle')))
    prims += _footer(pw, ph)
    ids = list(group)
    label = 'id ' + ', '.join(map(str, ids)) if len(ids) <= 4 else f'id {ids[0]} to {ids[-1]}'
    return Page(pw, ph, prims, label, label, ids)


def _footer(pw: float, ph: float) -> List[Prim]:
    """A 100 mm scale bar with 10 mm ticks and the printing hint."""
    out = []
    y = ph - MARGIN - 6.0
    bar = 100.0 if pw - 2 * MARGIN >= 110 else 50.0
    x = (pw - bar) / 2
    out.append(Prim('line', (x, y, x + bar, y, 0.0, 0.35, None)))
    for i in range(int(bar // 10) + 1):
        h = 3.0 if i % 5 == 0 else 1.8
        out.append(Prim('line', (x + i * 10, y, x + i * 10, y - h, 0.0, 0.3, None)))
    out.append(Prim('text', (pw / 2, y + 4.2, 3.0,
                             f'{int(bar)} mm: check with a ruler. Print at actual size (100 %), '
                             'not "fit to page".', 0.25, False, 'middle')))
    return out


def _exact_page(fam, mid, size, border, labels) -> Page:
    sheet = size + 2 * border
    prims = _marker_prims(fam, mid, border, border, size)
    if labels:
        h = min(border * 0.35, 6.0)
        if h >= 1.2:
            text = f'id {mid} {fam.name} {size_label(size)}'
            if _text_width(text, h) > sheet * 0.7:
                text = f'id {mid}'
            prims.append(Prim('text', (sheet - border * 0.15, sheet - border * 0.15, h, text, 0.55,
                                       False, 'end')))
    return Page(sheet, sheet, prims, f'id {mid}', f'id {mid}', [mid])


def _back_page(fam, mid, size, paper) -> Page:
    pw, ph = paper
    h = min(pw, ph) * 0.35
    prims = [Prim('text', (pw / 2, ph / 2 + h * 0.35, h, str(mid), 0.0, True, 'middle'))]
    sub = min(max(h * 0.08, 3.0), 12.0)
    prims.append(Prim('text', (pw / 2, ph / 2 + h * 0.35 + sub * 2.2, sub,
                               f'back of marker {mid}, {_detail(fam, size)}', 0.3, False, 'middle')))
    return Page(pw, ph, prims, f'id {mid} back', None, [])


def _contact_pages(plan: Plan) -> List[Page]:
    """A4 pages listing every ID with a small picture, where it is, and space to note
    where it was placed."""
    pw, ph = PAPERS['A4']
    rows_per_page = 11
    head = 34.0
    row_h = (ph - 2 * MARGIN - head) / rows_per_page
    where: Dict[int, str] = {}
    page_no = 0
    for p in plan.pages:
        page_no += 1
        for mid in p.ids:
            where[mid] = (marker_filename(plan, mid) if plan.job.split else f'page {page_no}')
    out = []
    ids = plan.ids
    for start in range(0, len(ids), rows_per_page):
        prims: List[Prim] = []
        y = MARGIN + 8
        prims.append(Prim('text', (MARGIN, y, 6.0, 'Markers', 0.0, True, 'start')))
        s = plan.summary()
        prims.append(Prim('text', (MARGIN, y + 7, 3.4,
                                   f'{plan.family.name}, black square {size_label(plan.job.size_mm)}, '
                                   f'white border {_num(plan.border_mm)} mm, paper {plan.paper_name}, '
                                   f'{len(ids)} IDs (at least {s["min_distance_bits"]} bits apart)',
                                   0.2, False, 'start')))
        prims.append(Prim('text', (MARGIN, y + 13, 3.4,
                                   'Measure the black square of a printed sheet: that length is the '
                                   'marker size the survey needs.', 0.2, False, 'start')))
        hy = MARGIN + head - 4
        for x, t in ((MARGIN + 22, 'ID'), (MARGIN + 50, 'in the PDF'), (MARGIN + 105, 'placed where')):
            prims.append(Prim('text', (x, hy, 3.4, t, 0.3, True, 'start')))
        for k, mid in enumerate(ids[start:start + rows_per_page]):
            ry = MARGIN + head + k * row_h
            thumb = row_h * 0.75
            prims.append(Prim('line', (MARGIN, ry + row_h, pw - MARGIN, ry + row_h, 0.8, 0.2, None)))
            prims += _marker_prims(plan.family, mid, MARGIN + 1, ry + (row_h - thumb) / 2, thumb)
            prims.append(Prim('text', (MARGIN + 22, ry + row_h * 0.62, 7.0, str(mid), 0.0, True, 'start')))
            prims.append(Prim('text', (MARGIN + 50, ry + row_h * 0.6, 3.4, where[mid], 0.2, False,
                                       'start')))
            prims.append(Prim('line', (MARGIN + 105, ry + row_h * 0.65, pw - MARGIN, ry + row_h * 0.65,
                                       0.6, 0.2, (1.0, 1.0))))
        n = start // rows_per_page + 1
        total = math.ceil(len(ids) / rows_per_page)
        label = 'contents' if total == 1 else f'contents {n}'
        out.append(Page(pw, ph, prims, label, label if n == 1 else None, []))
    return out


# ------------------------------------------------------------------ text metrics

_WIDTHS = {**{c: 0.556 for c in '0123456789'}, ' ': 0.278, '.': 0.278, ',': 0.278, ':': 0.278,
           '-': 0.333, '(': 0.333, ')': 0.333, '/': 0.278, '%': 0.889, '"': 0.355, "'": 0.191,
           'i': 0.222, 'l': 0.222, 'j': 0.222, 'f': 0.278, 't': 0.278, 'r': 0.333, 'm': 0.833,
           'w': 0.722, 'I': 0.278, 'J': 0.5, 'M': 0.833, 'W': 0.944}


def _text_width(text: str, height: float, bold: bool = False) -> float:
    """Approximate width in mm of Helvetica text at font size `height` mm (centring only)."""
    w = sum(_WIDTHS.get(ch, 0.667 if ch.isupper() else 0.556) for ch in text)
    return w * height * (1.06 if bold else 1.0)


# ------------------------------------------------------------------ PDF

def _esc(s: str) -> str:
    return s.replace('\\', '\\\\').replace('(', '\\(').replace(')', '\\)')


def _page_stream(page: Page) -> bytes:
    H = page.height
    ops = []

    def P(v):                                   # mm -> pt
        return f'{v * PT_PER_MM:.3f}'

    for prim in page.prims:
        a = prim.args
        if prim.kind == 'rect':
            x, y, w, h, g = a
            ops.append(f'{g:.3f} g {P(x)} {P(H - y - h)} {P(w)} {P(h)} re f')
        elif prim.kind == 'line':
            x1, y1, x2, y2, g, width, dash = a
            d = f'[{P(dash[0])} {P(dash[1])}] 0 d' if dash else '[] 0 d'
            ops.append(f'{g:.3f} G {P(width)} w {d} {P(x1)} {P(H - y1)} m {P(x2)} {P(H - y2)} l S')
        elif prim.kind == 'poly':
            pts, g = a
            path = ' '.join(f'{P(px)} {P(H - py)} {"m" if i == 0 else "l"}' for i, (px, py) in enumerate(pts))
            ops.append(f'{g:.3f} g {path} h f')
        elif prim.kind == 'text':
            x, y, h, text, g, bold, anchor = a
            w = _text_width(text, h, bold)
            tx = x - (w / 2 if anchor == 'middle' else w if anchor == 'end' else 0.0)
            ops.append(f'BT /{"F2" if bold else "F1"} {P(h)} Tf {g:.3f} g {P(tx)} {P(H - y)} Td '
                       f'({_esc(text)}) Tj ET')
    return '\n'.join(ops).encode('latin-1')


def pdf_bytes(pages: List[Page], title: str) -> bytes:
    """A PDF with these pages, a bookmark for each page that has one, and page labels.
    No dates inside: the same job gives the same bytes."""
    objs: Dict[int, bytes] = {}
    n_pages = len(pages)
    # object numbers: 1 catalog, 2 pages, 3 F1, 4 F2, 5 info, 6 outlines,
    # then per page: page, content; then one per bookmark
    page_ids = [7 + 2 * i for i in range(n_pages)]
    marks = [(i, p.bookmark) for i, p in enumerate(pages) if p.bookmark]
    mark_ids = [7 + 2 * n_pages + k for k in range(len(marks))]
    objs[2] = (f'<< /Type /Pages /Kids [{" ".join(f"{i} 0 R" for i in page_ids)}] '
               f'/Count {n_pages} >>').encode()
    objs[3] = b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>'
    objs[4] = b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold /Encoding /WinAnsiEncoding >>'
    objs[5] = f'<< /Title ({_esc(title)}) /Producer (marker_survey) >>'.encode('latin-1')
    if marks:
        objs[6] = (f'<< /Type /Outlines /First {mark_ids[0]} 0 R /Last {mark_ids[-1]} 0 R '
                   f'/Count {len(marks)} >>').encode()
    else:
        objs[6] = b'<< /Type /Outlines /Count 0 >>'
    for i, p in enumerate(pages):
        content = _page_stream(p)
        objs[page_ids[i]] = (f'<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {p.width * PT_PER_MM:.3f} '
                             f'{p.height * PT_PER_MM:.3f}] /Resources << /Font << /F1 3 0 R /F2 4 0 R '
                             f'>> >> /Contents {page_ids[i] + 1} 0 R >>').encode()
        objs[page_ids[i] + 1] = b'<< /Length %d >>\nstream\n' % len(content) + content + b'\nendstream'
    for k, (i, text) in enumerate(marks):
        links = ''
        if k > 0:
            links += f' /Prev {mark_ids[k - 1]} 0 R'
        if k < len(marks) - 1:
            links += f' /Next {mark_ids[k + 1]} 0 R'
        objs[mark_ids[k]] = (f'<< /Title ({_esc(text)}) /Parent 6 0 R /Dest [{page_ids[i]} 0 R /Fit]'
                             f'{links} >>').encode('latin-1')
    nums = ' '.join(f'{i} << /P ({_esc(p.label)}) >>' for i, p in enumerate(pages))
    objs[1] = (f'<< /Type /Catalog /Pages 2 0 R /Outlines 6 0 R /PageMode /UseOutlines '
               f'/PageLabels << /Nums [{nums}] >> >>').encode('latin-1')
    out = bytearray(b'%PDF-1.4\n%\xe2\xe3\xcf\xd3\n')
    offsets = {}
    for num in sorted(objs):
        offsets[num] = len(out)
        out += b'%d 0 obj\n' % num + objs[num] + b'\nendobj\n'
    xref = len(out)
    total = max(objs) + 1
    out += b'xref\n0 %d\n0000000000 65535 f \n' % total
    for num in range(1, total):
        out += b'%010d 00000 n \n' % offsets[num]
    out += (b'trailer\n<< /Size %d /Root 1 0 R /Info 5 0 R >>\nstartxref\n%d\n%%%%EOF\n'
            % (total, xref))
    return bytes(out)


# ------------------------------------------------------------------ SVG (the web preview)

def page_svg(page: Page, max_px: int = 360) -> str:
    """The page as SVG in millimetres, scaled to fit `max_px` on its long side."""
    scale = max_px / max(page.width, page.height)
    w, h = page.width * scale, page.height * scale
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w:.0f}" height="{h:.0f}" '
             f'viewBox="0 0 {page.width:.3f} {page.height:.3f}" shape-rendering="crispEdges">',
             f'<rect x="0" y="0" width="{page.width:.3f}" height="{page.height:.3f}" fill="#fff"/>']

    def col(g):
        v = int(round(g * 255))
        return f'#{v:02x}{v:02x}{v:02x}'

    for prim in page.prims:
        a = prim.args
        if prim.kind == 'rect':
            x, y, rw, rh, g = a
            parts.append(f'<rect x="{x:.3f}" y="{y:.3f}" width="{rw:.3f}" height="{rh:.3f}" '
                         f'fill="{col(g)}"/>')
        elif prim.kind == 'line':
            x1, y1, x2, y2, g, width, dash = a
            da = f' stroke-dasharray="{dash[0]} {dash[1]}"' if dash else ''
            parts.append(f'<line x1="{x1:.3f}" y1="{y1:.3f}" x2="{x2:.3f}" y2="{y2:.3f}" '
                         f'stroke="{col(g)}" stroke-width="{width:.3f}"{da}/>')
        elif prim.kind == 'poly':
            pts, g = a
            parts.append(f'<polygon points="{" ".join(f"{px:.3f},{py:.3f}" for px, py in pts)}" '
                         f'fill="{col(g)}"/>')
        elif prim.kind == 'text':
            x, y, th, text, g, bold, anchor = a
            t = text.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
            weight = ' font-weight="bold"' if bold else ''
            parts.append(f'<text x="{x:.3f}" y="{y:.3f}" font-size="{th:.3f}" '
                         f'font-family="Helvetica, Arial, sans-serif" fill="{col(g)}" '
                         f'text-anchor="{anchor}"{weight}>'
                         f'{t}</text>')
    parts.append('</svg>')
    return ''.join(parts)


# ------------------------------------------------------------------ outputs

def marker_filename(plan: Plan, mid: int) -> str:
    return f'{plan.family.name}_{size_text(plan.job.size_mm)}_id{mid:03d}.pdf'


def manifest_csv(plan: Plan) -> str:
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator='\n')
    w.writerow(['id', 'dictionary', 'size_mm', 'border_mm', 'paper', 'file', 'page'])
    page_no = 0
    for p in plan.pages:
        page_no += 1
        for mid in p.ids:
            if plan.job.split:
                w.writerow([mid, plan.family.name, _num(plan.job.size_mm), _num(plan.border_mm),
                            plan.paper_name, marker_filename(plan, mid), 1])
            else:
                w.writerow([mid, plan.family.name, _num(plan.job.size_mm), _num(plan.border_mm),
                            plan.paper_name, f'{plan.job.basename}.pdf', page_no])
    return buf.getvalue()


def outputs(plan: Plan) -> List[Tuple[str, bytes]]:
    """(file name, bytes) for every file the job writes."""
    title = f'{plan.family.name} markers, black square {size_label(plan.job.size_mm)}'
    files: List[Tuple[str, bytes]] = []
    if plan.job.split:
        k = 0
        while k < len(plan.pages):
            page = plan.pages[k]
            group = [page]
            k += 1
            while k < len(plan.pages) and not plan.pages[k].ids:      # its back pages
                group.append(plan.pages[k])
                k += 1
            name = marker_filename(plan, page.ids[0])          # split: one marker per page
            files.append((name, pdf_bytes(group, f'{title}, {page.label}')))
        files.append(('contact_sheet.pdf', pdf_bytes(plan.contact, f'{title}, contents')))
    else:
        files.append((f'{plan.job.basename}.pdf', pdf_bytes(plan.pages + plan.contact, title)))
    csv_name = 'markers.csv' if plan.job.split else f'{plan.job.basename}.csv'
    files.append((csv_name, manifest_csv(plan).encode()))
    return files


def zip_bytes(files: List[Tuple[str, bytes]]) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED) as z:
        for name, data in files:
            info = zipfile.ZipInfo(name, date_time=(2000, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(info, data)
    return buf.getvalue()
