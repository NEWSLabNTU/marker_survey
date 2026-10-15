"""marker_sheets: print-ready PDF sheets of markers at an exact size.

    marker_sheets --size 45cm --count 24 -o markers.pdf
    marker_sheets --size 45cm --count 24 --paper A1 -o markers.pdf
    marker_sheets --size 45cm --count 24 --paper exact --border 6cm -o markers.pdf
    marker_sheets --size 45cm --count 24 --split -o markers/
    marker_sheets --size 12cm --ids 0-9 --paper A4 -o small.pdf
    marker_sheets --size 45cm --count 6 --site site/marker_map.yaml -o more.pdf

--size is the edge of the black square, with a unit. The IDs are chosen
deterministically, as far apart in bits as the dictionary allows, unless
--ids names them; --site or --exclude keeps IDs already in use out. Every
page says its ID (bookmark, page label, the sheet itself, the file name with
--split), and a contact sheet page and a CSV manifest (<name>.csv, or
markers.csv with --split) list them all.
core/sheets.py explains the layout.
"""
import argparse
import os
import sys

from marker_survey.core import marker_map
from marker_survey.core.sheets import (DEFAULT_DICTIONARY, SheetError, SheetJob, outputs,
                                       parse_ids, parse_length, plan_job)


def build_parser():
    p = argparse.ArgumentParser(prog='marker_sheets', description=__doc__.splitlines()[0])
    p.add_argument('--size', required=True, help='edge of the black square, with a unit: 450mm, 45cm, 0.45m')
    p.add_argument('--count', type=int, help='number of distinct markers (IDs chosen for you)')
    p.add_argument('--ids', help='these IDs instead: 0-9 or 3,7,12')
    p.add_argument('--dictionary', default=DEFAULT_DICTIONARY,
                   help=f'marker family (default {DEFAULT_DICTIONARY}); OpenCV names such as '
                        'DICT_5X5_1000 too')
    p.add_argument('--paper', default='auto',
                   help='auto (smallest of A4 to A0 that fits), A0 to A6, B1 to B5, Letter, Legal, '
                        'Tabloid, a size such as 600x850mm, or exact (page = marker + white border)')
    p.add_argument('--border', help='white margin around the black square, with a unit '
                                    '(default: one cell of the marker)')
    p.add_argument('--per-page', choices=['auto', '1'], default='auto',
                   help='auto: several markers on a page when they fit; 1: one per page')
    p.add_argument('--split', action='store_true',
                   help='one PDF per marker, named after its ID, in the output directory')
    p.add_argument('--no-label', action='store_true', help='no ID text on the sheets')
    p.add_argument('--back-labels', action='store_true',
                   help='after each marker page, a page with its ID in large print (duplex or the back)')
    p.add_argument('--site', help="a site's marker_map.yaml: its IDs are not used again")
    p.add_argument('--exclude', help='IDs not to use: 0-9 or 3,7,12')
    p.add_argument('-o', '--output', required=True,
                   help='the PDF to write; with --split, a directory')
    return p


def job_from_args(a) -> SheetJob:
    if a.count is None and not a.ids:
        raise SheetError('give --count (IDs chosen for you) or --ids')
    exclude = set(parse_ids(a.exclude)) if a.exclude else set()
    if a.site:
        exclude |= set(marker_map.load(a.site).ids())
    out = a.output
    base = 'markers'
    if not a.split:
        if not out.lower().endswith('.pdf'):
            raise SheetError(f'output {out!r}: give a .pdf file, or use --split with a directory')
        base = os.path.splitext(os.path.basename(out))[0]
    return SheetJob(size_mm=parse_length(a.size, 'size'), count=a.count,
                    ids=parse_ids(a.ids) if a.ids else None, dictionary=a.dictionary,
                    paper=a.paper, border_mm=parse_length(a.border, 'border') if a.border else None,
                    per_page=a.per_page, labels=not a.no_label, back_labels=a.back_labels,
                    exclude=exclude, split=a.split, basename=base)


def main(argv=None):
    a = build_parser().parse_args(argv)
    try:
        job = job_from_args(a)
        plan = plan_job(job)
    except (SheetError, ValueError) as e:
        print(f'marker_sheets: {e}', file=sys.stderr)
        return 2
    files = outputs(plan)
    folder = a.output if a.split else (os.path.dirname(a.output) or '.')
    os.makedirs(folder, exist_ok=True)
    for name, data in files:
        with open(os.path.join(folder, name), 'wb') as f:
            f.write(data)
    s = plan.summary()
    print(f'{s["count"]} markers, {s["dictionary"]}, black square {a.size}, white border '
          f'{s["border_mm"]:g} mm (sheet {s["sheet_mm"]:g} mm), paper {s["paper"]}, '
          f'{s["per_page"]} per page')
    print(f'IDs: {", ".join(map(str, s["ids"]))}'
          + (f' (at least {s["min_distance_bits"]} bits apart)' if s['count'] > 1 else ''))
    for w in s['warnings']:
        print(f'warning: {w}')
    for name, _ in files:
        print(f'wrote {os.path.join(folder, name)}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
