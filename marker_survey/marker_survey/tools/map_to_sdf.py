"""marker_map_to_sdf: the markers of a marker map as Gazebo models, or a whole world.

    marker_map_to_sdf site/marker_map.yaml markers.sdf             # the models only
    marker_map_to_sdf site/marker_map.yaml site.world --world      # a world around them
    marker_map_to_sdf site/marker_map.yaml --replace-between my.world

Every marker is a flat sheet (black square of its size and a white margin of
one cell, as marker_sheets prints it) at its pose in the map, facing the way
the marker faces: ceiling, wall or floor. The textures (PNG, one per marker)
are written next to the output and named in it by relative paths.

--world adds a ground plane, a ceiling plane (at --ceiling, or just behind the
highest ceiling marker), lights that show markers facing down to an upward
camera, and the system plugins Gazebo Harmonic needs to render cameras. The
models sit between the lines `<!-- BEGIN ceiling markers ... -->` and
`<!-- END ceiling markers -->`.

--replace-between FILE rewrites the lines between those two lines in an
existing world (a robot's own) with the map's models, keeping both lines and
everything else; --begin and --end choose other comment lines.
"""
import argparse
import os
import sys

from marker_survey.core import marker_map as mm
from marker_survey.core import sdf


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('map', help='marker_map.yaml')
    ap.add_argument('out', nargs='?', help='the SDF to write (models only, or a world with --world)')
    ap.add_argument('--world', action='store_true', help='a whole world, not only the models')
    ap.add_argument('--ceiling', type=float, help='[m] the ceiling plane\'s height (--world)')
    ap.add_argument('--world-name', default='markers')
    ap.add_argument('--replace-between', metavar='FILE',
                    help='rewrite the models between the BEGIN and END lines of this world')
    ap.add_argument('--begin', default=sdf.BEGIN, help='the text of the BEGIN line (a prefix of it)')
    ap.add_argument('--end', default=sdf.END, help='the text of the END line')
    ap.add_argument('--prefix', default='marker', help='model and texture names: PREFIX_ID')
    ap.add_argument('--cell-px', type=int, default=sdf.CELL_PX, help='texture pixels per marker cell')
    a = ap.parse_args(argv)
    if bool(a.out) == bool(a.replace_between):
        ap.error('give either OUT or --replace-between FILE')
    if a.replace_between and a.world:
        ap.error('--world and --replace-between exclude each other')
    try:
        the_map = mm.load(a.map)
    except (OSError, ValueError) as e:
        print(f'marker_map_to_sdf: {e}', file=sys.stderr)
        return 2
    bad = sdf.check_ids(the_map.ids(), the_map.dictionary)
    if bad:
        for mid, why in bad.items():
            print(f'marker {mid}: {why}', file=sys.stderr)
        return 2
    target = a.replace_between or a.out
    out_dir = os.path.dirname(os.path.abspath(target))
    if a.replace_between:
        with open(target) as f:
            text = f.read()
        indent = '    '
        for line in text.split('\n'):
            if a.begin in line:
                indent = line[:len(line) - len(line.lstrip())]
        models = sdf.models_sdf(the_map, out_dir, a.prefix, a.cell_px, indent=indent)
        try:
            text = sdf.replace_between(text, models, a.begin, a.end)
        except ValueError as e:
            print(f'marker_map_to_sdf: {target}: {e}', file=sys.stderr)
            return 2
    else:
        models = sdf.models_sdf(the_map, out_dir, a.prefix, a.cell_px)
        text = (sdf.world_sdf(the_map, models, a.ceiling, a.world_name, a.begin, a.end) if a.world
                else models + '\n')
    with open(target, 'w') as f:
        f.write(text)
    print(f'{len(the_map.markers)} markers -> {target}'
          + (' (world)' if a.world else ' (between the BEGIN and END lines)' if a.replace_between else '')
          + f'; textures {a.prefix}_<id>.png in {out_dir}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
