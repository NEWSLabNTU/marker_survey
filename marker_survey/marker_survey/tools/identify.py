"""marker_identify: read the marker IDs in a photo.

    marker_identify photo.jpg                      # any common family
    marker_identify photo.jpg --dictionary tag36h11
    marker_identify ceiling_*.jpg --no-image

Prints each marker's ID and writes <photo>_ids.jpg with every marker
outlined and its ID in large print (the red dot is the marker's top-left
corner). For a sheet whose label is gone, or markers already on the ceiling.
"""
import argparse
import os
import sys

from marker_survey.core.identify import annotate, identify, summary


def main(argv=None):
    p = argparse.ArgumentParser(prog='marker_identify', description=__doc__.splitlines()[0])
    p.add_argument('images', nargs='+', help='photos (JPEG, PNG)')
    p.add_argument('--dictionary', default='auto', help='marker family, or auto (default)')
    p.add_argument('--no-image', action='store_true', help='do not write the annotated copy')
    a = p.parse_args(argv)
    import cv2
    status = 0
    for path in a.images:
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            print(f'{path}: cannot read', file=sys.stderr)
            status = 1
            continue
        found = identify(img, a.dictionary)
        family, ids = summary(found)
        if not ids:
            print(f'{path}: no marker found')
            status = max(status, 1)
            continue
        print(f'{path}: {family}: ' + ', '.join(f'id {i}' for i in ids))
        if not a.no_image:
            stem, _ = os.path.splitext(path)
            cv2.imwrite(f'{stem}_ids.jpg', annotate(img, found))
    return status


if __name__ == '__main__':
    raise SystemExit(main())
