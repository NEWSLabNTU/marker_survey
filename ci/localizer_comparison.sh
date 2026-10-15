#!/bin/bash
# The localizer against the source implementations' methods on the same rendered
# frames (marker_localizer_evaluate, no ROS needed), with the phase 3 checks:
#
#   ci/localizer_comparison.sh ceiling|wall [OUT_DIR]
#
# ceiling: the office, 2 laps; planar mean 1 cm and max 3 cm, no worse than the
#          ceiling source's method; with two markers in view planar within 2 cm
#          and 3d worse.
# wall:    the test course, 1 lap; 3d no worse than the wall source's method
#          (mean and max), at least as many fixes; planar against 3d by distance.
# Frames are rendered with blur (0.8 px) and sensor noise (3 grey levels).
set -eo pipefail
scene=${1:?scene}
out=${2:-tmp/ci}
mkdir -p "$out"
export PYTHONPATH=$PWD/marker_survey:$PWD/marker_localizer${PYTHONPATH:+:$PYTHONPATH}
sites=examples/sites
json=$out/comparison_$scene.json
case $scene in
  ceiling)
    python3 -m marker_localizer.tools.evaluate --map $sites/office/marker_map.yaml \
      --route $sites/office/route.csv --camera upward --laps 2 --rate 5 --speed 0.5 \
      --methods planar,3d,ceiling_source --blur-px 0.8 --noise-grey 3 --json "$json" ;;
  wall)
    python3 -m marker_localizer.tools.evaluate --map $sites/test_course/marker_map.yaml \
      --route $sites/test_course/route.csv --camera forward --laps 1 --rate 3 --sweeps 4 \
      --methods 3d,planar,wall_source --blur-px 0.8 --noise-grey 3 --json "$json" ;;
  *) echo "unknown scene $scene"; exit 2 ;;
esac
python3 - "$scene" "$json" <<'EOF'
import json, sys
scene, path = sys.argv[1], sys.argv[2]
s = json.load(open(path))
fail = []
def check(cond, what):
    print(('ok    ' if cond else 'FAIL  ') + what)
    if not cond:
        fail.append(what)
if scene == 'ceiling':
    p, d, c = s['planar'], s['3d'], s['ceiling_source']
    check(p['all']['mean_cm'] <= 1.0, f"planar mean {p['all']['mean_cm']:.2f} cm <= 1")
    check(p['all']['max_cm'] <= 3.0, f"planar max {p['all']['max_cm']:.2f} cm <= 3")
    check(p['all']['mean_cm'] <= c['all']['mean_cm'] + 0.05,
          f"planar mean <= source method mean {c['all']['mean_cm']:.2f} cm")
    check(p['all']['max_cm'] <= c['all']['max_cm'] + 0.1,
          f"planar max <= source method max {c['all']['max_cm']:.2f} cm")
    p2, d2 = p['by_count']['2'], d['by_count']['2']
    check(p2 is not None and p2['max_cm'] < 2.0, f"two markers: planar max {p2 and p2['max_cm']:.2f} cm < 2")
    check(d2 is not None and d2['mean_cm'] > p2['mean_cm'],
          f"two markers: 3d mean {d2 and d2['mean_cm']:.2f} cm worse than planar")
else:
    d, w, p = s['3d'], s['wall_source'], s['planar']
    check(d['all']['mean_cm'] <= w['all']['mean_cm'],
          f"3d mean {d['all']['mean_cm']:.2f} cm <= source method {w['all']['mean_cm']:.2f} cm")
    check(d['all']['max_cm'] <= w['all']['max_cm'],
          f"3d max {d['all']['max_cm']:.2f} cm <= source method {w['all']['max_cm']:.2f} cm")
    check(d['fixes'] >= w['fixes'], f"3d fixes {d['fixes']} >= source method {w['fixes']}")
    print('      x, y error by distance to the markers, mean / max (fixes):')
    for band, v in d['by_distance_xy'].items():
        q = p['by_distance_xy'][band]
        if v and q:
            print(f"      {band}: 3d {v['mean_cm']:.2f} / {v['max_cm']:.2f} cm ({v['n']}), "
                  f"planar {q['mean_cm']:.2f} / {q['max_cm']:.2f} cm ({q['n']})")
sys.exit(1 if fail else 0)
EOF
