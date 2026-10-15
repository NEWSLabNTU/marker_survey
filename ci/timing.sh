#!/bin/bash
# Detection and fit time per frame of the localizer, on one core and one OpenCV
# thread, for the two cameras of phase 3. Rendered frames with blur (0.8 px)
# and sensor noise (3 grey levels); the rendering is not timed. Runs from the
# repository root without a ROS workspace (numpy, scipy, OpenCV, PyYAML):
#
#   ci/timing.sh [CPU]      # CPU: the core to pin to (default 0)
set -eo pipefail
cpu=${1:-0}
export PYTHONPATH=$PWD/marker_survey:$PWD/marker_localizer${PYTHONPATH:+:$PYTHONPATH}
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
run() { taskset -c "$cpu" python3 -m marker_localizer.tools.evaluate --timing --blur-px 0.8 --noise-grey 3 "$@"; }
sites=examples/sites
echo "== ceiling webcam: 640 x 480 grey, tag36h11, planar"
run --map $sites/office/marker_map.yaml --route $sites/office/route.csv --camera upward \
    --laps 1 --rate 5 --speed 0.5 --methods planar --max-frames 200 | grep -E 'frames|^planar|detection'
for scale in 1.0 0.5; do
  echo "== wide forward camera: 1920 x 1200 colour, DICT_5X5_1000, 3d, detect_scale $scale"
  run --map $sites/test_course/marker_map.yaml --route $sites/test_course/route.csv --camera forward \
      --width 1920 --height 1200 --laps 1 --rate 3 --sweeps 4 --methods 3d --colour \
      --detect-scale $scale --max-frames 150 | grep -E 'frames|^3d|detection'
done
python3 -c "import cv2, platform; print('OpenCV', cv2.__version__, platform.machine())"
