#!/bin/bash
# The web interface's handheld case in a real browser: plan, a camera-only survey and the
# localization test (ci/web_handheld_test.py, Chromium with Playwright). No robot: the
# simulated recording of examples/handheld_ceiling (a person walking with an upward webcam
# under a 1.4 m grid of 0.40 m markers at 2.9 m, out past the west end where no marker is in
# view, and tilting the camera away from the ceiling for 3 s) is rendered twice, and
# marker_survey_web/launch/handheld.launch.xml runs the survey (the camera alone), the
# localizer (3d, no prior) and the web node on it. Run from the repository root in a
# sourced ROS 2 workspace where the packages are built, with Playwright for Python:
#
#   ci/web_handheld.sh [OUT_DIR]
#   STAND_IN=1 ci/web_handheld.sh [OUT_DIR]    # no ROS nodes: the stand-in of
#                                              # marker_survey_web/test/handheld_fixture.py
#
# Exits non-zero when a step fails; screenshots of every screen in OUT_DIR/handheld/shots.
set -eo pipefail
source "$(dirname "$0")/lib.sh"
set -m
mkdir -p "${1:-tmp/ci}"
out=$(cd "${1:-tmp/ci}" && pwd)/handheld
port=${WEB_PORT:-8091}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-41}
site=examples/handheld_ceiling/sites/office

rm -rf "$out"
mkdir -p "$out"
if [ -n "$STAND_IN" ]; then
  PYTHONPATH="$PWD/marker_survey_web/test:$PYTHONPATH" \
    python3 marker_survey_web/test/handheld_fixture.py "$port" "$out/site" "${WALK_SPEED:-2.0}" \
    > "$out/server.log" 2>&1 &
  drive=()
else
  echo "== the survey walk and the test walk (examples/handheld_ceiling, rendered)"
  record=(--map $site/marker_map.yaml --route $site/route.csv --camera handheld --open --rate 5
          --speed 0.5 --origin 0 --tilt-at 67)
  ros2 run marker_survey marker_survey_synthetic "$out/drive/survey" "${record[@]}" \
    > "$out/render1.log" 2>&1 &
  r1=$!
  ros2 run marker_survey marker_survey_synthetic "$out/drive/test" "${record[@]}" --seed 2 \
    --start-time 1000010000 > "$out/render2.log" 2>&1
  wait $r1
  tail -1 "$out/render1.log"; tail -1 "$out/render2.log"
  mv "$out/drive/survey/bag" "$out/drive/survey_walk"
  mv "$out/drive/test/bag" "$out/drive/test_walk"
  echo "== marker_survey_web/launch/handheld.launch.xml"
  ros2 launch marker_survey_web handheld.launch.xml site:=$out/site port:=$port launch_camera:=false \
    image_topic:=/camera/image camera_info_topic:=/camera/camera_info use_sim_time:=true \
    process_all:=true > "$out/server.log" 2>&1 &
  drive=(--drive "$out/drive")
fi
server=$!
trap 'stop $server INT 30' EXIT
for _ in $(seq 60); do
  kill -0 $server 2> /dev/null || { cat "$out/server.log"; exit 1; }
  python3 -c "import urllib.request as u; u.urlopen('http://127.0.0.1:$port/api/state', timeout=2)" \
    2> /dev/null && break
  sleep 1
done

timeout 1200 python3 ci/web_handheld_test.py "http://127.0.0.1:$port" "$out" "${drive[@]}"
stop $server INT 30
trap - EXIT
