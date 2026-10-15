#!/bin/bash
# The web interface in a real browser, on simulated recordings of the ceiling
# example: render the drives, start examples/ceiling/launch/web.launch.xml (the
# survey, the localizer and the web node), then ci/web_browser_test.py walks the
# steps in Chromium (Playwright) and plays the recordings when a person would
# drive. Run from the repository root in a sourced ROS 2 workspace where the
# packages are built, with Playwright for Python installed:
#
#   ci/web_browser.sh [OUT_DIR]
#
# Exits non-zero when a step fails; screenshots of every screen in OUT_DIR/web/shots.
set -eo pipefail
source "$(dirname "$0")/lib.sh"
set -m
mkdir -p "${1:-tmp/ci}"
out=$(cd "${1:-tmp/ci}" && pwd)/web
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-40}
site=examples/ceiling/sites/office
port=${WEB_PORT:-8090}

rm -rf "$out"
mkdir -p "$out/site"
echo "== the survey drive (two laps, stopped once after 60 s) and the check drive (two laps, later)"
ros2 run marker_survey marker_survey_synthetic "$out/survey_drive" --map $site/marker_map.yaml \
  --route $site/route.csv --camera upward --laps 2 --rate 5 --speed 0.5 \
  --map-noise 0.05 --map-noise-yaw-deg 1.0 --split-at 60 > "$out/render1.log" 2>&1 &
r1=$!
ros2 run marker_survey marker_survey_synthetic "$out/check_drive" --map $site/marker_map.yaml \
  --route $site/route.csv --camera upward --laps 2 --rate 5 --speed 0.5 --seed 2 \
  --map-noise 0.05 --map-noise-yaw-deg 1.0 --start-time 1000010000 --split-at 90 > "$out/render2.log" 2>&1
wait $r1
tail -1 "$out/render1.log"; tail -1 "$out/render2.log"

# the map the robot used before: an older survey, with marker 5 since moved by 30 cm
sed 's/{id: 5, x:  0.000/{id: 5, x:  0.300/' $site/marker_map.yaml > "$out/site/marker_map.yaml"
grep -q 'id: 5, x:  0.300' "$out/site/marker_map.yaml"

echo "== examples/ceiling/launch/web.launch.xml"
ros2 launch examples/ceiling/launch/web.launch.xml site:=$out/site port:=$port \
  image_topic:=/camera/image camera_info_topic:=/camera/camera_info \
  prior_topic:=/prior_pose prior_type:=pose use_sim_time:=true process_all:=true \
  > "$out/launch.log" 2>&1 &
launch_pid=$!
trap 'stop $launch_pid INT 30' EXIT
for _ in $(seq 60); do
  kill -0 $launch_pid 2> /dev/null || { cat "$out/launch.log"; exit 1; }
  python3 -c "import urllib.request as u; u.urlopen('http://127.0.0.1:$port/api/state', timeout=2)" \
    2> /dev/null && break
  sleep 1
done

timeout 1500 python3 ci/web_browser_test.py "http://127.0.0.1:$port" "$out"
stop $launch_pid INT 30
trap - EXIT
