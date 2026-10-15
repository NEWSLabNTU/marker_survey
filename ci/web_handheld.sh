#!/bin/bash
# The web interface's handheld case in a real browser: plan, a camera-only survey and the
# localization test (ci/web_handheld_test.py, Chromium with Playwright). The server is the
# web interface with the stand-in of marker_survey_web/test/handheld_fixture.py: a person
# walking under a row of ceiling markers with an upward webcam, a step aside where no marker
# is in view (the survey loses track, the localizer's pose freezes, then jumps). Run from the
# repository root in a sourced ROS 2 workspace where the packages are built, with
# Playwright for Python installed:
#
#   ci/web_handheld.sh [OUT_DIR]
#
# Exits non-zero when a step fails; screenshots of every screen in OUT_DIR/handheld/shots.
set -eo pipefail
source "$(dirname "$0")/lib.sh"
mkdir -p "${1:-tmp/ci}"
out=$(cd "${1:-tmp/ci}" && pwd)/handheld
port=${WEB_PORT:-8091}
speed=${WALK_SPEED:-2.0}          # the walk at this many times real time

rm -rf "$out"
mkdir -p "$out"
PYTHONPATH="$PWD/marker_survey_web/test:$PYTHONPATH" \
  python3 marker_survey_web/test/handheld_fixture.py "$port" "$out/site" "$speed" > "$out/server.log" 2>&1 &
server=$!
trap 'stop $server TERM 10' EXIT
for _ in $(seq 60); do
  kill -0 $server 2> /dev/null || { cat "$out/server.log"; exit 1; }
  python3 -c "import urllib.request as u; u.urlopen('http://127.0.0.1:$port/api/state', timeout=2)" \
    2> /dev/null && break
  sleep 1
done

timeout 900 python3 ci/web_handheld_test.py "http://127.0.0.1:$port" "$out"
stop $server TERM 10
trap - EXIT
