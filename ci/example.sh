#!/bin/bash
# Run one of the examples on its simulated recording, the way
# docs/integration.md describes it: render the recording, survey it through
# the example's survey launch file and compare the map with the truth, then
# localize on it through the example's localizer launch file and score the
# fixes. Run from the repository root in a sourced ROS 2 workspace where the
# packages are built:
#
#   ci/example.sh wall_course [OUT_DIR]
#   ci/example.sh ceiling [OUT_DIR]
#   ci/example.sh handheld_ceiling [OUT_DIR]
#
# Exits non-zero when the survey or the localizer misses its limits.
set -eo pipefail
source "$(dirname "$0")/lib.sh"
set -m      # every background job in its own process group: stop() reaches launch and its nodes
example=${1:?example name: wall_course, ceiling or handheld_ceiling}
mkdir -p "${2:-tmp/ci}"
out=$(cd "${2:-tmp/ci}" && pwd)/example_$example
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-39}
ex=examples/$example

case $example in
  wall_course)
    site=$ex/sites/test_course
    record=(--camera forward --origin 10 --laps 1 --sweeps 4 --rate 3
            --map-noise 0.05 --map-noise-yaw-deg 1.0)
    survey=(origin_marker:=10 image_topic:=/camera/image odom_topic:=/odom)
    truth=truth_origin.yaml
    survey_limits=(--max-median-cm 1.0 --max-cm 4.0)
    # the survey's frame is marker 10's; the recording's truth is in the course map's
    localize_map=$PWD/$site/marker_map.yaml
    localizer=(image_topic:=/camera/image prior_topic:=/prior_pose detect_scale:=1.0)
    localizer_limits=(--mode 3d --max-mean-cm 2.0 --max-cm 10.0)
    ;;
  ceiling)
    site=$ex/sites/office
    record=(--camera upward --laps 2 --rate 5 --speed 0.5 --map-noise 0.05 --map-noise-yaw-deg 1.0)
    survey=(image_topic:=/camera/image camera_info_topic:=/camera/camera_info)
    truth=truth_map.yaml
    survey_limits=(--max-median-cm 2.0)
    # align_to_track surveys into the map frame: localize on the map just surveyed
    localize_map=$out/survey/marker_map.yaml
    localizer=(image_topic:=/camera/image camera_info_topic:=/camera/camera_info
               prior_topic:=/prior_pose prior_type:=pose)
    localizer_limits=(--mode planar --max-mean-cm 1.0 --max-cm 3.0)
    ;;
  handheld_ceiling)
    # a person walking with the camera held up: no odometry, no TF; tilted away from
    # the ceiling for 3 s halfway along the middle column (a kidnap), and past the west
    # end where no marker is in view
    site=$ex/sites/office
    record=(--camera handheld --open --rate 5 --speed 0.5 --origin 0 --tilt-at 67)
    survey=(image_topic:=/camera/image camera_info_topic:=/camera/camera_info)
    truth=truth_origin.yaml
    # every marker good; median 1 cm, max 2 cm: half the robot's ceiling example's limit
    # (median 2 cm, against a reference track with 5 cm noise), since nothing but the
    # rendered corners limits the camera alone here (0.2 cm median, 0.3 cm max measured)
    survey_limits=(--max-median-cm ${HANDHELD_MEDIAN_CM:-1.0} --max-cm ${HANDHELD_MAX_CM:-2.0}
                   --require-good)
    # the origin marker's frame is the site map's (marker 0 at its origin, rotation 0):
    # localize on the map just surveyed, score against the true camera poses
    localize_map=$out/survey/marker_map.yaml
    localizer=(image_topic:=/camera/image camera_info_topic:=/camera/camera_info)
    # 3d, no prior: mean 1 cm as the ceiling example's planar fixes; max 6 cm for the
    # frames at the edges of the tilted stretch, which see markers at a slant (0.4 cm
    # mean, 4.6 cm max measured); the first fix after the stretch within 2 cm
    localizer_limits=(--mode 3d --max-mean-cm ${HANDHELD_LOC_MEAN_CM:-1.0}
                      --max-cm ${HANDHELD_LOC_MAX_CM:-6.0} --max-first-cm 2.0)
    ;;
  *)
    echo "unknown example $example"; exit 2;;
esac

rm -rf "$out"
mkdir -p "$out"

echo "== the simulated recording"
ros2 run marker_survey marker_survey_synthetic "$out/sim" \
  --map $site/marker_map.yaml --route $site/route.csv "${record[@]}"

echo "== survey: $ex/launch/survey.launch.xml"
ros2 launch $ex/launch/survey.launch.xml output_dir:=$out/survey \
  use_sim_time:=true process_all:=true "${survey[@]}" > "$out/survey.log" 2>&1 &
launch_pid=$!
trap 'stop $launch_pid INT 30' EXIT
for _ in $(seq 60); do
  ros2 service list 2>/dev/null | grep -q /marker_survey/save && break
  sleep 1
done
timeout 900 ros2 bag play "$out/sim/bag" --clock --delay 3 > "$out/survey_play.log" 2>&1
sleep 2
timeout 600 ros2 service call /marker_survey/save std_srvs/srv/Trigger | tail -2
stop $launch_pid INT 30
trap - EXIT
grep -E 'solved|aligned|keyframes,' "$out/survey.log" | tail -3 || true
ros2 run marker_survey marker_survey_compare "$out/survey/marker_map.yaml" "$out/sim/$truth" \
  "${survey_limits[@]}"
ros2 run marker_survey marker_map_check "$out/survey/marker_map.yaml" > /dev/null
if [ "$example" = handheld_ceiling ]; then
  # frames the camera alone could not place: the tilted stretch and the west end
  grep -E 'not placed' "$out/survey.log" | tail -1 || { echo "FAIL: no frame was left unplaced"; exit 1; }
  # the tilted stretch: no fix inside it, the first one after it in the right place
  gap=($(python3 -c "import json; print(*json.load(open('$out/sim/scene.json'))['tilt_window'])"))
  localizer_limits+=(--gap "${gap[@]}")
  echo "== a Gazebo world from the surveyed map"
  ros2 run marker_survey marker_map_to_sdf "$out/survey/marker_map.yaml" "$out/world/site.world" --world
  if command -v gz > /dev/null && gz sdf --help > /dev/null 2>&1; then
    gz sdf -k "$out/world/site.world"
  else
    python3 -c "import xml.etree.ElementTree as E, sys; E.parse(sys.argv[1]); print('parses (no gz sdf here)')" \
      "$out/world/site.world"
  fi
fi

echo "== localizer: $ex/launch/localizer.launch.xml on $localize_map"
ros2 launch $ex/launch/localizer.launch.xml marker_map:=$localize_map \
  use_sim_time:=true process_all:=true "${localizer[@]}" > "$out/localizer.log" 2>&1 &
launch_pid=$!
ros2 bag record -s sqlite3 -o "$out/poses" /marker_localizer/pose > "$out/record.log" 2>&1 &
rec_pid=$!
trap 'stop $rec_pid INT 10; stop $launch_pid INT 30' EXIT
for _ in $(seq 60); do
  ros2 topic info /marker_localizer/pose 2>/dev/null | grep -q 'Subscription count: [1-9]' && break
  sleep 1
done
timeout 900 ros2 bag play "$out/sim/bag" --clock --delay 3 > "$out/localizer_play.log" 2>&1
sleep 3
stop $rec_pid INT 30
stop $launch_pid INT 30
trap - EXIT
grep -E 'images,' "$out/localizer.log" | tail -2 || true
ros2 run marker_localizer marker_localizer_score "$out/poses" "$out/sim/truth_poses.csv" \
  "${localizer_limits[@]}" --min-fixes 100
