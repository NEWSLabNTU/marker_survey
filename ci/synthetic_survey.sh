#!/bin/bash
# Render a synthetic recording, play it through the survey node, solve, and
# compare the result with the truth. Run from the repository root in a sourced
# ROS 2 workspace where marker_survey is built:
#
#   ci/synthetic_survey.sh wall [OUT_DIR]
#
# Scenes: wall, ceiling_pose_frame, ceiling_align, ceiling_uneven.
# Exits non-zero when the comparison fails.
set -eo pipefail
source "$(dirname "$0")/lib.sh"
scene=${1:?scene name}
out=${2:-tmp/ci}/$scene
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-37}
wall=examples/wall_course/sites; ceil=examples/ceiling/sites
common=(--ros-args -p use_sim_time:=true -p output_dir:=$PWD/$out/result -p rate:=10.0 -p process_all:=true)

case $scene in
  wall)
    # 11 boards around the test course, forward camera, one lap at 3 Hz; 3 % of the
    # odometry messages lost, which must not lose a frame
    gen=(--map $wall/test_course/marker_map.yaml --route $wall/test_course/route.csv
         --camera forward --origin 10 --laps 1 --sweeps 4 --rate 3 --drop-odom 0.03)
    node=(-p dictionary:=DICT_5X5_1000 -p marker_size:=0.38 -p pose_source:=odometry
          -p frame.mode:=origin_marker -p frame.origin_marker:=10)
    truth=truth_origin.yaml
    limits=(--max-median-cm ${WALL_MEDIAN_CM:-1.0} --max-cm ${WALL_MAX_CM:-4.0})
    ;;
  ceiling_pose_frame)
    # 9 ceiling markers, upward camera, the map pose noisy (6 cm per axis)
    gen=(--map $ceil/office/marker_map.yaml --route $ceil/office/route.csv
         --camera upward --laps 2 --rate 5 --speed 0.5 --map-noise 0.06 --map-noise-yaw-deg 1.0)
    node=(-p dictionary:=tag36h11 -p marker_size:=0.45 -p pose_source:=tf -p pose_frame:=map
          -p frame.mode:=pose_frame -p pose_sd:=0.06)
    truth=truth_map.yaml
    limits=(--max-median-cm 4.0)
    ;;
  ceiling_align)
    gen=(--map $ceil/office/marker_map.yaml --route $ceil/office/route.csv
         --camera upward --laps 2 --rate 5 --speed 0.5 --map-noise 0.06 --map-noise-yaw-deg 1.0)
    node=(-p dictionary:=tag36h11 -p marker_size:=0.45 -p pose_source:=odometry
          -p frame.mode:=align_to_track -p frame.reference_frame:=map)
    truth=truth_map.yaml
    limits=(--max-median-cm 2.0)
    ;;
  ceiling_uneven)
    gen=(--map $ceil/office_uneven/marker_map.yaml --route $ceil/office_uneven/route.csv
         --camera upward --laps 2 --rate 5 --speed 0.5 --map-noise 0.06 --map-noise-yaw-deg 1.0)
    node=(-p dictionary:=tag36h11 -p marker_size:=0.45 -p pose_source:=odometry
          -p frame.mode:=align_to_track -p frame.reference_frame:=map)
    truth=truth_map.yaml
    limits=(--max-dz-cm 2.0 --max-normal-deg 1.0)
    ;;
  *)
    echo "unknown scene $scene"; exit 2;;
esac

rm -rf "$out"
mkdir -p "$out"
ros2 run marker_survey marker_survey_synthetic "$out" "${gen[@]}"
# the node's own executable, not `ros2 run`, so that the signal below reaches it
"$(ros2 pkg prefix marker_survey)/lib/marker_survey/marker_survey" "${common[@]}" "${node[@]}" \
  > "$out/node.log" 2>&1 &
node_pid=$!
trap 'stop $node_pid TERM 10' EXIT
for _ in $(seq 60); do
  ros2 service list 2>/dev/null | grep -q /marker_survey/save && break
  sleep 1
done
timeout 900 ros2 bag play "$out/bag" --clock --delay 3 > "$out/play.log" 2>&1
sleep 2
timeout 600 ros2 service call /marker_survey/save std_srvs/srv/Trigger | tail -2
stop $node_pid
trap - EXIT
grep -E 'solved|aligned|keyframes,' "$out/node.log" | tail -3 || true
if grep -E 'dropped: no camera pose' "$out/node.log"; then
  echo "FAIL: frames dropped for want of a camera pose"; exit 1
fi
ros2 run marker_survey marker_survey_compare "$out/result/marker_map.yaml" "$out/$truth" "${limits[@]}"
