#!/bin/bash
# Render a synthetic recording, play it through the localizer node, record its
# fixes and score them against the true poses. Run from the repository root in
# a sourced ROS 2 workspace where marker_survey and marker_localizer are built:
#
#   ci/synthetic_localize.sh ceiling [OUT_DIR]
#
# Scenes: ceiling (planar, office), ceiling_gate (the same, localized against a
# map in which marker 5 has moved by 1.5 m), wall (3d, test course).
# Exits non-zero when the score fails: errors over the limits, or a fix whose
# stamp is not its image's capture time.
set -eo pipefail
scene=${1:?scene name}
out=${2:-tmp/ci}/loc_$scene
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-38}
sites=examples/sites
common=(--ros-args -p use_sim_time:=true -p process_all:=true -p prior_topic:=/prior_pose)
office=(--map $sites/office/marker_map.yaml --route $sites/office/route.csv
        --camera upward --laps 2 --rate 5 --speed 0.5 --map-noise 0.05 --map-noise-yaw-deg 1.0)

rm -rf "$out"
mkdir -p "$out"
case $scene in
  ceiling)
    gen=("${office[@]}")
    node=(-p marker_map:=$PWD/$sites/office/marker_map.yaml -p mode:=planar)
    limits=(--mode planar --max-mean-cm ${CEILING_MEAN_CM:-1.0} --max-cm ${CEILING_MAX_CM:-3.0})
    ;;
  ceiling_gate)
    gen=("${office[@]}")
    sed 's/{id: 5, x:  0.000/{id: 5, x:  1.500/' $sites/office/marker_map.yaml > "$out/moved_map.yaml"
    grep -q 'id: 5, x:  1.500' "$out/moved_map.yaml"
    node=(-p marker_map:=$(cd "$out" && pwd)/moved_map.yaml -p mode:=planar)
    limits=(--mode planar --max-cm ${CEILING_MAX_CM:-3.0})
    ;;
  wall)
    gen=(--map $sites/test_course/marker_map.yaml --route $sites/test_course/route.csv
         --camera forward --laps 1 --sweeps 4 --rate 3 --map-noise 0.05 --map-noise-yaw-deg 1.0)
    node=(-p marker_map:=$PWD/$sites/test_course/marker_map.yaml -p mode:=3d)
    limits=(--mode 3d --max-mean-cm ${WALL_MEAN_CM:-2.0} --max-cm ${WALL_MAX_CM:-10.0})
    ;;
  *)
    echo "unknown scene $scene"; exit 2;;
esac

ros2 run marker_survey marker_survey_synthetic "$out" "${gen[@]}"
"$(ros2 pkg prefix marker_localizer)/lib/marker_localizer/marker_localizer" "${common[@]}" "${node[@]}" \
  > "$out/node.log" 2>&1 &
node_pid=$!
ros2 bag record -s sqlite3 -o "$out/poses" /marker_localizer/pose > "$out/record.log" 2>&1 &
rec_pid=$!
trap 'kill -TERM $rec_pid 2>/dev/null || true; kill -TERM $node_pid 2>/dev/null || true' EXIT
for _ in $(seq 60); do
  ros2 topic info /marker_localizer/pose 2>/dev/null | grep -q 'Subscription count: [1-9]' && break
  sleep 1
done
ros2 bag play "$out/bag" --clock --delay 3 > "$out/play.log" 2>&1
sleep 3
kill -TERM $rec_pid 2>/dev/null || true
wait $rec_pid || true
kill -TERM $node_pid 2>/dev/null || true
wait $node_pid || true
trap - EXIT
grep -E 'images,|gated' "$out/node.log" | tail -4 || true
if [ "$scene" = ceiling_gate ]; then
  n=$(grep -c 'gated' "$out/node.log" || true)
  echo "gate messages in the log: $n"
  [ "$n" -gt 0 ]
fi
ros2 run marker_localizer marker_localizer_score "$out/poses" "$out/truth_poses.csv" "${limits[@]}" \
  --min-fixes 100
