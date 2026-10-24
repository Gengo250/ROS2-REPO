#!/usr/bin/env bash
# Read-only snapshot, to be invoked during a real survey.
set -euo pipefail
output=${1:?Output directory required}
mkdir -p "$output"
timeout 30 ros2 run nav2_map_server map_saver_cli -f "$output/map" --ros-args \
  -p use_sim_time:=true -p map_subscribe_transient_local:=true -p save_map_timeout:=15.0
(cd "$output" && timeout 20 ros2 run tf2_tools view_frames --ros-args -p use_sim_time:=true)
