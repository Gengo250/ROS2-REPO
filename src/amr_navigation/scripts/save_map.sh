#!/usr/bin/env bash
# Run while mapping is active. Occupancy image and serialized graph are distinct.
set -euo pipefail
map_base=${1:?Usage: save_map.sh /absolute/path/warehouse}
[[ "$map_base" = /* ]] || { echo 'Use an absolute output path' >&2; exit 2; }
mkdir -p "$(dirname "$map_base")"
timeout 40 ros2 run nav2_map_server map_saver_cli -f "$map_base" --ros-args \
  -p use_sim_time:=true -p map_subscribe_transient_local:=true -p save_map_timeout:=20.0
# JSON is valid YAML and safely quotes filenames for the ROS CLI.
request=$(python3 -c 'import json,sys; print(json.dumps({"filename":sys.argv[1]}))' "$map_base")
timeout 40 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "$request"
