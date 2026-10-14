# Phase 2 rendering boundary

Three.js is installed; no renderer, WebGL context, physics engine, warehouse or
robot viewer is created in Phase 1. A future renderer subscribes to the validated
TwinStore and loads geometry derived from the existing warehouse_layout.json,
amr_dimensions.json and station_poses.json. It must explicitly resolve odom/world
registration and ROS Z-up / Three.js display axes. It must not infer global
localization or make navigation decisions. No duplicate layout is stored here.
