# Phase 4 navigation

The `amr_navigation` package adds ROS 2 Jazzy SLAM Toolbox, saved-map AMCL and
Nav2 to the existing Harmonic/ros2_control stack. Ground truth remains exclusively
in the existing visualization pipeline and evaluation; it never enters localization.

## Baseline and environment

Created `feature/amr-autonomous-navigation` from fetched `origin/main` `fafd145`
(Phase 3 merge). Working tree was clean. The user later committed the initial Phase 4 work as `855d4d3`;
subsequent fixes remain uncommitted. No assistant commits or pushes. Ubuntu 24.04,
ROS 2 Jazzy, Nav2 1.3.13, SLAM Toolbox 2.8.5; packages were already installed.
Full Debian versions and runtime evidence live in `evidence-phase4/` (ignored
runtime artifacts, consistent with earlier phases).

## Frames and sensors

`map → odom` belongs to SLAM **or** AMCL. `odom → base_link` belongs to the
existing differential-drive controller. The URDF publishes
`base_link → chassis_link → lidar_link → laser_frame`. The laser is at
(0.44, 0, 0.213) m relative to the base. `/scan` is best-effort/volatile,
361 samples, ±π/2, 0.08–20 m, approximately 15 Hz of simulation time.
`scan_time` is zero in the Gazebo bridge message; use header timestamps to measure
frequency. Odometry is 50 Hz; IMU and joints are 100 Hz. Everything uses `/clock`.
`world` is independent of `map`; warehouse camera transforms remain in `world`.

## Mapping

Build and source the workspace. Use the same ROS domain and Gazebo partition in
all terminals. Choose an unused domain; do not launch a second simulation.

```bash
source install/setup.bash
export ROS_DOMAIN_ID=44 GZ_PARTITION=warehouse_mapping
ros2 launch amr_simulation warehouse.launch.py gui:=false mapping_only:=true
# Another terminal, same environment:
ros2 launch amr_navigation mapping.launch.py
ros2 run amr_navigation sensor_audit --output /tmp/sensors.json
```

`mapping_only` derives a temporary SDF from the checksum-verified normal world,
omitting only the three worker models. All static geometry, collisions, sensors
and the original AMR/controller remain intact. Production scenarios retain workers.
This controlled survey avoids baking moving or stationary people into the saved map.
The existing warehouse generator is the sole geometry authority; no occupancy grid
is generated from its JSON or SDF.

Survey with supervised motion while watching real scan/TF/map in RViz. The optional
`mapping_jog.py` performs **one** operator-selected turn and forward displacement,
with SLAM-estimated pose feedback, a LiDAR stop, exclusive command ownership and a wall timeout.
It is a mapping aid, not an autonomous navigator. It refuses to run without SLAM.
Do not use it during Nav2 navigation. Its forward-sector check is not a full safety
certification; inspect surrounding clearance before rotating.

```bash
python3 src/amr_navigation/scripts/mapping_jog.py --distance 2 --output /tmp/survey.json
ros2 run rviz2 rviz2 -d src/amr_navigation/rviz/mapping.rviz --ros-args -p use_sim_time:=true
src/amr_navigation/scripts/save_map.sh "$PWD/src/amr_navigation/maps/warehouse"
python3 src/amr_navigation/scripts/inspect_map.py src/amr_navigation/maps/warehouse.yaml --output /tmp/map.json
```

The saved YAML/PGM is an occupancy grid for map_server/AMCL. `.posegraph` and
`.data` are SLAM Toolbox artifacts for continued mapping. Neither is a TF tree or
Gazebo world. Version the reviewed YAML/PGM and provenance together; regenerate
when static geometry or sensor mounting changes. Retain serialized graphs locally
unless continued mapping is required. Never fabricate or paint unknown pixels free.

## Sources

Installed Jazzy parameter files and plugin manifests are the primary compatibility
reference. See [Nav2 Jazzy controller parameters](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/controller_server/),
[Nav2 Jazzy velocity interface](https://api.nav2.org/nav2-jazzy/html/md_nav2_util_README.html),
and [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox).

## Saved-map localization and navigation startup

Stop mapping before starting AMCL. Both standalone modes check the discovered ROS
node graph and reject an already-running localization/navigation mode. This check
prevents sequential accidental launches; operators must still use one owner per
ROS domain and not launch two modes simultaneously.

```bash
ros2 launch amr_navigation localization.launch.py map:=/absolute/path/warehouse.yaml
# Explicit estimate in map coordinates, or use RViz's 2D Pose Estimate tool:
ros2 run amr_navigation initial_pose 0 0 0
```

AMCL uses a differential motion model and likelihood-field sensor model, 500–2500
particles, 90 selected LiDAR beams, and the real wheel odometry. Its initial pose
is not silently copied from Gazebo. `initial_pose` checks the selected map cell,
publishes a covariance-bearing estimate and waits for AMCL pose plus connected TF.
A free cell alone does not prove that an estimate is correct; verify convergence.

Stop standalone localization before starting the complete navigation launch:

```bash
# Attach to an existing simulation:
ros2 launch amr_navigation navigation.launch.py
ros2 run amr_navigation initial_pose X Y YAW
# Or start ONE complete existing simulation + Gateway + navigation stack:
ros2 launch amr_navigation autonomous_demo.launch.py scenario:=normal \
  initialize:=true initial_x:=0 initial_y:=0 initial_yaw:=0
# Frontend, separate terminal:
npm --prefix web/digital-twin run dev
# A bounded real Nav2 action, map coordinates:
ros2 run amr_navigation navigate X Y YAW --timeout 240 --output /tmp/goal.json
```

Do not run the attach and full-demo commands together. Set the same `ROS_DOMAIN_ID`
and `GZ_PARTITION` for observers. `initialize` defaults false; use the measured
initial map pose from map provenance, not warehouse station coordinates. No
`world → map` TF is broadcast. Evaluation registration remains an offline artifact.

## Nav2 choices and command ownership

NavFn A* plans over known free space; Regulated Pure Pursuit follows with
footprint collision checking and cost/curvature speed regulation. Navigation speed
is 0.45 m/s, rotation 0.35 rad/s. The original differential-drive controller still
limits velocity to 0.70 m/s / 1.50 rad/s and acceleration to 0.50 m/s² / 1.50 rad/s².
No velocity smoother is necessary because that controller already limits acceleration.

The Jazzy controller and behavior servers explicitly enable stamped commands,
remapped to `/nav_cmd_vel`. They are sequenced by the standard Nav2 behavior tree.
The `navigation_command_guard` is the sole `/cmd_vel` publisher. It forwards those
`TwistStamped` messages only with fresh scan, odometry, advancing clock, AMCL
covariance and valid TF. Its wall-clock timer outputs zeros during stale input or
clock pauses. It never plans paths or synthesizes navigation velocities. A crashed
guard leaves the original controller's 0.5 s simulation-time watchdog in charge.
Real fault testing found that stopping the ROS clock bridge freezes the controller
update loop while Gazebo can continue integrating its last wheel velocity. A zero
ROS command cannot stop that failure. The guard therefore latches motion disabled
and requests a native `/world/warehouse/control` emergency pause on clock loss
during motion or a backwards clock jump. It uses the existing **GZ_PARTITION**;
that variable is required when attaching navigation. The full demo sets it once
for all children. No second clock or velocity publisher is introduced. This pause
is a fault response, never a way to pass an obstacle test. The guard never unpauses
automatically: stop/restart the owned stack, reinitialize AMCL, then issue a new
goal. A paused controller's stale velocity sample is not proof of a physical stop;
fault tests check independent world-pose stationarity and native Gazebo statistics.

The installed standard replanning/recovery tree is adapted only to bound retries
to three and omit `BackUp` because rear approaches are unobserved by the 180° scan.
Spin and wait use Nav2's collision-aware behavior server. Cancellation is an action
request, never a competing manual velocity publisher. The action client has a wall
timeout and cancels before cleanup. RViz goals should also be cancelled explicitly
when testing; the bounded client is the recommended repeatable test interface.

## Footprint and costmaps

The chassis and bumpers fit x ±0.475 m, y ±0.35 m. The camera collision box reaches
x=0.494 m. Both costmaps use corners (0.50, ±0.35), (−0.48, ±0.35) with 0.04 m
padding. This includes the protrusion instead of shrinking the robot to fit aisles.
Inflation is 0.8 m with factor 3; warehouse rack aisles are nominally 3 m wide.

Global map frame is `map`; rolling 6×6 m local costmap frame is `odom`. Both use
static, LiDAR obstacle and inflation layers at 5 cm resolution. Global updates
are 2 Hz; local updates 5 Hz. Laser observations mark to 8 m and clear to 10 m,
using infinite returns for clearing and no artificial observation persistence.
Unobserved areas stay unknown. There is no Gazebo-human-position obstacle injection.
The global live obstacle layer lets periodic replanning account for observed
barriers/workers; RPP can stop for an obstacle rather than steer blindly around it.

## Read-only Digital Twin diagnostics

The existing Gateway adds `/api/v1/navigation`, with independent
`navigation_status` protocol version `1.0` and
`shared/schemas/navigation-status.v1.schema.json`. It consumes only two named ROS
DiagnosticArray entries from `/navigation/diagnostics`. The new browser section
polls this endpoint and reports stale/unavailable status. The action-client entry
reports its selected goal, feedback and terminal result; RViz-created goals do not
supply that client-specific record. WebSocket v1/v2 and simulation pose rendering
are unchanged. No browser command interface or second FastAPI app is added.

## Repeatable validation

```bash
ros2 run amr_navigation sensor_audit --output /tmp/sensors.json
python3 -m pytest -q src/amr_navigation/test
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -p pytest_asyncio.plugin -q src/digital_twin_gateway/test
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q src/digital_twin_recorder/test src/amr_simulation/tests
npm --prefix web/digital-twin test
npm --prefix web/digital-twin run build
# Stop other instances first; ports 8000/5173 must be free for the owned harness:
python3 src/amr_navigation/scripts/integration.py --initial X Y YAW \
  --goal GX GY GYAW --scenario normal --browser --output /tmp/nav-evidence
```

`integration.py` starts one existing simulator/Gateway, audits sensors, sends real
NavigateToPose actions, captures path/estimated pose/odometry/world trajectory and
optional browser screenshots/video, and stops its owned process groups. Failed
results and logs survive cleanup. `goal_client` never publishes velocity commands.
Metrics label estimated (`map`), wheel-derived (`odom`), and ground-truth (`world`)
travel separately. Path publication count is reported separately from replans;
recovery count comes from Nav2 feedback. Collision count remains null unless an
independent contact observer supplies evidence; absence of a crash is not a zero
collision measurement.

## Current saved map and measured runs

`maps/warehouse.yaml`, `warehouse.pgm`, and `provenance.json` are real SLAM
artifacts. The 729×491 grid uses 0.05 m cells and origin (−10.196, −10.052, 0).
Its validated initial map pose is (0, 0, 0) at the unchanged world spawn (−11, 0, 0).
The +11 m x registration is used only for evaluation. The central cross aisle is
covered; rack aisles and perimeter remain partly unknown. See the validation
report before using a goal outside demonstrated coverage.

The real browser run reached map goals (2, 0, 0), (5, 1, π/2), and (10, −1, 0).
For a repeat, pass these explicit map poses to `integration.py`. The bounded
client normally rejects a goal whose conservative footprint overlaps unknown or
occupied map cells. `--server-validation` is an acceptance-test option that sends
a finite goal directly to Nav2 to exercise its own rejection; all Nav2 costmap
and collision checks remain enabled.

```bash
python3 src/amr_navigation/scripts/integration.py --initial 0 0 0 \
  --goal 2 0 0 --goal 5 1 1.57079632679 --goal 10 -1 0 \
  --scenario normal --browser --output /tmp/phase4-normal
python3 src/amr_navigation/scripts/fault_suite.py --output /tmp/phase4-faults
python3 src/amr_navigation/scripts/analyze_results.py /tmp/phase4-normal/goal-0.json \
  --output /tmp/phase4-normal/goal-0.metrics.json
```

The fault suite owns a fresh stack for each case, uses wall deadlines and cancels
or shuts down all owned processes. Contact evidence covers only the existing
worker, south-wall and scenario-barrier sensors, not every warehouse collision.
The obstacle observer compares LiDAR/costmap observations with independent worker
poses solely for evaluation; it publishes nothing.

To continue mapping, retain the serialized graph alongside the occupancy map.
The supported SLAM `map_file_name` and `map_start_pose` parameters may be supplied
in a copy of `slam_params.yaml` via `mapping.launch.py slam_params_file:=...`.
Use the verified initial map pose after a fresh simulation restart, and validate
registration again before replacing a reviewed map. Do not run AMCL concurrently.
