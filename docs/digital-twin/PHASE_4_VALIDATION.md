# Phase 4 validation record

This file reports executed checks, including failures. It is not an acceptance
claim until the real Nav2/physics/browser tests have finished.

## 4A — validated environment and sensor prerequisites

- Clean baseline `fafd145`, fetched before branching; no commits or pushes.
- Ubuntu 24.04 / Jazzy. Nav2 1.3.13, SLAM Toolbox 2.8.5 already installed.
- `evidence-phase4/dependencies.txt` records exact Debian versions.
- `sensors-4a.json`: 93 real scans over 15 wall seconds, 15.154 Hz simulation
  frequency; 361 beams, ±π/2, frame `laser_frame`, 183 finite returns in the
  stationary initial view. Ranges 0.08–20 m. Sensor QoS best effort/volatile.
- Odometry 50 Hz, IMU 100 Hz, joints approximately 100 Hz, clock 1000 Hz,
  monotonic observed timestamps. Measured real-time factor 0.452.
- TF resolves from odom through the original URDF to the laser. Laser translation
  is (0.44, 0, 0.213) m. Original controller is the sole `/cmd_vel` subscriber,
  type `geometry_msgs/msg/TwistStamped`; no publishers before mapping.

## 4B — mapping development and rejected survey

SLAM activated using the existing sensors/TF. `initial_map.yaml/pgm` was saved by
Nav2 map_saver from a real 330×267 occupancy grid at 0.05 m/pixel. The robot
physically traversed 9.96 m on the first supervised survey leg. Worker-free
mapping derives from the checksum-verified normal SDF; production scenarios retain
all workers and their original HumanSystem motion.

The first long survey was **rejected**: `rejected-map.pgm`, `mapping-east-rviz.png`
and `registration-after-turn.json` show distorted walls after a poorly matched
turn. Earlier position differences were 0.27–0.37 m; the failed turn reached
2.27 m and 0.42 rad under initial registration. None of these artifacts is the
navigation map. Survey safety stops and failed launch/test logs are retained.

The retry increases angular scan density, keeps a larger scan buffer, disables
scan barycenter matching for the partial field of view and slows supervised turns.
This is a mapping configuration adjustment; physics, collisions, sensors and
controller limits have not changed.

## Regression checks so far

- Gateway: **44 passed**, including v1/v2 and asynchronous transport tests.
- Recorder and warehouse: **43 passed, 1 skipped** (optional integration).
- Navigation mapping/launch checks: **11 passed**.
- Frontend: **13 passed**; production build passed (existing large-chunk warning).
- Combined Python collection initially failed because ROS's automatic launch-test
  plugin imports same-named test modules and Gateway tests import `conftest`
  directly. Reproducible commands run packages separately, disable automatic
  plugin loading, and explicitly enable pytest_asyncio for Gateway.
- An existing warehouse launch test expected direct AMR spawn; Phase 3 moved it
  behind a Gazebo readiness callback. The test now exercises readiness success,
  readiness failure, spawn failure and controller failure.

The revised survey's northern pass also accumulated non-rigid distortion before
loop closure (approximately 1.52 m at one sampled stop). This remains a mapping
quality issue, not a Gazebo pose synchronization defect. Scan matching settings
and explicit slow turn commands were verified in the active parameter dump.
The converted SDF sensor pose matches the ROS laser extrinsics exactly. These
intermediate maps must not be represented as a validated full-warehouse map.

The added navigation extension has its own versioned schema and REST route.
Gateway regression after the extension: **45 passed**. Browser production build
also passes. The entire ROS workspace built successfully (7 packages); setuptools
printed existing packaging warnings, without compilation failures.

## Mapping acceptance boundary

The long-loop v2 survey failed to close accurately: the final measured difference
was 3.79 m / 0.21 rad. It is retained as `rejected-map-v2.*`. The subsequent
controlled survey refines scan-matcher angular resolution and records a central
corridor before extending coverage. Its first straight-pass comparison at 8.54 m
was 0.009 m / 0.006 rad. This is a local measurement, not proof of whole-map accuracy.

## Storage regression

The existing native Mosaico services started successfully. A real odometry/IMU
recording completed as `digital_twin_normal_20261023T223712Z_3c6116f72715`.
Independent SDK read-back verified topic integrity and finalization; stored size
60,991 bytes. See `mosaico-readback.json`. No database, SDK, Python interpreter or
Recorder implementation was replaced.
