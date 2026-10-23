# Phase 3 validation and technical report

Validation date: 2026-10-23. This report is being completed from the owned live
validation runs; only executed checks and measured values are recorded below.

## Repository audit and baseline

`origin/main` was fetched and verified at `00439c9`, the merge of Phase 2
`6450fdc`. `feature/digital-twin-realtime-sync` has that same committed baseline.
The continuation preserved the existing uncommitted Phase 3 implementation and
its evidence. Phase 2's warehouse adapter, four generated scenarios, original
Blender GLB, camera controls, odometry diagnostics and independent Recorder were
reviewed. No commits or pushes were made.

## Implemented

- **3A:** SceneBroadcaster pose extraction, scene-ID/hierarchy resolution, explicit
  stable identities, simulation timestamps and a bounded dedicated ROS pose topic.
- **3B:** Version 2 REST/WebSocket state, Pydantic/JSON Schema/generated TypeScript,
  independent entity availability and simulation epochs. Version 1 remains intact.
- **3C:** The existing AMR GLB follows measured Gazebo world position/quaternion
  through bounded interpolation. Odometry stays separate in its reported frame.
- **3D:** The original three workers use real model, body and visual transforms.
  Walking and bobbing come directly from Gazebo; no browser gait or route physics.

The [synchronization guide](PHASE_3_SYNCHRONIZATION.md) documents transport,
frame semantics, schema fields, launch commands and resource ownership.

## Source selection and extraction evidence

Gazebo Sim 8.15.0 / ROS 2 Jazzy was investigated in headless mode. The dynamic
pose topic omitted the stationary human and animated visuals; the full pose topic
provided both. Its transforms are local to their parents, not uniformly world
relative. The scene service supplies the matching model/link/visual hierarchy.
The installed Pose_V-to-TFMessage bridge lost the enclosing timestamp and yielded
empty frame names, so a small adapter preserves those explicitly. No ground truth
enters the TF graph, no world-to-odom transform is fabricated, and no second
simulation server or physics engine is introduced.

The initial independent extraction probe matched **132 messages** by exact source
timestamp, with **zero coordinate differences**. Raw pose, scene, bridge and
filtered-message samples are retained in [the audit directory](evidence-phase3/audit/),
including [extraction.json](evidence-phase3/audit/extraction.json).

## Defects found and corrected during validation

1. **Slow-start/restart spawn race.** Two pre-fix runs initialized two controller
   managers and failed activation. The existing create client retries requests
   every five seconds during startup; the failures are consistent with queued
   create retries before world readiness. A bounded read-only scene readiness barrier
   now runs before that client. The successful normal restart log contains one
   controller-manager initialization. Controller configuration and physics remain
   unchanged. See the [create client source](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim/src/create.cpp).
2. **First-load warehouse visibility.** Rendering is deferred until the existing
   scene and GLB are loaded; a real screenshot pixel-coverage assertion checks the
   first frame. The earlier Phase 2 normal screenshot exposed the same defect.
3. **Interpolation under arrival jitter.** A monotonic per-entity render cursor
   prevents backward playback within an epoch; epochs/frames/sessions reset it.
4. **Evidence capture consistency.** Root, limb and render-time diagnostics are
   read atomically. Separate browser reads could cross render updates and create
   a false numerical mismatch. Pause assertions compare physical transforms while
   allowing availability to age to stale/offline.
5. **Validation cleanup.** The harness retains observations on failure, joins its
   observer and detects a failed simulator launch promptly. Motion probes send
   stamped zero commands in cleanup.

## Automated and live results

Final scenario, numerical, performance and storage results are populated after the
current four-scenario run and regression runs finish. Failed exploratory runs are
retained separately and are not acceptance evidence.

## Measurement method and limits

An independent Gazebo Transport subscriber records original source stamps,
positions, quaternions and local human visuals. WebSocket and browser records are
matched to that source time within the same simulation epoch. Rendered values are
compared with the position interpolation and shortest-arc quaternion SLERP of the
actual bracketing source samples. This measures preservation/interpolation error,
not zero-latency tracking of the latest physical pose.

Latency uses wall timestamps on this one host, from independent Gazebo message
receipt to browser message receipt or the completed rendered diagnostics frame.
It excludes time before the Gazebo observer receives a message and does not measure
physical display scanout. The intentional interpolation target is 120 ms behind
arrival; measured delay additionally includes scheduling and rendering. FPS is
measured while capturing real browser evidence. Memory numbers are peak RSS per
process name, not summed memory or a long-duration leak certification.

## Scope limits and Phase 4 readiness

The four current scenarios have static obstacle models; no moving obstacle is
invented. The entity schema and transform registry support registering future
movable obstacles. Dynamic cast shadows are omitted from the cached static shadow
map to avoid stationary ghost shadows. Workers and the AMR still receive lighting.

A Gazebo full world reset removes the AMR inserted at runtime. Its last pose is
retained with unavailable status; restarting the simulation launch restores it.
Automatic respawn and controller lifecycle management after a full reset are not
implemented. Pause/stall status describes observed clock progress, not the cause
of a stall. Exact visual transforms are synchronized at the sampled/interpolated
times; the browser does not reproduce unsampled physics steps.

Deferred by scope: SLAM, Nav2, localization transforms, autonomous missions,
planning, fleets, dynamic-state recording and historical 3D replay. Phase 4 must
add localization explicitly; simulation ground truth is not a localization estimate.
