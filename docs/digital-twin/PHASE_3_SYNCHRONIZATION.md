# Phase 3: real-time simulation synchronization

The active Three.js scene follows Gazebo world ground truth. ROS odometry remains
an independent diagnostic in its original `odom` frame. No localization transform,
physics, trajectory, gait clock, teleoperation, or navigation stack is added.

## Baseline and scope

Work starts at `00439c9` (current `origin/main` re-fetched and verified on 2026-10-23), the merge
of Phase 2 `6450fdc`. The initial working tree was clean. Local main was fast
forwarded and `feature/digital-twin-realtime-sync` created from that merge. No
commits or pushes are part of this work. Original warehouse generation, controller,
GLB, scene API and Recorder remain the sources of geometry/control/storage.

## 3A: verified source and ROS transport

Installed Gazebo Sim is **8.15.0** (Harmonic), ROS Jazzy, Gazebo Transport 13 and
Messages 10 through the Jazzy vendor packages. Investigation artifacts are in
`evidence-phase3/audit/`.

`/world/warehouse/dynamic_pose/info` contains `warehouse_amr`, `human_01`,
`human_02`, duplicate `body` link names and AMR links. It omits static `human_03`
and animated visuals. `/world/warehouse/pose/info` contains all of those,
including static models and current animated visual components. Both are
`gz.msgs.Pose_V`. The enclosing header contains simulation time; individual poses
carry names and temporary entity IDs, but no frame metadata.

The [SceneBroadcaster source](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/scene_broadcaster/SceneBroadcaster.cc)
reads each entity's local `components::Pose`. Top-level model poses are relative
to world; link poses are relative to their model; visual poses are relative to
their link. The `/world/warehouse/scene/info` service returns that hierarchy with
matching IDs. The live samples confirm world spawn X=-11 m for the AMR,
X=11.5 m for human_03, and local zero body transforms. Human limbs have changing
local poses. Do not interpret the duplicate `body` poses as world positions.

The installed `ros_gz_bridge` Pose_V → TFMessage mapping was exercised on a
**temporary dedicated investigation topic**, never `/tf`. It preserved numeric
translations/quaternions but yielded empty parent/child frames and zero stamps.
The [conversion implementation](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/src/convert/tf2_msgs.cpp)
converts individual poses rather than propagating the enclosing timestamp.
Directly bridging these messages is therefore unsuitable.

The selected narrow Python adapter (`ros/simulation_adapter.py`) reuses
SceneBroadcaster; no new Gazebo system/plugin or physics process is needed.
PosePublisher was not installed/configured because the existing source supplies
both physical and exact visual poses once identities are resolved. The adapter:

1. Queries the scene hierarchy at startup and every two seconds, with a 200 ms
   timeout, in its own process. No Gateway request/callback performs these queries.
2. Joins poses to the hierarchy by Gazebo ID and validates the name, then emits
   stable scoped names: `human_01`, `human_01/body`,
   `human_01/body/leg_1_visual`, etc. Duplicate scoped names are rejected.
3. Filters to configured top-level models and human link/visual descendants.
   `warehouse_amr` explicitly maps to `amr_01`; workers map to their existing IDs.
4. Propagates the enclosing simulation stamp onto every TransformStamped.
5. Publishes the latest sample at at most 20 Hz wall time on
   `/digital_twin/simulation/poses`, using depth-one sensor-data QoS. It does not
   replay old samples when Gazebo stops publishing.

Current payload: four model transforms plus 33 human link/visual transforms.
Parent transforms remain explicit; the browser composes the actual hierarchy.
No transforms enter `/tf` or `/tf_static`. `odom → base_link` remains untouched.
An independent observer matched 132 adapter messages to exact Gazebo timestamps,
with zero numeric position/quaternion differences. See `audit/extraction.json`.

The original full pose stream is relatively large (~63 kB per sample in the
investigation); only the filtered transforms cross ROS and WebSocket boundaries.
The adapter owns one Gazebo transport node and one ROS executor. The Gateway
continues to own its original single executor thread.

## 3B: protocol and Gateway

Existing `/api/v1/state`, `/ws/v1/twin`, health/status/scene routes and v1 schemas
remain available. v1 humans/obstacles/stations remain empty reserved collections.
New endpoints are `/api/v2/state` and `/ws/v2/twin?protocol_version=2.0`.

`TwinStateV2` contains:

| Field | Meaning |
| --- | --- |
| `robots` | Existing odometry diagnostics, including source frame, velocity and receipt |
| `simulation_robots` | World ground truth for explicitly mapped robot IDs |
| `humans` | Actual physical poses plus parent-relative visual/body components |
| `obstacles` | Tracked ground truth for configured obstacle identities; empty today |
| `simulation.epoch` | Counter incremented on backward simulation time, resetting interpolation |
| `simulation.status` | running / paused_or_stalled / offline / unknown |
| `session_id`, `sequence` | Existing Gateway lifecycle/order guarantees |

Each dynamic entity has stable ID, Gazebo name, type, frame, pose (nullable before
receipt), nullable velocity, source simulation timestamp, UTC receipt timestamp,
monotonic age, availability and pose topic. Components have scoped ID, parent frame
and pose. Velocity is **null**: SceneBroadcaster does not measure velocity.
Pydantic rejects extra fields/nonfinite numbers; collection normalization rejects
mixed timestamps, duplicate frames, broken hierarchy, non-world roots and invalid
unit quaternions. There is a 64-component per-entity limit and 256-transform input
limit. Configured IDs bound the latest-value store.

Freshness advances only when an entity's source timestamp advances. Repeated
frozen poses cannot make an entity appear live indefinitely. Defaults remain
2 s stale / 10 s offline. Missing entities age independently; configured entities
with no sample are unknown, then offline. The clock status distinguishes observed
progress from stalled/unavailable data, **not** the cause of the stall.

The publisher builds v1 and v2 snapshots from the same locked state at ~10 Hz.
Each version has the existing depth-one WebSocket fanout and bounded send timeout.
Static scene data, camera frames, clouds, configuration and storage never enter
these snapshots. Schema export generates `shared/schemas/twin-state-v2.schema.json`;
TypeScript is generated from that schema and validated with strict AJV on receipt.

## 3C/3D: renderer and animation

`StateSynchronizer` updates existing Three.js object position and quaternion.
Its per-entity buffer holds at most 20 samples. It rejects older Gateway sequence
numbers, clears on session/epoch/frame changes, interpolates positions and SLERPs
quaternions at a target 120 ms behind arrival using observed simulation-time rate.
The target clamps to the newest actual measurement: there is **no extrapolation**.
Repeated timestamps add no samples. A monotonic render cursor prevents network-arrival jitter from rewinding playback within an epoch. A stopped source reaches the final observed
pose after the short interpolation drain, then freezes. Stale/missing entities and
disconnections stop interpolation. No trajectories or velocities are invented.

The AMR GLB loads once per scene load, preserving its original scale and single
Y-up → Z-up conversion. Odometry never positions the GLB. Scenario previews that
differ from the running scenario keep their configured initial geometry.

`HumanVisuals` separates human primitives from the static batches, creates the
original model → body → visual transform nodes, and updates existing instance
matrices. All original dimensions, colours and primitive geometry remain. There
are exactly three human roots and 30 visual instances, with no static duplicate.
The limbs, shoes, torso and bobbing use **actual HumanSystem visual poses**, not an
approximation. The renderer does not read waypoint routes or gait parameters.

Static InstancedMesh batching remains in place; the complete scene still has
50 geometries and 51 draw calls in normal/person-crossing scenarios. Moving humans
and the AMR receive lighting but do not cast into the cached static shadow map,
avoiding stationary ghost silhouettes without redrawing every static shadow each
frame. Dynamic cast shadows are a documented visual limitation.

The first-load renderer now waits for the scene and original GLB before rendering.
Live validation exposed a pre-existing first-load defect where the warehouse's
instanced geometry could be invisible even while draw-call counts looked valid.
The old Phase 2 normal screenshot also shows this. A browser pixel-coverage gate
now checks that the warehouse is actually visible on first load.

## Obstacles and reset behavior

The four existing scenarios' pallets, cones, barrier, carts and other equipment
are SDF-static models. No independent dynamic obstacle is present and no movement
is fabricated. The entity contract/registry accepts obstacle identities and real
poses for future use; adding movable geometry requires explicitly registering its
scene representation, not a new transport or physics design.

Gazebo's full reset removes the AMR created at runtime by the existing launch.
The Gateway consequently marks it unavailable and the renderer freezes its last
known pose; it does not silently respawn it. Human entities belonging to the world
reset and continue. Restarting the existing simulation launch restores the AMR.
The AMR launch now waits for a successful read-only scene response before starting the existing `ros_gz_sim create` client. This avoids queuing create retries during slow startup; no controller configuration or physics was changed. The integration harness keeps Gateway and adapter running through simulator
shutdown/restart to verify disappearance, epoch changes and reappearance.

## Run and reproduce

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
colcon build --symlink-install
source install/setup.bash
ros2 launch digital_twin_gateway digital_twin.launch.py scenario:=normal
# Separate frontend terminal:
export PATH="$PWD/.tools/node_modules/.bin:$PATH"
npm --prefix web/digital-twin run dev
```

Open `http://127.0.0.1:5173`. The composed launch starts one adapter alongside the
existing warehouse and Gateway. For separately launched processes, use the same
explicit `GZ_PARTITION` for the warehouse and adapter, and the same ROS domain for
all ROS processes. Do not also start the composed launch.

Additional environment settings: `TWIN_SIMULATION_POSE_TOPIC` (default above),
`TWIN_SIMULATION_ENTITIES` (JSON Gazebo-name → application-ID mapping). Topic
configuration explicitly rejects using `/tf` or `/tf_static` for ground truth.
Jazzy `gz_transport_vendor` and `gz_msgs_vendor` supply Python Gazebo bindings.

```bash
python src/digital_twin_gateway/scripts/export_schemas.py --check
npm --prefix web/digital-twin run build
npm --prefix web/digital-twin test
colcon test --event-handlers console_direct+
colcon test-result --verbose
# Ports 8000 and 5173 must be free. Owns and tears down its processes:
python src/digital_twin_gateway/scripts/validate_phase3.py
python src/digital_twin_gateway/scripts/analyze_phase3.py docs/digital-twin/evidence-phase3/<run>
# Optional hardware comparison on a graphical desktop:
python src/digital_twin_gateway/scripts/validate_phase3.py --scenarios normal --headed
# Independent Phase 2 recording/retrieval/failure regression:
python src/digital_twin_recorder/scripts/validate_phase2.py
```

Evidence is timestamped under `docs/digital-twin/evidence-phase3/`. Every motion
probe sends stamped zero commands in cleanup. Mosaico remains an independent
Recorder process; dynamic entity persistence is intentionally not added.
See [validation results](PHASE_3_VALIDATION.md) for executed checks and limits.
