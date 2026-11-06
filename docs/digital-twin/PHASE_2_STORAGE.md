# Telemetry persistence with Mosaico (Phase 2, extended for the lite runtime)

The independent Recorder subscribes to real ROS 2 topics of the running simulation and persists
them in [Mosaico](https://github.com/mosaico-labs/mosaico) (`mosaicod` + PostgreSQL + object store)
through the official Python SDK. ROS callbacks only convert, decimate, hash and enqueue; a separate
process owns the SDK, Arrow transport, finalization and read-back verification. Neither the
simulator, Nav2, the Gateway nor the browser depends on Mosaico: they keep running when it is down.

Phase 2 recorded `/odom` and `/imu/data` of the Gazebo warehouse. Since 2026-11-06 the Recorder
records the lite ATLAS campus (world state of every AMR, worker and forklift, events, obstacles,
navigation status, and optionally plans, TF and both LiDARs), under configurable profiles, with
execution metadata of the running system, and Mosaico runs from Docker Compose with persistent
named volumes. The Gazebo path is kept (`--simulation-backend gazebo`).

```text
ROS 2 graph (lite engine, Nav2, AMCL, guard, robot_state_publisher)
   |  rclpy subscriptions, QoS matched to each publisher
   v
Digital Twin Recorder, ROS process          convert -> decimate (sim time) -> SHA-256 -> bounded queue
   |  multiprocessing queue (2048 records)   never blocks a callback; full queue = failed execution
   v
Storage worker process (Mosaico SDK)        one sequence per execution, one topic per ROS topic
   |  Arrow Flight / gRPC (TLS + API key optional)
   v
mosaicod 0.7.0-rc ---> PostgreSQL 16.15     catalog: sequences, topics, metadata (volume `metadata`)
              \------> object store         telemetry chunks (Parquet) (volume `objects`)
```

## Recording inventory

Verified on the running `lite_navigation.launch.py` stack with
`src/digital_twin_recorder/scripts/inspect_topics.py` (types, publisher QoS, measured rate and size):

| Topic | Type | Publisher QoS | Rate (measured) | Profile | Required | Stored |
|---|---|---|---|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | reliable, volatile | 50 Hz | core | yes | every message (`--odom-hz`) |
| `/imu/data` | `sensor_msgs/msg/Imu` | best effort | 50 Hz | core | yes | every message (`--imu-hz`) |
| `/lite_sim/world_state` | `amr_lite_interfaces/msg/WorldState` | best effort | 30 Hz, ~3 KB | core (lite) | yes | ≤ 10 Hz (`--world-state-hz`, 0 = 30 Hz) |
| `/lite_sim/events` | `amr_lite_interfaces/msg/SimEvent` | reliable | on event | core (lite) | no | every event |
| `/lite_sim/obstacles` | `amr_lite_interfaces/msg/ObstacleArray` | reliable, transient local | on change | core (lite) | yes | every change |
| `/navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray` | reliable, transient local | on change | core | no | every change |
| `/navigation/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | reliable | 1 Hz / on change | core | no | every message |
| `/plan` | `nav_msgs/msg/Path` | reliable | ~1 Hz, ~44 KB | extended | no | every plan (`--plan-hz`) |
| `/tf` | `tf2_msgs/msg/TFMessage` | reliable (3 publishers) | 77 Hz | extended | no | only `map -> odom` (AMCL) by default (`--tf-pair`) |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | reliable, transient local | once | extended | no | every message |
| `/scan` | `sensor_msgs/msg/LaserScan` (361 beams) | best effort | 15 Hz | extended | yes | ≤ 2 Hz (`--scan-hz`, 0 = full rate) |
| `/top_scan` | `sensor_msgs/msg/LaserScan` (720 beams) | best effort | 10 Hz | extended | yes | ≤ 2 Hz (`--scan-hz`) |

- **core**: what reconstructs motion and operations: odometry, IMU, ground-truth world state,
  events, obstacles, mission outcomes (action status) and the navigation interlock.
- **extended**: core plus plans, the AMCL correction, static transforms and both scanners.
- Not recorded: `odom -> base_link` on `/tf` (same data as `/odom`), `/clock` (every record keeps its
  simulation stamp), costmaps, `/joint_states`, `/digital_twin/simulation/poses` (same data as the
  world state). Full-rate scans only when asked (`--scan-hz 0`).
- Decimation keeps a message when at least 1/rate seconds of simulation time passed since the last
  kept one; counts of received, recorded, decimated and filtered messages per topic are stored in the
  execution summary, so nothing is dropped silently.
- Latched topics (`/lite_sim/obstacles`, action status, `/tf_static`) deliver their value once, when
  the subscription matches, usually before an execution starts; the Recorder keeps the latest value
  of each and records it first in every execution (found in the first end-to-end run, where
  `/tf_static` had 0 samples; `test_latched_values_received_before_an_execution_start_it`).
- `required` topics must have samples for the execution to complete; optional ones are created in
  Mosaico on their first message only (no empty streams). With `--simulation-backend gazebo` the
  lite topics are not subscribed.

## Data representation

Typed Mosaico ontologies (`models/ontology.py`), one per message type, versioned by tag
(`digital_twin_<kind>_v1`); a later change gets a `_v2` class, never a modified tag. Nested lists
of structs carry entities, obstacles, goal statuses, diagnostics, path poses and transforms.

Every record has `robot_id` (recording robot), `source_topic`, `source_timestamp_ns` (ROS header
stamp = simulation time; for header-less action status and `/tf_static` the recorder's
simulation-clock reception time, declared per topic as `source_stamp: reception`),
`received_wall_ns` (Unix UTC ns; also the Mosaico message timestamp) and `frame_id`. Values are
copied as published: odometry stays local in `odom` (never relabelled as global), world-state
entities are engine ground truth in `world`, plans stay in `map` (world = inverse of the verified
map registration stored in the metadata), scans in their sensor frame. Non-finite numbers are
rejected except in LaserScan ranges/intensities, where REP 117 gives `+inf` (no return) a meaning;
the integrity hash encodes them explicitly.

The ROS bridge of the SDK (`mosaicolabs.ros_injector`, rosbags-based) was evaluated: it ingests
recorded bags offline and is available in the recorder venv for rosbag2 files (e.g. Gazebo
validation bags; the lite interfaces would need their message definitions registered). Live
recording keeps direct SDK ingestion, which provides the bounded queue, typed ontologies,
identity checks and read-back verification above.

## Execution metadata

One Mosaico sequence per execution: `digital_twin_<backend>_<robot>_<UTC>_<uuid12>`. Sequence
metadata (written at creation): `execution_id`, `simulation_backend`, `simulator`,
`recording_profile`, `robot_id`, `scene_id`, `scene_sha256`, `scene_geometry_sha256` (installed
scene), `simulation_session_id`, `simulation_epoch`, `engine_scene_sha256`,
`sim_time_at_discovery_s` (first world state), `simulation_profile`, `simulation_seed`,
`simulation_step_s`, `simulation_target_real_time_factor` (`/lite_sim` parameters), `map_yaml`,
`map_version` (map image SHA-256, as in Phase 5) and `map_registration` (only when a verified
registration names that image; read from `/map_server`'s `yaml_filename`), `git_commit`,
`git_dirty`, `amr_config_sha256`, `started_wall_utc`, `recording_configuration` (+ its SHA-256),
`coordinate_frames`, `timestamp_semantics`, `previous_execution_id` (after a recovery). Anything that
could not be read is listed in `unavailable` instead of being guessed; Gazebo recordings keep
`scenario` and `warehouse_config_sha256`.

The finalization summary (topic `/recording/summary`, written last) holds `ended_wall_utc`,
`recording_wall_seconds`, `stop_reason`, per-topic count/SHA-256/first/last record, the
received/recorded/decimated/filtered counts, the engine session and the simulation-time span.
A sequence without it is incomplete by definition. The same data is in the local
`recordings/<execution_id>/report.json`, plus the read-back verification.

## Storage infrastructure

Docker Compose (`infrastructure/mosaico/compose.yaml`), runtime-tested on 2026-11-06:

- `database`: `postgres:16.15` pinned by digest, named volume `metadata`, health check `pg_isready`,
  not published (reachable only on the private Compose network).
- `mosaicod`: `ghcr.io/mosaico-labs/mosaicod:0.7.0-rc-c18683a` pinned by digest, named volume
  `objects`, published on `${MOSAICO_BIND_ADDRESS:-127.0.0.1}:${MOSAICO_PORT:-6726}`, waits for a
  healthy database, `restart: unless-stopped`, 40 s stop grace period for active uploads.
- The database password reaches both containers as a Docker secret (file in `/run/secrets`, not a
  container environment variable), from `MOSAICO_DB_PASSWORD` in the untracked `.env` (mode 600).
  An existing `metadata` volume keeps the password it was initialised with.
- Health: the database check is `pg_isready`. The `mosaicod` image is distroless (no shell); its
  check runs `mosaicod ps`, which proves the catalog connection only (it exits 0 even when it lists
  the server as dead; the container itself stops when the server process exits). Service readiness
  is an SDK round trip: `history health` (below).
- `config.toml` bounds SDK batch buffering and daemon memory (unchanged from Phase 2).

```bash
cd ~/ros2_ws/infrastructure/mosaico
cp .env.example .env && chmod 600 .env     # first time only: set MOSAICO_DB_PASSWORD
docker compose up -d --wait                # both services healthy
docker compose ps
docker compose stop                        # or `down`: containers only, volumes stay
# Never `docker compose down -v`, never delete the named volumes.
```

**Separate storage server** (e.g. Windows + Docker Desktop on the LAN, ROS 2 on Ubuntu): the same
compose file plus `compose.remote.yaml`, which enables server-authenticated TLS and API-key
enforcement in `mosaicod`; PostgreSQL stays unpublished. On the server set in `.env`
`MOSAICO_BIND_ADDRESS` to the LAN/VPN interface address (never a public interface; allow the port
only from the ROS machines in the host firewall), `MOSAICO_TLS_CERT_FILE` and `MOSAICO_TLS_KEY_FILE`
(certificate whose name or IP matches what the ROS machine connects to), then:

```bash
docker compose -f compose.yaml -f compose.remote.yaml up -d --wait
docker compose exec mosaicod mosaicod api-key --help   # create a key with read/write permissions
```

On the ROS machine nothing is hard-coded: `MOSAICO_HOST`, `MOSAICO_PORT`, `MOSAICO_TLS_CERT_FILE`
(CA or server certificate) and `MOSAICO_API_KEY` are read from the environment by the Recorder and the
history CLI (`MosaicoClient.from_env`). Keep the key out of shell history and version control.
The TLS/API-key overlay is configuration-validated (`docker compose config`), not runtime-tested here.

The native deployment of Phase 2 (`bootstrap_native.sh`, `local_services.py`; data in
`.runtime/mosaico`) remains for machines without Docker and still holds the Phase 2 recordings. Use
one deployment at a time (both use port 6726).

## Python environments

Use Python 3.12 with ROS 2 Jazzy. Keep the Gateway in `.venv` as documented in
[DEVELOPMENT.md](DEVELOPMENT.md). Do not install the SDK or NumPy 2 into that venv.
The Recorder uses a separate venv without system-site-packages; sourcing ROS makes
the Jazzy Python modules available, while the Recorder's dependencies remain local.

```bash
cd ~/ros2_ws
python3 -m venv --without-pip .venv-recorder
env -u PYTHONPATH .venv/bin/python -m pip --python .venv-recorder install \
  pip -r src/digital_twin_recorder/requirements.txt pytest==8.4.2
env -u PYTHONPATH .venv-recorder/bin/python -m pip check
```

Pins (unchanged, verified again by real ingestion and retrieval on 2026-11-06): mosaicolabs 0.7.0
(matches `mosaicod` 0.7.0-rc), pyarrow 26.0.0, numpy 2.4.6, pydantic 2.14.1, PyYAML 6.0.1. Run pip
checks without the sourced workspace's PYTHONPATH. Recorder tests disable automatic pytest plugin
discovery so ROS launch-testing plugins are not imported into the storage venv.

## Record the lite simulation

```bash
# Terminal 1: storage (see above), then the lite stack
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && source .venv/bin/activate && source install/setup.bash
ros2 launch amr_lite_sim lite_navigation.launch.py

# Terminal 2: the Recorder (same ROS_DOMAIN_ID); Ctrl+C finalizes and verifies
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
.venv-recorder/bin/python -m digital_twin_recorder.ros.recorder_node --profile core --output recordings
#   --profile extended            plans, AMCL TF, static TF, both LiDARs (2 Hz)
#   --scan-hz 0 --world-state-hz 0   full sensor and world-state rates
#   --duration 600                stop after 10 min of recording
#   --recover                     after a failure start a NEW execution (never resume a failed one)
#   MOSAICO_HOST=storage.lan MOSAICO_PORT=6726 MOSAICO_TLS_CERT_FILE=... MOSAICO_API_KEY=...  remote server
```

Gazebo warehouse (Phase 2 path):
`ros2 launch digital_twin_recorder digital_twin_recording.launch.py scenario:=normal
recorder_python:=$HOME/ros2_ws/.venv-recorder/bin/python [recording_profile:=extended]`
or the node with `--simulation-backend gazebo --scenario normal`.

## Failure handling

Lifecycle: idle → connecting → recording → finalizing → completed, or failed (terminal). Completion
requires the finalized SDK session, samples on every required topic, no dropped records, no
ingestion notifications and a read-back whose per-topic counts and SHA-256 equal what the ROS
side hashed. A delayed worker event cannot turn a failed execution into a completed one.

- Callbacks never do I/O; the queue is bounded (2048); a full queue fails the execution instead of
  blocking ROS. SDK buffers are bounded per topic (~3.6 MiB, `config.toml`).
- Storage heartbeat deadline 15 s (`--io-timeout`), server round trip every 3 s, finalization
  deadline 120 s; the worker is a daemon process that also stops when the Recorder dies, and is
  terminated/killed if it overruns. Storage health is polled on a wall-clock timer, so it is checked
  even while simulation time stands still.
- Mosaico down at start or during recording: the execution fails with the reason; the partial
  sequence stays for diagnosis and retrieval refuses it as incomplete. The simulator, Nav2, the
  Gateway and its WebSocket are unaffected.
- `/clock` going back, a different engine `session_id` or `epoch`, or a source stamp going back on
  any topic fails the execution ("refusing to merge executions").
- `--recover`: after such a failure the Recorder waits (`--recover-delay`), re-reads the running
  system and starts a new execution whose metadata names the failed one (`previous_execution_id`).
- Nothing is ever deleted automatically.

## Historical retrieval

Needs only Mosaico (no ROS graph, no simulator). Run the module with the recorder venv (the
`recorder`, `verify_persistence` and `recording_history` console scripts installed by colcon get the
Gateway venv's interpreter, which deliberately has no SDK). The same functions are importable from
Python (`digital_twin_recorder.storage.history`):

```bash
H=".venv-recorder/bin/python -m digital_twin_recorder.storage.history"   # with install/setup.bash sourced
$H health                                   # server version and sequence count; exit 1 if unreachable
$H list                                     # executions: backend, robot, profile, start/end, status, counts, bytes
$H show <execution_id>                      # metadata, summary, per-topic ontology/size/time range
$H verify <execution_id> --expected recordings/<execution_id>/report.json   # integrity vs the ROS-side hashes
$H export <execution_id> --topic /lite_sim/events --output events.jsonl
$H export <execution_id> --topic /odom --sim-from 60 --sim-to 70 --output odom.jsonl   # simulation-time window
$H export <execution_id> --wall-from 2026-11-06T20:30:00Z --wall-to 2026-11-06T20:31:00Z --topic /scan
$H trajectory <execution_id> --output amr_01_world.csv                  # world_state ground truth (world frame)
$H trajectory <execution_id> --source odometry --output amr_01_odom.csv # odometry (odom frame)
$H trajectory <execution_id> --entity EMP-1 --output forklift.csv       # any engine entity
```

```python
from digital_twin_recorder.storage import history
with history.open_client() as client:                      # MOSAICO_HOST/PORT/TLS/API key from the env
    rows = history.list_recordings(client)
    for topic, record in history.records(client, rows[-1]['execution_id'], topics=['/lite_sim/events']):
        print(record['source_timestamp_ns'], record['kind'], record['entity'])
```

The Phase 2 verifier (`scripts/verify_persistence.py`, `storage.retrieval`) keeps working and reads
both schema 1.0 (Phase 2) and 2.0 recordings.

## Validation (2026-11-06, lite runtime, Compose storage)

Evidence: `docs/digital-twin/evidence-phase6/followup/persistence_e2e_v2/` (and the first run in
`persistence_e2e/`); recordings in `recordings/followup/`. Real `lite_navigation.launch.py` stack
(16 workers, 3 forklifts, Nav2 at 0.60 m/s), Mosaico from `compose.yaml`; no fake publishers.

| Step | Result |
|---|---|
| Compose start | both services healthy; PostgreSQL not published; `mosaicod` on 127.0.0.1:6726; existing volumes reused (they held no sequence before) |
| SDK round trip of every record kind (`test_mosaico_integration.py`, opt-in) | 12 topics written, finalized, read back; hashes equal, `+inf` ranges and nested lists preserved |
| Extended recording during a Nav2 mission with a placed box | completed and verified: 15,919 records in 12 topics (`/odom` 6,328, `/imu/data` 6,327, world state 1,266 at 10 Hz, events 2, obstacles 3, action status 2, interlock 125, plans 107, `map -> odom` 1,266, `/tf_static` 1, scans 238/254 at 2 Hz), 6.97 MB; decimated/filtered counts in the summary (world state 2,532, `/tf` 8,441 other transforms, scans 1,661/1,012) |
| `mosaicod` stopped during a core recording (`--recover`, `--io-timeout 5`) | execution failed ~10 s after the daemon exited ("Write failed ... unavailable"); two restarts while it was down failed at connection; the next one (daemon back) completed and verified, `previous_execution_id` set. Meanwhile: engine RTF 1.000–1.004, a Nav2 mission accepted and driven, Gateway WebSocket 20.0 Hz, latency p50 1.0 ms |
| Recorder started with `mosaicod` down | exit 1 after ~13 s with the connection error; engine RTF 0.999 |
| Simulator restarted under a running recorder (`--recover`) | "Source clock reset ... start a new execution"; the new execution carries the new engine session id (`fe06d2bb…` instead of `a931e31b…`) and completed |
| Simulator stopped, `docker compose down` + `up -d --wait` (containers recreated, volumes kept) | all sequences listed again; no ROS process running during retrieval |
| `history verify` after the restart | 4 finalized executions verified (incl. the first run's 37,569-record extended recording from before both restarts), SHA-256 per topic equal to the ROS-side report; the 3 interrupted ones refused as incomplete |
| `history trajectory` (world state and odometry) | 1,266 / 6,328 samples, path 60.213 m; the mission runner measured 60.21 m live through the Gateway |
| `history export` | obstacle add/remove events, goal `a89db0e9` executing → succeeded, `/odom` window 60–70 s simulated (172 records), first 3 `/top_scan` records |
| Phase 2 recordings (native store, schema 1.0) | Compose stopped, native services started: the new `history verify` accepts all 9 finalized Phase 2 sequences (one also against its original `report.json`); `history list` marks the 7 Phase 2 failure-test sequences incomplete; native services stopped, Compose restarted with its 12 sequences (`evidence-phase6/followup/phase2_compat/`) |
| Metadata | engine session, profile `operations`, scene hash equal to the engine's, `map_version` = v9 image hash, registration `89768ddd2c7518fe`, `git_commit` + `git_dirty` |

The first run found one defect, fixed before the second: latched topics delivered their value
before the execution started and it was lost (`/tf_static` 0 samples; `/lite_sim/obstacles` would
have failed a recording without obstacle changes). Recorder tests: 23 (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
MOSAICO_TEST_HOST=127.0.0.1 .venv-recorder/bin/python -m pytest src/digital_twin_recorder/test`).

Not runtime-tested here: the Gazebo backend of the new recorder (inventory and metadata are unit
tested; the Phase 2 harness was not re-run), the TLS/API-key overlay, a Windows Docker host.

## Limitations

- One recording robot per Recorder process (`--robot-id`); the world state still holds every entity.
- Scans and world state are decimated by default; full rates are an explicit choice and grow the
  store accordingly (measured at default rates: core 15–20 KiB/s, extended 50–61 KiB/s of wall time,
  i.e. roughly 55–70 MiB and 175–215 MiB per hour).
- Execution metadata is written at sequence creation (the SDK has no metadata update); the end
  state lives in the finalization summary.
- A failed execution is never resumed or merged; `--recover` starts a new one, so a storage outage
  leaves a gap between two sequences (the summary of the second names the first).
- The history CLI streams through the SDK; very long recordings are read sequentially.


## Phase 2 (Gazebo) harness

`src/digital_twin_recorder/scripts/validate_phase2.py` (Gazebo warehouse, native storage, browser
tests; now passes `--simulation-backend gazebo`). It starts native storage and refuses occupied
ports, so stop the Compose services first. Results of the original Phase 2 run:
[PHASE_2_VALIDATION.md](PHASE_2_VALIDATION.md).
