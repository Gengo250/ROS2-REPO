# Phase 2 final validation — 2026-10-17

**PHASE 2: FULLY VALIDATED** for the implemented Phase 2 scope on this development
machine. Real headless Gazebo telemetry was stored in Mosaico and recovered after
Gazebo shutdown and restart of both storage services. All four Three.js scenarios
and the original GLB passed real Chromium tests. No Phase 3 capabilities were added.

## 1. Initial repository and audit

Branch: `feature/digital-twin-visualization-storage`; starting commit
`acadf8ee16ca12287fa0560e521747193555951a`. The working tree was initially clean.
Existing code already implemented the scene adapter, primitive batching, GLB loader,
cameras, separate Recorder process/queue, typed ontologies, integrity accumulation,
retrieval and native/Compose infrastructure. Existing AMR models, controller and
bridge configuration, warehouse generator/layout and scenarios were preserved.

The initial Recorder environment could not import rclpy or mosaicolabs. Required
Phase 2 guides were absent. The initial integration harness skipped the Phase 1
browser test, ran rendering after recording, did not stop native services, and had
no port ownership preflight. Actual testing additionally exposed terminal-state,
WebSocket cancellation, camera-reset and daemon-shutdown bugs detailed below.

## 2. Python environments

Both interpreters are Python 3.12.3, using Jazzy rclpy 7.1.11 from `/opt/ros/jazzy`.
Gateway: FastAPI 0.143.0, Pydantic 2.14.1, WebSockets 15.0.1, NumPy 1.26.4.
Recorder: mosaicolabs 0.7.0, PyArrow 26.0.0, NumPy 2.4.6, Pydantic 2.14.1,
PyYAML 6.0.1. Native serialization/deserialization of Odometry and Imu passed in
both environments, in addition to the real recording pipeline.

Recorder was repaired with a pure venv and complete dependency installation,
including PyYAML. Its isolated `pip check` reports **No broken requirements found**.
Gateway has no SDK/PyArrow; Recorder has no FastAPI/WebSockets. No global packages
or ROS installation were changed. The Gateway environment was left unchanged.

The system-site Gateway environment's pre-existing pip metadata warning says
PyNaCl lacks cffi; PyNaCl is not used by this pipeline. The ROS workspace overlay
also exposes Gateway metadata to Recorder pip checks, so isolated dependency
checks unset PYTHONPATH, while runtime import/tests explicitly source ROS.
Recorder pytest disables unrelated ROS plugin auto-discovery. Exact setup and
verification commands: [storage guide](PHASE_2_STORAGE.md#python-environments).

## 3. Mosaico infrastructure

Selected deployment: native services only. PostgreSQL 16.15, port 55432;
`mosaicod` 0.7.0-rc, port 6726; loopback binds. SDK 0.7.0 successfully performed
server-info, creation, upload, list and actual streaming reads. The correct SDK
call is `client.info().version`; `client.version()` does not exist in this SDK.
The downloaded daemon archive passed its SHA-256 check. Tools were already
provisioned; the bootstrap script was reviewed, not reinstalled globally.

`config.toml` was accepted by the real daemon; SDK server info confirms the 4 MiB
decode bound. Persistent directories: `.runtime/mosaico/postgres` and
`.runtime/mosaico/objects/telemetry`. Credentials remain ignored, mode 0600;
no credential files are tracked. Existing data was retained. Compose parsed
successfully with a placeholder password but was not started, because this run
uses the approved native alternative exclusively.

Active Arrow Flight uploads can prevent graceful daemon exit. Native stop waits
30 seconds, logs a fallback, and kills only the verified local daemon PID when
needed. PostgreSQL then shuts down normally. This forced outage was exercised
against the real storage service; subsequent retrieval proves completed data
survived it as well as the explicit restart.

## 4. Real Recorder validation

Execution: `digital_twin_normal_20261017T144649Z_608e5457e691`.
Start: `2026-10-17T14:46:49.169516+00:00`; finalization timestamp:
`2026-10-17T14:48:26.145782+00:00`. Report elapsed wall time:
**97.866 s**, including connection/finalization.
The controlled recording ran throughout the browser suite and ended by SIGINT.
Source spans below are ROS simulation time, not wall duration.

| Topic | Captured/retrieved count | First source/retrieved ns | Last source/retrieved ns | Source span |
| --- | ---: | ---: | ---: | ---: |
| /odom | 1367 | 9440000000 | 36760000000 | 27.320 s |
| /imu/data | 2735 | 9430000000 | 36770000000 | 27.340 s |

Real publisher inventories identify the existing diff-drive controller and
`ros_gz_bridge`; no fake publisher was used. Robot ID `amr_01`, odom/base_link and
IMU frame IDs, source/reception timestamps, position, orientation, velocities,
acceleration and all covariance arrays are included in each canonical record.
The sequence completed the SDK finalization lifecycle and read-back verification.
Actual diagnostic motion occurred during recording; the 3D model stayed at spawn.

## 5. Historical persistence and failure isolation

Simulation stopped before retrieval: **True**.
Service PIDs changed from `{'mosaicod.pid': '245885', 'postgres/postmaster.pid': '245878'}` to
`{'mosaicod.pid': '253120', 'postgres/postmaster.pid': '253113'}` after restart. The existing
`verify_persistence.py` streamed the original sequence and compared all records
against the original source report. Counts, first/last records and SHA-256 hashes
match for both topics; no ingestion notifications were reported.

Stored sequence size reported by Mosaico: **427,506 bytes**.
This is sequence data size, not total PostgreSQL/filesystem allocation. All three
sequences listed before this session remain listed after the final restart.
The machine-readable [summary](phase2-validation-summary.json) retains both source
and retrieved timestamps and hashes; the raw retrieval includes complete first
and last messages, stored metadata and finalization summary.

Missing IMU, storage outage during recording and storage unavailable at startup
all produced **failed**, nonzero exits. The partial outage sequence was rejected
by historical retrieval. Gateway health remained online and WebSocket snapshots
continued with advancing real telemetry. Queue-overflow and hung-worker tests
verify nonblocking submission, deadlines and process termination. Outage RSS
samples are retained (peak combined parent/worker/resource-tracker RSS: 277,835,776 bytes); no Recorder/worker processes remained afterward. This is a
bounded functional outage test, not a long-duration memory soak test.

## 6. Three.js and measured performance

Four real scene API responses and seven requested screenshots are retained:

[normal](evidence-phase2/20261017T144619Z/browser/normal.png) · [person_crossing](evidence-phase2/20261017T144619Z/browser/person_crossing.png) · [obstacle_in_aisle](evidence-phase2/20261017T144619Z/browser/obstacle_in_aisle.png) · [corridor_blocked](evidence-phase2/20261017T144619Z/browser/corridor_blocked.png) · [robot-close-up](evidence-phase2/20261017T144619Z/browser/robot-close-up.png) · [top-down](evidence-phase2/20261017T144619Z/browser/top-down.png) · [overview](evidence-phase2/20261017T144619Z/browser/overview.png)

Geometry parity checks cover the 30 × 20 × 5 m building, 34 × 24 m apron floor,
16 racks, ten pillars, six stations, corridor layouts, equipment, human override,
pallet/cone and barrier. Hash validation ties the rendered primitives to the
existing generated SDF files and AMR configuration. GLB bounds are approximately
0.951 × 0.700 × 0.344 m with one axis conversion and configured world spawn.

Playwright checked WebGL initialization, real GLB bounds, all previews, overview,
robot focus, top-down, orbit, pan, zoom, exact reset, viewport resizing, scene
resource counts, pagehide cleanup, reload and diagnostics. No unexpected browser
errors or HTTP asset failures occurred. A separate deliberate GLB-404 test passed.

| Scenario | Mean FPS | Scene load | Draw calls | Geometries | Triangles |
| --- | ---: | ---: | ---: | ---: | ---: |
| normal | 46.54 | 875.4 ms | 51 | 50 | 24676 |
| person_crossing | 8.44 | 823.2 ms | 51 | 50 | 24676 |
| obstacle_in_aisle | 8.28 | 1054.2 ms | 51 | 50 | 25232 |
| corridor_blocked | 8.61 | 1217.3 ms | 51 | 50 | 24808 |

FPS means use five measured intervals per scenario; raw samples remain in
`browser/rendering.json`. Browser backend: **SwiftShader software rendering**,
WebGL 2.0 via ANGLE/Vulkan. These values describe this headless machine, not GPU
performance. Browser JS heap estimates are retained per scenario; they exclude
GPU/native process memory and can be quantized by Chromium. Geometry counts stay
bounded across reloads. No performance target was invented or claimed as met.

Gateway WebSocket arrival frequency: baseline **9.985 Hz**;
with Gazebo + Recorder + Chromium **9.873 Hz**;
storage unavailable **9.949 Hz**. Per-window samples
also cover the storage shutdown wait. Compare these measured values with the
Phase 1 reference of approximately 9.96 Hz. Snapshot frequency is distinct from
simulation speed: this recording covers about 27.3 seconds of simulation in
97.9 seconds of wall time including startup/finalization. The source spans and
counts match approximately 50 Hz odometry and 100 Hz IMU in simulation time; this
is not a claim of real-time-factor 1.0 under software rendering.

## 7. Bugs fixed

1. Incomplete Recorder venv: installed missing transitive dependencies and explicit
   PyYAML while keeping NumPy 2 isolated from system SciPy and the Gateway.
2. Recorder terminal failure could be overwritten by late worker completion:
   terminal states now reject late events; regression failed before and passes after.
   Finalizing state is also persisted before waiting for the worker.
3. WebSocket cleanup could be cancelled repeatedly: shielded bounded cleanup and
   joined child tasks. Original test failed 20/30 repetitions; after correction,
   30/30 passed. Added a deterministic child-cleanup cancellation regression.
4. Camera reset retained orbit/pan damping: presets flush inertia before applying
   the exact pose. The browser caught a 0.238 m X deviation before the correction;
   the exact reset assertion now passes.
5. Native daemon stop could hang on active uploads: bounded graceful wait with
   owned-PID fallback. Full restart/retrieval validates retained completed data.
6. Validation gaps: both browser gates enabled, simultaneous recording/rendering,
   measured controls/memory/backend, actual ROS topic discovery without a stale
   daemon, port conflict refusal, owned service teardown, explicit failure cases,
   restart PID evidence and real retrieval through the existing verification script.
7. Missing/stale documentation: added storage, visualization and validation guides
   and updated development instructions. No models or approved architecture changed.

## 8. Automated results

| Command/check | Final result |
| --- | --- |
| `colcon build --symlink-install` | Six packages built |
| `colcon test --event-handlers console_direct+` / `colcon test-result --verbose` | 79 reported tests; 0 errors, 0 failures, 2 skips |
| Recorder pytest in isolated venv | 14 passed, 0 skipped |
| WebSocket regression repeated | 30 passed after fix |
| Gateway API tests including cancellation regression | 11 passed |
| `npm --prefix web/digital-twin test` | 8 passed |
| `npm --prefix web/digital-twin run build` | Passed type/schema checks and production build |
| Playwright through full harness | 4 passed, 0 skipped |
| Gateway schema export `--check` | Passed |
| Full `validate_phase2.py` | Passed, including restart retrieval and failure isolation |
| Compose configuration / daemon archive checksum | Passed; Compose runtime intentionally not started |

The two colcon skips are explicit: the existing my_py_pkg copyright placeholder,
and the SDK ontology test under the Gateway interpreter (SDK intentionally absent).
That ontology test **did run and pass** in the separate Recorder test invocation.
No real warehouse/browser test was skipped. Non-failing warnings include distro
pytest-repeat metadata, deprecated Starlette/httpx TestClient integration, and the
frontend's roughly 807 kB minified bundle / 209 kB gzip size warning.

## 9. Evidence, limitations and acceptance

Full execution evidence: `docs/digital-twin/evidence-phase2/20261017T144619Z`.
Build/environment/regression logs: `docs/digital-twin/evidence-phase2/audit-20261017`.
The existing ignore rule keeps large screenshots/raw logs local. The checked-in
candidate files [this report](PHASE_2_VALIDATION.md) and
[summary JSON](phase2-validation-summary.json) retain results and SHA-256 references.
Final independent process/port audit found no relevant remaining processes and
ports 8000, 5173, 6726 and 55432 all free (`audit-20261017/shutdown-audit.json`).
No commits or pushes were created. Earlier failed runs are retained for diagnosis;
they are not used as proof of final acceptance. Phase 1 evidence was not overwritten.

- [x] Warehouse, original GLB, geometry parity, cameras and four scenarios.
- [x] Real browser tests, seven screenshots and working diagnostics.
- [x] Both Python environments, ROS imports, SDK imports and operational compatibility.
- [x] PostgreSQL, daemon, SDK connectivity, sessions and actual odometry/IMU ingestion.
- [x] Historical retrieval and matching data after full storage restart.
- [x] Failure isolation, failed-state correctness, bounded queue and worker termination.
- [x] Workspace/frontend builds, automated tests and Phase 1 diagnostic motion regression.
- [x] Headless Gazebo, owned service shutdown and documentation with reproducible commands.

Remaining boundaries: hardware-accelerated browser performance and prolonged soak
behavior are not established by this run; Compose runtime is an untested alternative.
These are distinct from the explicitly deferred Phase 3 live 3D motion/localization,
Nav2, SLAM, graph planning and fleet management, none of which were introduced.

## 10. Verdict

**PHASE 2: FULLY VALIDATED.** The two required deliverables are demonstrated with
actual application screenshots and real ROS → Mosaico → restart → historical
retrieval, with failure handling and regression tests passing. The result applies
to the tested native development deployment and the Phase 2 static-scene scope.
