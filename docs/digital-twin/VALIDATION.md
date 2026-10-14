# Phase 1 implementation and validation report

Date: 2026-10-13. Workspace: `/home/gengo/ros2_ws`.
Branch: `feature/digital-twin-foundation`. No commits or pushes were made.

## Repository analysis and changes

Starting working tree was clean on `main`. The existing AMR and warehouse packages,
Xacro/URDF/STL assets, sensor bridges, controllers, physics, human system, procedural
generator and four scenarios were inspected and reused. The initial warehouse
pytest baseline passed all **30 tests**. The audit is recorded in ARCHITECTURE.md.

The only existing tracked file modified is `.gitignore` (local dependencies,
frontend output and caches). No files in `amr_description`, `amr_simulation`,
`my_cpp_pkg` or `my_py_pkg` were changed. The workspace now builds five packages.
`web/COLCON_IGNORE` explicitly excludes the frontend from ROS package discovery.
Previously recorded simulation evidence is untouched.

The implemented communication path is:

```text
Existing Gazebo Harmonic headless physics and sensors
 → existing ros_gz bridges + ros2_control
 → /clock, /odom, /tf, /tf_static, /joint_states
 → managed rclpy executor → synchronized Pydantic state
 → FastAPI HTTP + bounded WebSocket snapshots
 → shared JSON Schema validation → TypeScript diagnostics in Chromium
```

The Gateway is read-only. Pose comes from real odometry in its original frame;
there is no global localization, artificial robot data or browser robot command.
Only the test helper publishes controlled TwistStamped messages via ROS.

## Dependencies actually used

| Component | Version / compatibility |
|---|---|
| Ubuntu | 24.04.4 LTS |
| ROS | Jazzy, rclpy 7.1.11, system Python extensions |
| Gazebo | Harmonic / Sim 8.15.0, existing ros_gz and gz_ros2_control |
| Python | 3.12.3, project `.venv` with system-site-packages |
| FastAPI / Starlette | 0.143.0 / 1.8.0 |
| Pydantic / Uvicorn | 2.14.1 / 0.54.0 |
| websockets / HTTPX | 15.0.1 / 0.28.1 |
| jsonschema / pytest / pytest-asyncio | 4.26.0 / 8.4.2 / 0.26.0 |
| colcon-core | 0.20.1; virtual-environment entrypoint ensures correct shebang |
| Node / npm | 22.23.3 project-local / 9.2.0 existing npm |
| Vite / TypeScript | 8.3.4 / 7.0.2 |
| Three.js | 0.186.1, dependency only; no rendering is executed |
| Ajv / ajv-formats | 8.20.0 / 3.0.1, JSON Schema 2020-12 validation |
| json-schema-to-typescript | 16.0.0, generated types checked for drift |
| Vitest / Playwright | 5.0.4 / 1.64.0, Chromium headless shell 156.0.8078.4 |

Python dependencies are pinned in the package's `requirements.txt`; all npm versions
and transitive dependencies are locked in `web/digital-twin/package-lock.json`.
System Node 18.19.1 was insufficient for Vite; Node 22 was installed under `.tools`.
Ubuntu ensurepip was absent and passwordless sudo unavailable, so the already
installed system pip bootstrapped `venv --without-pip --system-site-packages`.
No global pip or ROS installation changes were made.

## Commands and results

Commands run from the workspace after sourcing ROS, install and `.venv` as specified
in DEVELOPMENT.md. Initial builds also used the equivalent `python -m colcon`;
the final build/test commands below used the venv `colcon` executable directly.

| Executed command | Actual result / evidence |
|---|---|
| `python3 -m pytest src/amr_simulation/tests -q` before changes | 30 passed (baseline). |
| `colcon build --symlink-install` | All 5 packages built; [log](evidence/build-final.log). |
| `colcon test --packages-select digital_twin_gateway --event-handlers console_direct+` | 23 passed; [log](evidence/gateway-tests-final.log). |
| `colcon test-result --test-result-base build/digital_twin_gateway --verbose` | 23 tests, no errors/failures/skips; [result](evidence/gateway-test-results-final.txt). |
| `colcon test --event-handlers console_direct+` | All 5 package test tasks completed; [log](evidence/workspace-tests-final.log). |
| `colcon test-result --verbose` | 58 reported tests, zero errors/failures, one existing copyright skip; [result](evidence/workspace-test-results-final.txt). The aggregate includes CTest suite wrapper entries. |
| `python src/digital_twin_gateway/scripts/export_schemas.py --check` | Passed; checked-in schemas match model-generated schemas. |
| `npm --prefix web/digital-twin test` | 6 client/protocol tests passed; [log](evidence/frontend-tests-final.log). |
| `npm --prefix web/digital-twin run build` | Type generation drift check, strict TypeScript and Vite production build passed; [log](evidence/frontend-build-final.log). |
| `npm run test:browser` through integration harness | 2 Chromium tests: real telemetry/motion/network recovery, and usable UI with backend blocked. |
| Existing `validate_urdf.py` in copied AMR package | Passed Xacro/URDF tree, inertias, geometry and Harmonic SDF checks; [report](evidence/amr-regression/urdf.json). |
| Existing `smoke_test.py --only simulation` in copied AMR package | Passed real sensors, TF, physical motion, odometry, turn and braking; [log](evidence/amr-regression.log), [measurements](evidence/amr-regression/runtime.json). |
| `python3 -m pytest test -q` in `src/my_py_pkg` | 2 passed, 1 existing skip; [log](evidence/legacy-package-tests.log). |

Gateway test coverage includes quaternion/frame preservation, all velocity axes,
finite values, schema/format compliance, null data, stale/offline/recovery, frozen
clock, simulation reset, static TF retention, immutable snapshots, sequencing,
concurrent state updates, multiple robot topic configurations, all HTTP routes,
CORS/Origin/version rejection, multiple sockets, reconnect, command/binary rejection,
client cap, capacity-one queues, timed-out send, vanished peers during close,
startup failure cleanup, real executor lifecycle, duplicate SIGINT, and all four
launch scenario compositions. Frontend tests cover schema rejection, HTTP errors,
sequence/session handling, connection transitions, exponential retry limits/reset,
heartbeat timeout and event/timer cleanup.

The original AMR regression ran against a temporary copied package so its existing
validators could write their usual `validation/` files without overwriting old
records. Its measured translation was 0.575246 m versus odometry 0.575245 m, and
turn was 1.246539 rad. The copy was removed only after saving new evidence here.
Full warehouse collision/human trajectory GUI acceptance and Blender reconstruction
were not repeated: no corresponding code or assets changed. The original AMR GUI
launch remains available; this phase intentionally exercises headless operation.

## Issues found and resolved

1. Launch tests needed ROS substitution normalization and source resolution before
   asserting the included launch path. Corrected tests; the composition was unchanged.
2. Gazebo's Ruby launcher rewrites `/proc` argv as one process-title string. The
   integration observer tokenizes it before checking `-s --headless-rendering`.
3. `ros2controlcli` is absent. Controller activation is verified through the actual
   `/controller_manager/list_controllers` service using the installed message type.
4. Duplicate terminal/launch SIGINT could activate Uvicorn's force-exit path and
   skip lifespan cleanup. The Gateway coalesces repeated shutdown signals and
   explicitly joins its ROS executor. The harness signals launch itself, allowing
   it to forward the shutdown once to its children.
5. Uvicorn closes sockets with 1012 at shutdown. The observer now records that
   expected close code rather than treating it as a failed test.
6. Browser network loss exposed `WebSocketDisconnect` during a final close attempt
   on an already lost peer. Cleanup now handles that condition. Regression tests
   cover it and the final integration also checks for unhandled Gateway ASGI errors.

Earlier diagnostic runs remain in timestamped evidence directories; they are not
silently relabeled as successful acceptance runs. See their individual reports/logs.
An exploratory legacy pytest invocation from the workspace root incorrectly scanned
the entire workspace for lint; its compressed log is retained. That is not a valid
package regression invocation. The correct package-directory and colcon invocations
passed. Historical legacy lint failures mentioned in old repository reports did not
reproduce with the correct current invocation; no example package was edited.

Nonblocking environment messages: setuptools/colcon reports the pre-existing
`pytest-repeat` egg metadata warning; Starlette warns about future HTTPX test-client
migration. System and venv `pip check` both report the same pre-existing PyNaCl →
missing cffi dependency (see their paired logs); the Gateway does not import PyNaCl.
No unrelated environment repair was performed.

## Scope and Phase 2 readiness

Ready: stable robot identities, configurable topic mapping, multi-robot state array,
versioned protocol and generated browser types, read-only REST/WebSocket clients,
status/freshness handling, bounded queues, clean lifecycle, headless composition,
real browser integration tests, and installed Three.js with a rendering boundary.

The original JSON files remain the only geometry/dimension/station sources.
Phase 2 can add a normalized configuration export and a Three.js renderer consuming
this state; that renderer must explicitly register odom/world and convert axes.
No duplicate hardcoded warehouse, model viewer, physics, human tracking, graph,
Nav2/SLAM, fleet adapter, mission state or command dashboard is present.

Limitations: localhost-only development policy with no authentication/TLS; running
simulation is inferred from clock progression, so pause and silence become unavailable;
scenario is declared configuration, not detected world identity; TF/joints expose
receipt diagnostics only; no history or replay; reserved entity arrays stay empty.
The measured rate and bandwidth are from short local runs, not capacity, latency,
CPU or long-duration production benchmarks. The late static TF receipt was verified
through the three latched publishers in runtime status, but no global localization
is claimed. No cloud deployment or Phase 2 rendering was performed.

## Final real integration evidence

Final normal-scenario acceptance: [report](evidence/20261013T163702Z/report.json),
[launch log](evidence/20261013T163702Z/launch.log),
[Chromium test log](evidence/20261013T163702Z/browser-test.log).
Command: `.venv/bin/python src/digital_twin_gateway/scripts/validate_integration.py`.
All 12 harness checks passed, including no unhandled Gateway errors.

- Server process was `gz sim -r -v 3 -s --headless-rendering .../warehouse_normal.sdf`;
  process inspection found no Gazebo GUI or RViz. Robot entity creation succeeded.
- The controller-manager service reported both original controllers active:
  [service response](evidence/20261013T163702Z/controllers.txt). Real topic types are
  recorded in [topics](evidence/20261013T163702Z/topics.txt).
- A Python WebSocket observer and Chromium received simultaneous schema-valid,
  strictly sequenced states. The observer retained 210 snapshots,
  measured **9.959 updates/s**, and mean JSON size after
  Python re-encoding was **2127.5 bytes/snapshot**.
  This is a snapshot-size diagnostic, not a measured wire bandwidth/latency claim.
- A ROS-only test published 0.12 m/s `TwistStamped` for 2 simulation seconds, then
  zero velocity. Gateway displacement was **0.236405 m**;
  independent ROS odometry ended at **0.236405 m**.
  Chromium displayed X changing from **0.0000** to **0.2364 m**.
  Final reported linear X velocity was **3.947e-17 m/s**.
  [Motion data](evidence/20261013T163702Z/motion.json),
  [browser text](evidence/20261013T163702Z/browser.json),
  [before screenshot](evidence/20261013T163702Z/browser-before.png),
  [after screenshot](evidence/20261013T163702Z/browser-after.png).
- Chromium passed network-loss/reconnect assertions and a separate test with
  backend access blocked. [Disconnected screenshot](evidence/20261013T163702Z/browser-disconnected.png).
  No fake telemetry was used by these real integration tests.
- The pose stayed in `odom`, velocity in `base_link`, source `/odom`; all five
  subscriptions received real data. [Final TwinState](evidence/20261013T163702Z/final-state.json),
  [connection diagnostics](evidence/20261013T163702Z/status.json).
- SIGINT to launch terminated all owned processes without forced cleanup; active
  WebSocket closed with code 1012, API port was released,
  and no Gateway ASGI exception remained. The browser/Vite processes were also stopped.

The other scenarios ran the same harness with `--scenario NAME --skip-browser`:

| Scenario | Evidence | Result |
|---|---|---|
| person_crossing | [report](evidence/20261013T163413Z/report.json) | 9 checks passed |
| obstacle_in_aisle | [report](evidence/20261013T163434Z/report.json) | 9 checks passed |
| corridor_blocked | [report](evidence/20261013T163453Z/report.json) | 9 checks passed |

Those smoke runs verify selection, headless execution, controllers, real odometry,
schema-valid streaming and process cleanup. The full browser/motion/network-loss
acceptance applies to `normal`. The small final socket-close correction was then
retested with unit tests and the full normal integration; other scenarios do not
have separate protocol implementations.

## Files created or modified

All paths below are relative to the workspace. Every entry is new except `.gitignore`.
The grouped entries list files with the same responsibility.

| Files | Responsibility |
|---|---|
| `.gitignore` | Ignore isolated tools, dependencies, caches and frontend build/test output. |
| `web/COLCON_IGNORE` | Keep the frontend and node_modules outside colcon discovery. |
| `src/digital_twin_gateway/package.xml` | ament_python package and ROS/runtime/test dependency metadata. |
| `src/digital_twin_gateway/setup.py`, `setup.cfg` | Python package installation, executable, launch data and pytest configuration. |
| `src/digital_twin_gateway/resource/digital_twin_gateway` | Required ament package-index marker. |
| `src/digital_twin_gateway/requirements.txt` | Exact project Python dependency versions. |
| `src/digital_twin_gateway/README.md` | Package entrypoint and links to detailed guides. |
| `src/digital_twin_gateway/digital_twin_gateway/__init__.py` | Python package declaration. |
| `src/digital_twin_gateway/digital_twin_gateway/config.py` | Validated external configuration and existing-layout world metadata. |
| `src/digital_twin_gateway/digital_twin_gateway/main.py` | Single Uvicorn worker and managed process signal behavior. |
| `src/digital_twin_gateway/digital_twin_gateway/ros/__init__.py` | ROS adapter subpackage declaration. |
| `src/digital_twin_gateway/digital_twin_gateway/ros/node.py` | Owned ROS context, executor thread and shutdown. |
| `src/digital_twin_gateway/digital_twin_gateway/ros/subscriptions.py` | Real telemetry subscriptions, QoS, receipt diagnostics and invalid-data rejection. |
| `src/digital_twin_gateway/digital_twin_gateway/ros/state_collector.py` | Odometry-to-protocol normalization without frame/orientation changes. |
| `src/digital_twin_gateway/digital_twin_gateway/state/__init__.py` | State subpackage declaration. |
| `src/digital_twin_gateway/digital_twin_gateway/state/models.py` | Pydantic wire contract source, finite values and reserved collections. |
| `src/digital_twin_gateway/digital_twin_gateway/state/state_manager.py` | Locked latest values, freshness, clock progress, immutable sequenced snapshots. |
| `src/digital_twin_gateway/digital_twin_gateway/api/__init__.py` | API subpackage declaration. |
| `src/digital_twin_gateway/digital_twin_gateway/api/app.py` | FastAPI lifespan, CORS and timed snapshot publisher. |
| `src/digital_twin_gateway/digital_twin_gateway/api/routes.py` | Three read-only HTTP routes and WebSocket route. |
| `src/digital_twin_gateway/digital_twin_gateway/api/websocket.py` | Origin/version checks, bounded fan-out, send timeout, cleanup. |
| `src/digital_twin_gateway/launch/digital_twin.launch.py` | Existing headless warehouse launch composition, scenario passthrough, Gateway process. |
| `src/digital_twin_gateway/scripts/export_schemas.py` | Generate/check public JSON Schema from Pydantic models. |
| `src/digital_twin_gateway/scripts/validate_integration.py` | Owned real simulation/API/browser validation and timestamped evidence. |
| `src/digital_twin_gateway/scripts/validate_motion.py` | Controlled stamped ROS test command, independent odometry comparison and guaranteed stop attempts. |
| `src/digital_twin_gateway/test/conftest.py` | Deterministic time, lifecycle doubles and real ROS odometry fixtures. |
| `src/digital_twin_gateway/test/test_state.py` | Normalization, state transitions, schema, concurrency and configuration tests. |
| `src/digital_twin_gateway/test/test_api.py` | HTTP, WebSocket, bounded clients, lost-peer cleanup and startup failure tests. |
| `src/digital_twin_gateway/test/test_ros_runtime.py` | Real ROS lifecycle and duplicate process signal tests. |
| `src/digital_twin_gateway/test/test_launch.py` | All four existing scenario compositions. |
| `shared/schemas/twin-state.schema.json`, `connection-status.schema.json` | Versioned wire contracts generated from runtime models. |
| `shared/schemas/README.md` | Contract regeneration and compatibility rules. |
| `web/digital-twin/package.json`, `package-lock.json` | Scripts and pinned runtime/development dependencies including Three.js. |
| `web/digital-twin/tsconfig.json`, `vite.config.ts` | Strict TypeScript and local Vite configuration. |
| `web/digital-twin/index.html` | English diagnostic application shell. |
| `web/digital-twin/.env.example` | Configurable REST/WebSocket endpoints. |
| `web/digital-twin/README.md` | Frontend commands and integration test entrypoint. |
| `web/digital-twin/scripts/generate-types.mjs` | Generate/check TypeScript types directly from shared schemas. |
| `web/digital-twin/src/app/main.ts` | Diagnostics UI, status/telemetry binding and browser lifecycle cleanup. |
| `web/digital-twin/src/core/api/client.ts` | Typed REST client with deadline, cancellation and schema validation. |
| `web/digital-twin/src/core/websocket/client.ts` | Native WebSocket, validated messages, heartbeat, bounded exponential reconnect and cleanup. |
| `web/digital-twin/src/core/state/store.ts` | Connection state, current snapshot and session-aware sequence acceptance. |
| `web/digital-twin/src/core/types/twin-state.ts`, `connection-status.ts` | Generated protocol types, no independent hand-maintained model. |
| `web/digital-twin/src/core/types/validation.ts` | Shared-schema runtime validation with Ajv. |
| `web/digital-twin/src/rendering/README.md` | Explicit Phase 2 rendering boundary and frame requirements. |
| `web/digital-twin/src/styles/main.css` | Responsive diagnostic layout. |
| `web/digital-twin/test/clients.test.ts` | Lightweight client/protocol/connection automated tests. |
| `web/digital-twin/playwright.config.ts`, `test/diagnostics.browser.ts` | Real Chromium integration and backend-unavailable UI checks. |
| `docs/digital-twin/ARCHITECTURE.md` | Repository audit, Mermaid architecture, concurrency, coordinate semantics and references. |
| `docs/digital-twin/COMMUNICATION_PROTOCOL.md` | Wire fields, routes, timestamps, freshness, versioning and examples. |
| `docs/digital-twin/DEVELOPMENT.md` | Exact dependency/build/run/inspect/test/stop commands and configuration. |
| `docs/digital-twin/VALIDATION.md` | This technical report, measured evidence and limitations. |
| `docs/digital-twin/evidence/` | Build/test logs, retained diagnostic failures, timestamped scenario reports, controller/topic evidence, actual JSON payloads and Chromium screenshots. |

No empty speculative navigation, graph, fleet or rendering implementation was added.
Package markers and `__init__.py` files serve Python/ament discovery; they are not
placeholder features.
