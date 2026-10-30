# Phase 5.1 validation record

Executed checks only, including failures. Raw evidence lives in the git-ignored
`docs/digital-twin/evidence-phase5-1/` directory, as in earlier phases. Paths below
are relative to it. The diagnosis and design are in `PHASE_5_1_REMEDIATION.md`.

## Environment

- Branch `feature/phase5-1-remediation`, created from `3bdc33a` (Phase 5). No commits
  or pushes.
- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (gz-sim 8, gz-rendering 8.2.3, ogre2),
  Nav2 1.3.13, SLAM Toolbox 2.8.5.
- Host: 8 cores, 15.8 GB RAM, Intel UHD + NVIDIA GTX 1650 (Mesa 25.2.8).
- Sensor rendering, measured from `~/.gz/rendering/ogre2.log`:
  - default (defective): `zink Vulkan 1.4 (NVK TU117)`;
  - fixed (`NOUVEAU_USE_ZINK=0`, set by `sim.launch.py`): `NV167` (nouveau GL).

## 1. Simulated LiDAR integrity (root cause 0)

`validate_lidar_rendering.py`: 4 poses × 5 scans per scanner, every beam compared
with a ray cast against the warehouse model's visuals.

| Pose | zink: front / top beams missing | nouveau GL: front / top missing | nouveau GL: median error of agreeing beams (front / top) |
|---|---|---|---|
| spawn | 49.9 % / 68.1 % | **0.0 % / 1.1 %** | 4.5 / 4.6 mm |
| junction, facing east | 49.6 % / 57.2 % | **0.0 % / 0.6 %** | 4.2 / 4.7 mm |
| junction, facing north | 42.4 % / 62.1 % | **0.0 % / 0.4 %** | 3.5 / 4.3 mm |
| west service aisle, facing north | 43.0 % / 52.5 % | **0.0 % / 0.3 %** | 1.8 / 2.9 mm |

- Verdict: zink `lidar-rendering-zink-defect` **FAILED**; nouveau GL
  `lidar-rendering-fixed` **PASSED** (limit 2 %).
- Supporting evidence:
  - `streak-probe/per-beam-no-return.json`: front scanner returned nothing at 45–90°
    left in 100 % of 1,388 scans;
  - `static-scan-capture/summary.json`: the same defect on the untouched Phase 5
    robot;
  - `streak-probe/first-scan-vs-geometry.png`.
- Every runtime sensor audit in this phase passed the new blind-arc check. The front
  scanner's widest no-return arc was 0–3°; the 360° scanner's was 54.5°, the open
  door seen from the spawn.

## 2. SLAM diagnosis (experiments A–F, 21 steps, 71 m, 22.1 rad)

| Run | Renderer | Odometry | SLAM params | Rotations, open junction | Rotations, facing wall | Final / max error | Largest step |
|---|---|---|---|---|---|---|---|
| `diag-baseline` | zink | Phase 5 | Phase 5 | 0.017–0.153 m | 0.069–0.731 m | 0.285 / 1.916 m | 0.741 m |
| `diag-baseline-fixed-render` (untouched `3bdc33a`) | nouveau GL | Phase 5 | Phase 5 | 0.002–0.015 m | 0.002–0.010 m | **0.020 / 0.097 m** | 0.087 m |
| `diag-fixed-render-phase5-params` | nouveau GL | calibrated | Phase 5 | 0.001–0.016 m | 0.001–0.023 m | 0.025 / 0.106 m | 0.044 m |
| `diag-fixed-render-production` | nouveau GL | calibrated | tuned (reverted) | 0.016–0.032 m | 0.014–0.033 m | 0.025 / 0.123 m | 0.026 m |

Earlier runs under zink (`diag-calibrated-*`, surveys v7–v9, `pitch-probe`,
`opening-probe`, `latency-probe`, `streak-probe`) are kept as the record of the
investigation.

## 3. Candidate maps

`survey-v10-coverage-fixed-render`, `replay-v10-*` and `survey-v11-coverage-production`
use the 34-step coverage plan (`amr_route_planner/config/survey_coverage.json`,
163 m). Each was evaluated with `calibrate_registration`, `build_graph` and
`evaluate_map.py`.

| Candidate | Survey error final / p90 / max | Registration (median, inlier 0.20) | Worst 4 m tile | Loop return | Graph (nodes / connections / stations / station pairs / cycles) | Result |
|---|---|---|---|---|---|---|
| `survey-v10` (live, Phase 5 loop closure) | 4.19 / 2.89 / 5.23 m | rejected: 0.148 m, 0.57 | — | — | — | FAILED (false loop closures) |
| `replay-v10-phase5-params` | 0.46 / 0.98 / 1.58 m | rejected: yaw vs prior | — | — | — | FAILED |
| `candidate-v10-replay-loop-narrow` | 0.145 / 0.41 / 0.54 m | 0.069 m, 0.836 | 0.219 m | 0.145 m / 0.010 rad | 29 / 34 / 6 / 30 / 6 | local target missed by 0.019 m |
| **`candidate-v11`** (live, production) | 0.003 / 0.42 / 0.55 m | **0.065 m, 0.841** | **0.170 m** | **0.003 m / 0.001 rad** | 27 / 29 / **6** / **30** / **3** | **all engineering targets met** |

- Candidate v11: map version `09d81243…c575`, registration `788d6c32e623626e`, transform
  (11.065, −0.03, −0.006), graph `070036e0b9cfac97`.
- Blocked in v11, all near-limit design margins (`PHASE_5_1_REMEDIATION.md`):
  - nodes `north_cross_aisle__aisle_02` and `south_cross_aisle__aisle_02` (pillar
    guards, 5.8 cm design margin);
  - node `south_cross_aisle__west_service_aisle` (a sign 0.99 m away);
  - 8 connections through those nodes.
- The approved map was left unchanged until the operator accepted the candidate on
  2026-10-30 (section 9).

## 4. Navigation on candidate v11 (real Gazebo + Nav2 + Gateway, AMCL on the front scanner)

`route_integration.py --candidate candidate-v11 --scenario normal --initial 0 0 0`.
The normal scenario includes the walking workers human_01 and human_02.

| Run / step | Route | Result | Final error map / truth | Replans | Notes |
|---|---|---|---|---|---|
| `nav-stations` 0 | spawn → pickup_station_01 | COMPLETED | 0.184 / 0.21 m | 0 | |
| `nav-stations` 1 | **pickup_station_01 → dropoff_station_02** | COMPLETED | 0.174 / 0.191 m | 0 | 30.0 m along the main aisle |
| `nav-stations` 2 | dropoff_station_02 → pickup_station_02 | **FAILED** (exit 7) | — | 1 | `Route wall timeout of 600.0 s` at RTF 0.36 (216 s simulated). The last objective lies on human_01's patrol line. Zero contacts, minimum envelope distance 0.66 m. Root cause and fix: wall-clock deadline → simulation-time deadline |
| `nav-stations-2` 0 | spawn → pickup_station_02 | COMPLETED | 0.190 / 0.278 m | 1 (Nav2 failure while the worker crossed the objective) | waited out human_01; 10 recoveries |
| `nav-stations-2` 1 | **pickup_station_02 → dropoff_station_01**, operator closes `main_cross_aisle__aisle_03--main_cross_aisle__east_service_aisle` 25 s into the route | COMPLETED | 0.197 / 0.176 m | 1 (`restriction`) | **live alternative route**: 31.8 m plan → 38.8 m via `south_cross_aisle__aisle_03`, `south_cross_aisle__east_service_aisle` |
| `nav-stations-2` 2 | dropoff_station_01 → staging_area | COMPLETED | 0.172 / 0.139 m | 0 | 31.1 m |
| `nav-stations-2` 3 | **staging_area → pickup_station_01** | COMPLETED | 0.199 / 0.19 m | 0 | |
| `nav-stations-2` 4 | pickup_station_01 → dropoff_station_02 with that connection **pre-blocked** | COMPLETED | 0.185 / 0.245 m | 0 | **alternative route** via the north cross aisle, 36.2 m |
| `nav-stations-2` 5 | **dropoff_station_02 → charging_station** | COMPLETED | 0.191 / 0.216 m | 0 | |

- Contacts across both runs: **0 AMR contact samples, 0 episodes**.
- Worker yielding in `nav-stations-2`:
  - human_01: 2 events, 31.2 s in total, minimum 0.49 m;
  - human_02: 2 events, 37.2 s in total, minimum 0.77 m;
  - 0 reversals.
- Command ownership: `navigation_command_guard` is the only `/cmd_vel` publisher in
  every route report.
- Digital Twin browser (`nav-stations-2/browser.json`, `route.webm`):
  - 876 samples, no page errors;
  - route overlay visible, 3 human models;
  - the AMR moved 22.3 m in the viewer;
  - statuses including REPLANNING and COMPLETED observed.

  The Playwright test itself timed out after 28 min, before the 60 min run ended, so
  the final viewer-vs-truth comparison was not executed. The observation window now
  scales with the number of steps (wall clock, up to 2 h).

## 5. Sensor coverage and worker yielding (real messages; Gazebo poses for evaluation only)

### Real-sensor coverage

`validate_sensor_coverage.py` (`sensor-coverage`, **PASSED**). Detection means a real
scan endpoint within 0.45 m of the worker, counted in scans with the worker within
3 m.

| Worker passes the AMR at | front `/scan` detections | 360° `/top_scan` detections | closest detected |
|---|---|---|---|
| front | 113 / 113 (100 %) | 75 / 75 (**100 %**) | 1.69 m |
| rear | 0 / 89 (out of field of view, as designed) | 58 / 58 (**100 %**) | 1.72 m |
| left | 78 / 213 (37 %, forward part only) | 140 / 140 (**100 %**) | 1.54 m |
| right | 37 / 213 (17 %, forward part only) | 140 / 140 (**100 %**) | 1.54 m |

- Static structure: `/top_scan` returns in all four quadrants (151–162 points each).
  Plot: `sensor-coverage/sensor-coverage.png`.
- The first run reported `passed: false` from a bug in the script's static check: it
  read a sample without stored points. The real data passed. The script was fixed
  and rerun.

### Worker yielding

`validate_crossing_contact.py --mode yield` (`crossing-yield`, **PASSED**), production
yielding, AMR placed across human_01's line:

- free walking speed 0.96 m/s;
- yield began 2.07 m from the AMR envelope; the worker came to rest 2.11 m clear;
- resting speed 0.009 m/s, gait at rest;
- after the AMR left, the worker resumed at 0.89 m/s with a walking gait;
- closest approach 0.97 m, **0 contacts**, 0 reversals.

Two earlier attempts failed on **test assumptions**, kept as
`crossing-yield-attempt1` and `crossing-yield-attempt2`; HumanSystem was not changed:

- a fixed 6 s wait does not cover a braking-consistent approach from 2 m;
- a fixed later gait window fell into the worker's normal pause at the end of its
  segment.

### Intentional contact (yielding disabled)

`validate_crossing_contact.py --mode contact` (`crossing-contact`, **PASSED**, original
assertions unchanged, `human_yielding:=false` TEST MODE): 16,728 contact samples,
stopped displacement 0.0 m, walking gait span 0.47.

The bounded-wait reversal was **not triggered** in any run (0 reversals); every yield
released on its own. It is therefore implemented but not exercised.

### Nav2 front / left / right / rear

`nav2-costmap-sectors.json` covers the 4 runs that used the corrected observer:
`nav-stations-2` and the three `person-crossing` runs. Each sample has a worker within
3 m. The costmap counts as marked when the maximum local cost within 0.4 m of the
worker is ≥ 99.

| Sector (bearing from heading) | Samples | Local costmap marks the worker | Front scan detects | 360° scan detects |
|---|---|---|---|---|
| front (±45°) | 545 | 537 (**99 %**) | 538 | 540 |
| left (45–135°) | 682 | 681 (**100 %**) | 198 | 675 |
| right (−45…−135°) | 906 | 905 (**100 %**) | 284 | 902 |
| rear (beyond ±135°) | 1,812 | 1,807 (**100 %**) | 0 | 1,796 |

Rear and most side marking comes from `top_obstacle_layer`, the 360° scanner: the gap
behind the Phase 5 contacts is closed.

## 6. Dynamic worker acceptance on candidate v11 (`person_crossing`)

Same three steps as Phase 5 `run2-person-crossing`:

1. → `pickup_station_01_via_west_service_aisle`;
2. → `main_cross_aisle__aisle_01`;
3. → `pickup_station_01_via_west_service_aisle`.

The routes cross human_01's walking line at x = −8.7. Timing was varied with
`--step-delay`.

| Run | Routes | Contacts (samples / episodes) | human_01 yielding | Closest approach |
|---|---|---|---|---|
| `candidate-v11-person-crossing-delay0` | 3/3 COMPLETED (0.173–0.187 m) | **0 / 0** | 4 events, 46.1 s | 0.48 m |
| `candidate-v11-person-crossing-delay12` | 3/3 COMPLETED (0.155–0.199 m) | **0 / 0** | 4 events, 45.6 s | 0.46 m |
| `candidate-v11-person-crossing-delay30` | 3/3 COMPLETED (0.164–0.181 m) | **0 / 0** | 4 events, 40.6 s | 0.43 m |

Phase 5's equivalent run failed with worker-initiated contacts from the side and
rear. Together with the normal-scenario station runs, **0 contacts in 5 runs and 20
routes**.

Collision Monitor: evaluated, not adopted. With both scanners in the costmaps and
worker yielding, no AMR-initiated contact remained. Adding it would introduce a
second `/cmd_vel` stage in front of the fail-closed guard, and nothing measured
justifies that.

## 7. Performance (measured)

| Metric | Value | Source |
|---|---|---|
| Real-time factor, mapping world, static | 0.76 (nouveau GL) vs 0.62 (zink) | `static-scan-capture/summary.json` |
| Real-time factor, full stack (Gazebo + Nav2 + Gateway + observers) | 0.34–0.37 (0.36 with the browser) | `*/sensors.json` |
| Front scan rate / 360° scan rate (sim) | 15.1–15.2 Hz / 10.0 Hz | `*/sensors.json` |
| CPU, % of one core, median (8 cores), `person-crossing-delay0` | Gazebo 167, route executor + guard 95, Nav2 58, Gateway 56, bridge 19, AMCL 6 | `resources-person-crossing.json` |
| Resident memory, median | Gazebo 1.17 GB, Nav2 412 MB, executor + guard 216 MB, Gateway 206 MB, bridge 168 MB, AMCL 59 MB | `resources-person-crossing.json` |
| Viewer FPS (headless Chromium, under full load) | median 7.4 (6.4–8.4); 27k triangles, 61 draw calls | `candidate-v11-nav-stations-2/browser.json` |
| SLAM survey real-time factor (mapping only) | about 0.36 live; replay at 2.5× ran at RTF 0.92 | `survey-v11-*`, `replay-v10-*` |

## 8. Regression

| Suite | Result | Evidence |
|---|---|---|
| Phase 4 fault suite: cancel, outside-map (204), occupied-goal (208), timeout, pause, localization-loss (102), controller-inactive (107), **clock-bridge-loss** | **8/8 PASSED**: world-stationary over the final 2 s, zero commands, Gazebo confirmed paused where applicable | `regression-phase4-fault-suite/*/report.json` |
| Phase 4 fault suite: gazebo-reset | **FAILED, pre-existing**: same signature as Phase 4's own evidence (`evidence-phase4/faults-verified/gazebo-reset`), "Action response deadline exceeded" with no final action status. The robot is stationary with zero commands and Gazebo is paused, so the safety outcome holds; only result reporting after a world reset fails | `regression-phase4-fault-suite/gazebo-reset` |
| Phase 2 Mosaico round trip (`validate_phase2.py`) | **PASSED** (first attempt below) | `regression-phase2-mosaico` |
| Phase 2 browser checks | 4 passed, 5 skipped (those need owned runs). The first attempt failed the scene geometry budget: 56 > 52 (Phase 3 scene 50 + Phase 5 route overlay 4 + Phase 5.1 top-scanner meshes 2). The leak budget was updated to 58 with that breakdown | `regression-phase2-mosaico-geometry-budget/browser.log` |
| Workspace build | `colcon build --symlink-install`: 8 packages; no generated-file drift | — |
| `build_graph --check` | approved artifacts consistent before and after promotion | — |

Phase 2 round-trip detail:

- real recording completed in 107.9 s (31 records per wall-clock second);
- the storage-outage and unavailable-at-start cases fail in isolation while the
  Gateway stays online;
- the recording is retrieved after the storage services restart;
- an incomplete sequence is rejected.

## Automated tests

| Suite | Result |
|---|---|
| `amr_navigation` pytest | 28 passed (adds `test_audit_detects_blind_sectors`, top-scan guard and costmap tests) |
| `amr_route_planner` pytest | 26 passed. After promotion, `test_direct_connector_requires_a_map_check` was made map-independent: it relied on the Phase 4 map leaving the spawn's lanes unvalidated, so it now closes them explicitly and expects the nearest available node |
| `digital_twin_gateway` pytest | 48 passed |
| frontend `npm run build`, `vitest` | build OK, 16 passed |
| AMR generation (Blender) | `validate_amr.py` / `test_rebuild.py` / `validate_urdf.py` pass: 18 links, 4 sensors, 720-ray self-occlusion clear, front bay ±98.1° |

## 9. Map promotion (operator-approved 2026-10-30) and verification

The operator approved `candidate-v11` when asked explicitly. Steps:

1. archived the Phase 4 v3 package (`approved-map-v3/`: map, provenance, registration,
   graph);
2. installed `maps/warehouse.{yaml,pgm}` and a new `provenance.json` (SLAM file hash,
   survey plan, evidence, quality at approval);
3. `calibrate_registration` → `788d6c32e623626e` (identical to the candidate);
4. `build_graph` → `31fba3aa050bd251`; `build_graph --check` passes.

| Approved map | Phase 4 v3 (before) | Phase 5.1 v11 (after) |
|---|---|---|
| registration id | `87e82b11cf133151` | `788d6c32e623626e` |
| transform x, y, yaw | 11.0375, 0.065, 0.0145 | 11.065, −0.030, −0.006 |
| occupied cells fitted | 3,722 | 13,935 |
| registration median / p90 / inlier 0.20 | 0.056 / 0.324 m / 0.807 | 0.065 / 0.258 m / 0.841 |
| worst 4 m tile / p90 tile / tiles > 0.20 m | 1.102 / 0.198 m / 3 | **0.170 / 0.107 m / 0** |
| known free area | 346 m² | 565 m² |
| validated nodes / connections | 4 / 3 | **27 / 29** |
| validated stations | 0 | **6** |
| reachable station pairs | 0 | **30** |
| independent validated cycles | 0 | **3** |

The global median rises slightly (0.056 → 0.065 m) because the fit now covers the
whole warehouse (3.7× the cells); every local measure improves.

Verification on the approved map (`approved-map-v11-route-verification`, no
`--candidate`, normal scenario, browser):

- spawn → pickup_station_01 COMPLETED, 0.177 m;
- pickup_station_01 → dropoff_station_02 COMPLETED, 0.189 m (truth 0.193 m), 30.0 m;
- 0 contacts;
- the Gateway serves graph `31fba3aa050bd251`;
- **browser test passed** (401 samples, no page errors, final viewer-vs-truth
  comparison within 0.6 m).

A first attempt (`…-system-python-build`) did not start: a full `colcon build` run
without `.venv` active had rewritten console-script interpreters (Gateway: "No module
named uvicorn"). Rebuilding as documented in `DEVELOPMENT.md` (`.venv` active)
restored them. Every earlier result predates that build.

## 10. Acceptance status

| Area | Item | Status |
|---|---|---|
| Audit | baseline audit, separate untouched `3bdc33a` build for "before" measurements | VALIDATED |
| Problem A | root cause: simulated LiDAR rendering defect (zink-on-NVK), proved by per-beam ray cast and by the untouched Phase 5 stack mapping to 0.020 m once fixed | VALIDATED |
| Problem A | rendering fix (`NOUVEAU_USE_ZINK=0` in `sim.launch.py`), fail-closed blind-arc audit, `validate_lidar_rendering.py` | IMPLEMENTED, VALIDATED |
| Problem A | odometry calibration (`odometry_wheel_separation` 0.5258 m through the generator) | IMPLEMENTED, VALIDATED (yaw error 0.000 rad / 90°) |
| Problem A | SLAM: Phase 5 matcher unchanged; loop search narrowed to 1.5 m / 3 m against aisle aliasing | IMPLEMENTED, VALIDATED (v10 vs replay vs v11) |
| Problem A | matcher tuning and odometry-pose profile developed under the defect | REVERTED / WITHDRAWN (kept as evidence) |
| Mapping | `mapping_jog` refuses unobserved rotations; turns at junctions; tracked survey plans; `--bag` plus offline replay | IMPLEMENTED, VALIDATED (v11 completed 34/34 legs) |
| Mapping | candidate workflow: never auto-overwrites, no painted cells, no SDF occupancy, operator approval | IMPLEMENTED, VALIDATED (approval requested and given) |
| Mapping | engineering targets: loop ≤ 0.25 m / 0.05 rad, global ≤ 0.10 m, local ≤ 0.20 m | VALIDATED for v11 (0.003 m / 0.001 rad, 0.065 m, 0.170 m) |
| Mapping | six station approaches; west–east connectivity; alternative route via a cycle | VALIDATED (graph and real Nav2: all six stations reached; live and pre-blocked alternatives) |
| Mapping | north/south cross aisles | PARTIALLY VALIDATED: east and west parts validated and driven (charging, dropoff alternatives); the aisle_02 nodes are blocked by real pillar guards at a 5.8 cm design margin, so neither cross aisle is continuous west–east in the graph |
| Mapping | registration and graph regenerated after acceptance, before/after table | VALIDATED (section 9) |
| Mapping | hard-coded registration offsets removed (`coordinate_registration.active()`) | IMPLEMENTED |
| Problem B | 360° elevated scanner (option C) through the generation pipeline (parameters, Blender, URDF, Gazebo, bridge, GLB) | IMPLEMENTED, VALIDATED (100 % worker detection front/left/right/rear) |
| Problem B | Nav2 observation sources per scanner; costmap marks workers front/left/right/rear | IMPLEMENTED, VALIDATED (99–100 % per sector) |
| Problem B | guard stays fail-closed, also requires `/top_scan` | IMPLEMENTED, unit-VALIDATED; all runtime audits passed |
| Problem B | HumanSystem yielding (forces only, gait, hysteresis, bounded wait) | IMPLEMENTED, VALIDATED (yield test; 0 contacts) |
| Problem B | deadlock reversal | IMPLEMENTED, NOT EXERCISED (no run needed it) |
| Problem B | intentional-contact test preserved with yielding disabled | VALIDATED |
| Problem B | Collision Monitor | DEFERRED (evaluated; no AMR-initiated contact remains; would add a competing `/cmd_vel` stage) |
| Navigation | station-to-station routes: pickup_01 → dropoff_02, pickup_02 → dropoff_01, staging → pickup_01, dropoff_02 → charging | VALIDATED (candidate runs; priority route also on the approved map) |
| Navigation | route deadline in simulation time | IMPLEMENTED, VALIDATED (`nav-stations-2`) |
| Navigation | `plan_deviation_tolerance_m` 1.0 baseline; no clearance reduced | unchanged |
| Safety | dynamic worker, zero contacts across runs and timings | VALIDATED (0 contacts, 5 runs, 20 routes, 3 timings) |
| Regression | Phase 4 fault suite incl. clock-bridge loss | VALIDATED 8/9; gazebo-reset FAILED (pre-existing, identical in Phase 4 evidence) |
| Regression | Phase 2 Mosaico round trip; Gateway, frontend and unit suites; workspace build | VALIDATED |
| Performance | RTF, scan rates, CPU, memory, viewer FPS | VALIDATED (measured, section 7) |

## 11. Known limitations

1. **Host-specific renderer workaround.** `NOUVEAU_USE_ZINK=0` fixes Mesa
   zink-on-NVK. Other drivers ignore it. Any other renderer defect is caught by the
   audit's blind-arc check and by `validate_lidar_rendering.py`; run the latter on each
   new host.
2. **Near-limit graph nodes.** `north/south_cross_aisle__aisle_02` (pillar guards) and
   `south_cross_aisle__west_service_aisle` (a sign) have less than 6 cm of design margin,
   so they validate or block depending on centimetre-level map differences. Moving
   them is a layout change, recommended for Phase 6.
3. **Asynchronous loop closure is timing-sensitive.** The same bag gave different
   outcomes live and in replay (v10). Narrowing the search removed false closures in
   the replay and in v11, but candidates must still pass the objective gates; repeat
   surveys when the layout changes. Live estimates reached 0.55 m before closures
   corrected them.
4. **gazebo-reset fault case** (pre-existing): navigation action result reporting
   after a world reset times out, although the robot stops safely.
5. **Throughput.** The full stack runs at RTF ≈ 0.35 on this host. Route deadlines are
   now in simulation time, so outcomes no longer depend on it.
6. **Worker deadlock reversal** is implemented but was never needed in testing.
