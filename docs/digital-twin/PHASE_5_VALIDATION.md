# Phase 5 validation record

Executed checks only, including failures. Raw evidence lives in the ignored
`docs/digital-twin/evidence-phase5/` directory (same policy as earlier phases);
paths below are relative to it.

## Environment and baseline

- Branch `feature/warehouse-graph-routing` from Phase 4 `5ca6a95` (not in `main`);
  `git fetch` first; clean tree; no commits or pushes.
- Ubuntu 24.04, ROS 2 Jazzy, Nav2 1.3.13 (incl. `nav2_route` 1.3.13 — evaluated, not used),
  SLAM Toolbox 2.8.5, Python 3.12 (`.venv` with system site packages), Node 22.23.3 (`.tools`).

## 5A–5C — graph, costs, registration, approved-map coverage

- Registration fit on the approved map (3 722 occupied cells, 158 landmark boxes):
  `x 11.0375 m, y 0.065 m, yaw 0.0145 rad`; median residual 0.056 m, 80.7 % of cells
  within 0.20 m. The provenance `+11 m` offset gives 0.073 m / 77.9 %. The rotation is
  significant: at 15 m it moves points by ≈0.22 m. An independent scratch
  implementation reproduced the same transform.
- Generated graph: 30 nodes (15 intersections, 6 stations, 2 open areas, 1 spawn,
  6 approaches), 37 connections / 74 directed edges, one topology component, no
  structural errors. `build_graph --check` reproduces the committed artifact byte for byte.
- **Approved map coverage (strict criteria)**: 4 nodes and 3 connections validated
  (main cross aisle between the west service aisle and aisle_02, and the west-service
  segment to the pickup_station_01 approach). **No station is validated**; no validated
  loop exists. Even the spawn's rotation disc contains 15 unknown cells (never observed
  behind the start pose). Evidence: `graph-approved-map-v3/graph-on-map.png`,
  `graph-approved-map-v3/graph-validation.json`.

## Mapping prerequisite — coverage extension attempts

All surveys reuse the Phase 4 infrastructure (worker-free `mapping_only` world, SLAM
Toolbox resumed from the approved serialized pose graph with `map_start_pose [0,0,0]`,
`mapping_jog.py` legs). After each leg, SLAM is compared with independent Gazebo
poses through the verified registration (evaluation only).

| Survey | Route | Outcome |
|---|---|---|
| `survey-v4` | full perimeter loop | **Rejected.** 16.8 m westward north-cross-aisle pass: 0.49 m disagreement; candidate shows doubled north/west walls and NW racks shifted ≈0.3–0.5 m |
| `survey-v5-startup-race` | — | Aborted: initial TF check ran before the 55 MB pose graph loaded (fixed: retry until SLAM TF exists) |
| `survey-v5` | short loops | **Rejected.** North aisle 4.2 m: 0.40 m; south aisle 4.2 m: 0.78 m. Vertical passes stayed 0.20–0.23 m and recovered at the main junction |
| `survey-v6-comb` | vertical out-and-back passes only | **Rejected.** During the in-place 180° turn at (8.1, 8.5), facing the north wall, the robot did not translate but the SLAM estimate moved 0.54 m and its yaw went wrong by ≈0.07 rad. The heading-hold then drove the robot 1.3 m east of the aisle centre (independent ground truth). Disagreement: 2.02 m |

Diagnosis: two mechanisms, both observed with independent poses.

1. Longitudinal travel along a cross aisle drifts. Its only structure is a plain wall
   on one side and rack ends repeating every 4.2 m on the other, so scan matching
   aliases along x. This is the same failure Phase 4 recorded for its northern pass.
2. In-place turns where the 180° scan sees mostly a flat wall corrupt the estimate,
   even with no translation.

Vertical aisle passes and main-aisle travel stay at 0.18–0.23 m. Rejected candidates
are kept for review and were never promoted. **Decision:** automated survey attempts
stopped after four candidates. Phase 4 defines mapping as an operator-supervised RViz
procedure, and SLAM must not be redesigned. Extending the approved map is therefore a
documented prerequisite (see the recommendations in the final section), not something
claimed. All Phase 5 runtime validation uses the unchanged approved map and only its
validated graph elements.

## Test A — graph route planning (no motion)

`plan_route --from main_cross_aisle__aisle_02 --to pickup_station_01_via_west_service_aisle --algorithm both`
(`test-a-plan.json`, `test-a-route.png`). Both algorithms return the same validated route
`aisle_02 junction → aisle_01 junction → west-service junction → pickup_station_01 approach`:
13.4 m, cost 15.4196. That equals 13.4 m plus one π/2 turn at 1.2857 m/rad, checked
by hand. Objectives: the west-service junction (turn) and the destination. A* expanded
4 states, Dijkstra 5.

## Tests B and E, cancellation and Digital Twin — real Gazebo/Nav2 (`run1-normal`)

Graph version during this run: `2c7f8be6c77f3cb8`. Widening the deviation tolerance
afterwards changed only the planner-config hash. The current graph is
`0e127f48069cca0a`, with identical topology and map validation.

`route_integration.py --scenario normal --initial 0 0 0 --browser`. The harness owns one
simulation, Gateway and Nav2 stack with the existing guard, the existing sensors and
AMCL on the approved map. It publishes no velocity. Result: **passed**. All six steps
matched their expected exit codes, the browser assertions passed, the existing
contact sensors recorded **0 AMR contact samples**, and all owned processes stopped.
RTF was 0.37.

| Step | Request | Outcome | Nav2 objectives | Route / travelled (ground truth) | Arrival error map / ground truth | Corridor conformance (GT) | Replans / recoveries |
|---|---|---|---|---|---|---|---|
| 0 | spawn → aisle_02 junction (A*) | COMPLETED | west-service junction (corridor change), destination | 10.68 m / 10.53 m | 0.177 m, 0.11 rad / 0.176 m | 100 % | 0 / 0 |
| 1 | → pickup_station_01 approach (A*) | COMPLETED | west-service junction (turn), destination | 13.4 m / 12.84 m | 0.181 m / 0.129 m | 100 % | 0 / 0 |
| 2 | → aisle_01 junction (**Dijkstra**) | COMPLETED | west-service junction (turn), destination | 9.2 m / 8.42 m | 0.187 m / 0.186 m | 100 % | 0 / 0 |
| 3 | → approach with `--block` on its only connection | BLOCKED `no_route` (exit 3) | none dispatched | — / 0.00 m | — | — | 0 / 0 |
| 4 | → approach; operator closure published at 22 s | EXECUTING → cancel acknowledged → REPLANNING → BLOCKED (exit 3) | cancelled | 9.2 m / 0.00 m (still turning in place) | — | 100 % | 1 / 0 |
| 5 | → aisle_02 junction, `--cancel-after 14` | CANCELED (exit 6), cancel acknowledged, robot stopped | cancelled | 4.2 m / 0.00 m | — | — | 0 / 0 |

- Every Nav2 `/plan` received during execution stayed inside the route lanes: 0
  points outside across all plan checks. The AMR therefore followed the graph's
  corridor, not merely reached the destination.
- Runtime registration check (evaluation only): the AMCL map pose against the registered
  ground truth had a median of 0.047–0.065 m per route. This confirms the
  calibration on the live stack.
- The `/cmd_vel` publisher set was exactly `navigation_command_guard`
  (`TwistStamped`) before every dispatch.
- Gateway `/api/v1/routes/status` was sampled at 1 Hz throughout
  (`gateway-route-status.jsonl`). The browser saw the overlay in 327 of 344 samples,
  through every state: PLANNING, EXECUTING, COMPLETED, REPLANNING, BLOCKED and CANCELED.
  `route-*.png` and `route.webm` hold the evidence. Three.js ran at a median of
  8.0 FPS (minimum 6.6) in headless Chromium while the simulation ran on the same host.
  The human models reported 3 and the browser AMR matched the Gazebo pose.
- A first attempt (`run1-map-version-false-positive`) **failed closed**: `map_server`
  publishes a float32 resolution (0.0500000007) and my content version rounded to 9
  decimals. Fixed by rounding map geometry to 6 decimals, with a test through ROS
  serialization. A second attempt (`run1-amcl-latched-pose-missed`) failed closed
  with `localization_unavailable`: AMCL latches its estimate and republishes only
  after motion, so a late volatile subscriber never saw it. Fixed by subscribing
  transient-local as well. Both failed runs are kept.
- Dev-stack repetition (`dev-stack/step0–5.json`, same commands, no browser, RTF higher):
  identical outcomes. Arrival errors were 0.176–0.191 m, ground-truth errors 0.08–0.19 m,
  and execution took 75–119 s wall.

## Test F — dynamic obstacle (`person_crossing`): routes completed, **contact criterion FAILED**

`human_01` repeatedly walks x = −8.7, y ∈ [−2.5, 2.5] at 1.1 m/s. That is exactly
through the west-service/main junction where every validated route turns. The worker
is scripted and never yields. Three routes ran in each run: spawn → pickup_station_01
approach → aisle_01 junction → approach.

| Run | Deviation tolerance | Routes | Graph replans | Same-route retries | AMR contact episodes (samples) |
|---|---|---|---|---|---|
| `run2-person-crossing-tolerance-0.35` | 0.35 m | 3/3 COMPLETED | 0 | 2 (1 plan deviation around the worker, 1 Nav2 abort 104) | 5 (15 235) |
| `run2-person-crossing` | 1.0 m | 3/3 COMPLETED, 0 plan points outside lanes, 100 % ground-truth corridor conformance, arrival error 0.18–0.19 m | 0 | 0 | 2 (5 439) |

Contact sensors do not say who initiated a contact. The independent evaluation of
worker pose against the robot's commanded and measured velocity, its scan and its
costmap (`obstacles.json`) shows the following:

- Whenever the worker was in the front LiDAR field of view, it appeared in the
  scan and the global/local costmap (cost 100), and Nav2 stopped the AMR (commanded
  0, 0). In the 0.35 m run, the worker then walked into the **stationary** AMR twice
  and pushed it for up to 10 s.
- The two contacts in the 1.0 m run were worker-initiated from outside the sensor
  coverage. In one, the worker approached the left side (≈77° from the robot centre,
  ≈116° from the forward-mounted laser, beyond its ±90°) while the AMR rotated at the
  junction. In the other, the worker caught up from behind (≈−155°) at 1.1 m/s while
  the AMR drove away at 0.45 m/s.
- No corridor was closed and no graph reroute happened, which is correct: a person
  never blocks a whole corridor cross-section, so there was no costmap evidence. The
  0.35 m run's stop-and-wait inside the junction increased exposure. That is why the
  tolerance was widened to 1.0 m, which still catches every aisle bypass.
- **Conclusion:** local obstacle response (Nav2 stop/slow) and graph-level behaviour
  (no reroute without corridor evidence) are distinguished and both behave as designed.
  However, **zero contacts is not achieved**. The AMR has no side or rear perception
  (Phase 4 front 180° laser), and the simulated worker does not yield. Phase 5 must not
  use worker ground truth to avoid contacts, and must not alter Phase 4 safety
  mechanisms to pass. This remains an open safety limitation (see recommendations).

## Planning performance (measured, identical inputs)

`planning-benchmark.json`: every ordered pair of usable nodes, 20 repeats, wall-clock
medians, 8-core host, simulation idle except for one background run.

| Graph | Pairs | A* median / max | Dijkstra median / max | Mean expanded states A* / Dijkstra |
|---|---|---|---|---|
| validated subgraph (approved map) | 12 | 51 µs / 66 µs | 63 µs / 70 µs | 2.7 / 3.2 |
| full topology (`--topology-only`) | 870 | 79 µs / 309 µs | 139 µs / 403 µs | 9.2 / 27.8 |

The costs agree on all 882 pairs. A* expanded fewer states on 828 of the 870 topology
pairs and equal states on the other 42. On the tiny validated subgraph the difference
is within noise, so no practical speed claim is made there. Graph construction plus
map validation takes a median of 0.164 s (5 builds).

## Regression (real stack)

- **Phase 4 navigation** (`regression-phase4-navigation`, Phase 4's own `integration.py`,
  goals (2,0,0), (5,1,π/2), (10,−1,0)): **passed**. `NavigateToPose` reached every goal
  with errors of 0.177 / 0.206 / 0.173 m and 0.129 / 0.073 / 0.078 rad, 0 recoveries,
  robot stopped, `navigation_command_guard` the sole `/cmd_vel` publisher, 0 contact
  samples. The additional `navigate_through_poses` navigator does not change
  `NavigateToPose` behaviour.
- **Phases 1–3** (Gateway, WebSocket v1/v2, scene, real-time synchronization, humans):
  covered by the recorded `run1-normal`. Browser assertions passed: 4 synchronized
  entities, 3 human models, AMR displacement > 0.5 m, final browser AMR within 0.6 m of
  the Gateway pose, no page errors. The Gateway's unit suite covers v1/v2 contracts
  unchanged.
- **Not re-run live:** the Phase 4 clock-loss fault suite and the Phase 2 Mosaico
  storage round trip. Phase 5 does not modify the command guard, the controller,
  Gazebo control, the recorder or storage, and their unit suites pass. A live re-run
  remains recommended before merging.

## Automated tests (current)

Final run on the code in the working tree (`tests/*.txt`):

- `amr_route_planner`: 26 passed (`test_search`, `test_geometry`, `test_warehouse`):
  lattice and hand-computed optima, 40 random graphs cross-checked against exhaustive
  enumeration with turn costs, A* vs Dijkstra equality and returned-path cost
  consistency, disconnected/blocked/one-way/invalid endpoints, determinism,
  structural defects, scenario-specific zones, registration binding/tampering/fit
  recovery, trinary loading, unknown/occupied/clearance/outside rejection,
  footprint-vs-point-robot gap, deterministic generation equal to the committed
  artifact, staleness rejection, start attachment/off-graph/retreat, costmap
  blockage evidence, executor messages against the Gateway schema, live float32
  `/map` versus YAML version, direct start connector, exclusion of the robot's own cells.
- Gateway: 48 passed (45 existing + 3 route tests); schemas regenerated.
- `amr_navigation`: 25 passed (adds the NavigateThroughPoses tree check).
- Recorder and warehouse: 43 passed, 1 skipped (unchanged).
- Frontend: 16 passed (13 existing + 3 route tests); `npm run build` passes (type
  drift check, `tsc`, Vite; existing large-chunk warning); the no-backend browser test
  passes with the new route panel.
- `export_schemas.py --check` and `build_graph --check`: no drift.

## Acceptance status

| Area | Item | Status |
|---|---|---|
| Graph | generated from the authoritative layout; stable ids; stations; junctions; no impossible connections; disconnected-region reporting | IMPLEMENTED, unit-VALIDATED |
| Algorithms | Dijkstra, A*, deterministic costs, restrictions, blocked edges, invalid destinations, no-route | IMPLEMENTED, unit-VALIDATED; A*/Dijkstra both executed on the real stack |
| Mapping | approved SLAM map used; registration verified (fit + runtime median 0.047–0.081 m); unknown rejected; footprint and clearance | VALIDATED |
| Mapping | station approach poses validated | PARTIALLY VALIDATED: the machinery classifies all six; **none passes** on the approved map |
| Mapping | coverage extension | BLOCKED: four automated SLAM candidates rejected for drift (evidence kept) |
| Nav2 | existing Nav2 functional; map-frame objectives; Nav2 drives; progress; cancellation; failure reporting; guard sole publisher | VALIDATED (`run1-normal`, `regression-phase4-navigation`) |
| Nav2 | replanning to a validated alternative | PARTIALLY VALIDATED: closure → cancel → replan → BLOCKED is validated in simulation; choosing an alternative is unit-tested only, because the approved map has no validated loop |
| Digital Twin | routes visualized and tracked; status available; map → world transform; synchronization; humans | VALIDATED (`run1-normal` browser) |
| Validation | multi-waypoint route with real Nav2 | VALIDATED (3 routes, 2 objectives each) |
| Validation | blocked-route and safe no-route | VALIDATED (static and mid-route closure; nothing dispatched or safely cancelled) |
| Validation | station-to-station; six-station operation | BLOCKED by map coverage (no validated station) |
| Validation | dynamic obstacle, zero contacts | **FAILED**: worker-initiated contacts from outside the sensor field of view |
| Validation | Phase 4 navigation regression | VALIDATED |
| Validation | live fault-suite and storage regressions | DEFERRED (unchanged code; unit suites pass) |

## Known limitations and recommended prerequisites for Phase 6

1. **Map coverage.** Extend the approved map with the Phase 4 operator-supervised
   procedure (RViz), not automated jogs. Measured lessons:
   - do not travel along the north/south cross aisles (aliasing rack ends drift
     0.4–0.8 m per 4.2 m);
   - do not rotate in place facing the end walls (0.5 m / 0.07 rad jumps);
   - map those areas from vertical aisle passes, and turn in the feature-rich main aisle;
   - accept a candidate only after `calibrate_registration --map …`, tile residuals
     and a ghosting review, then `build_graph --map … --registration …`.

   After approval, replace `maps/warehouse.*` together with `provenance.json`,
   recalibrate, rebuild the graph, and re-run localization and navigation validation
   before Tests C/D.
2. **Side and rear perception.** The front 180° laser cannot observe people
   approaching from the side or rear. Fleet operation (Phase 6) needs added
   perception (e.g. a rear or 360° scan) or a collision monitor fed by such sensors.
   Ground truth must not be used.
3. **Human models** in simulation do not yield. Contact metrics in shared areas mix
   robot behaviour with scripted pedestrians; add yielding behaviour or evaluate with
   that explicitly in mind.
4. The Route Server (`nav2_route` 1.3.13) remains an option for Phase 6 traffic
   management. The graph can be exported to its GeoJSON format if adopted.
5. `/route_planner/status` uses `std_msgs/String` with a versioned, schema-validated
   JSON document, because there is no interface package. A typed interface package
   would be preferable once multiple robots publish route state.
