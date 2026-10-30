# Phase 5 — warehouse graph route planning

The new `amr_route_planner` package decides the **operational** route between named
warehouse locations. Nav2 remains the physical navigator: it plans collision-aware
geometry between the graph objectives and drives through the unchanged Phase 4
command guard. No second controller, velocity publisher or localization source
is introduced.

```text
station request ─► graph planner (A*/Dijkstra, costs, restrictions)
                ─► validated route (approved map, footprint, registration)
                ─► route executor ─► Nav2 NavigateThroughPoses ─► guard ─► ros2_control ─► Gazebo
                ─► /route_planner/status ─► Gateway /api/v1/routes/* ─► Three.js overlay
```

## Baseline

Phase 4 (`5ca6a95`, branch `feature/amr-autonomous-navigation`) is not merged into
`main`, so `feature/warehouse-graph-routing` was created locally from `5ca6a95`
after `git fetch`. The working tree was clean. Nothing is committed or pushed.

## Station semantics (audited)

`amr_simulation/config/warehouse_layout.json` is the single source of truth.
`station_poses.json` is a derived generator output (`goal_pose` when present,
else `pose`) and is not read by the planner.

| Field | Meaning | Used as |
|---|---|---|
| `pose` (x, y, yaw) | centre of the painted docking marking; the generator draws the dock cross here and places no collision geometry inside it | dock pose, final stop and yaw (all stations except staging) |
| `size` | marking footprint (visual) | visualization and access-lane width |
| `goal_pose` | navigation target where the area itself holds pallets (only `staging_area`) | dock pose for staging |

Every dock is checked against the compiled static geometry with the padded Nav2
footprint and 0.30 m clearance, and against the real map. The approach is a
corridor point on the nearest collision-free corridor; the access edge from that
point to the dock is validated like any other edge. Visual geometry and approach
metadata live in the graph artifact's `stations[]`, separate from graph nodes.

## Graph model and generation

`graph_builder.py` loads the layout **through the existing warehouse compiler**
(identical validation and `static_bounds`) and reads wall collisions from the
compiled, checksum-verified building model. Generation is deterministic; the same
inputs produce byte-identical `config/warehouse_graph.json`.

* **Nodes** (`STATION`, `INTERSECTION`, `CORRIDOR_POINT`, `OPEN_AREA`) carry stable ids,
  `position` in `world`, optional `yaw`, the warehouse entity and the corridors whose
  centreline contains them.
* **Junctions** exist only when two perpendicular corridors overlap or abut and the
  shared opening is at least the padded robot width plus clearance on both sides
  (1.38 m). Proximity alone never connects nodes; parallel overlaps are reported, not joined.
* **Edges** are directed. A bidirectional connection is two edges sharing a
  `connection` id; a closure removes both. Fields: `distance`, `base_cost`, `width`,
  `direction`, `corridor_id`, `kind` (`corridor`/`station_access`) and the configured
  restriction `zones` the robot sweep physically crosses.
* Station access candidates are generated from every corridor and rejected when the
  swept footprint hits static geometry (each rejection is recorded in
  `structure.notes`). Example: the camera mast at (−10, −8.5) rejects the south-aisle
  access to `staging_area`; its west-service access remains.
* Structural checks: unique ids, valid endpoints, no duplicates or self-loops, matching
  reverse edges, nonnegative finite costs, `base_cost ≥ distance` (keeps A* admissible),
  frame consistency, station references, connected components and isolated nodes.

NetworkX was evaluated and not added: it is not installed, and the needed graph
operations (directed adjacency, union-find components, a heap search over
(node, incoming-edge) states) are short and fully tested.

## Costs and restrictions

Cost unit: metres-equivalent of travel (`config/route_planner.yaml`).

```text
edge cost  = distance_weight · distance + configured static penalty   (base_cost, ≥ distance)
turn cost  = turn_weight_m_per_rad · |Δheading|                         at every node, start and dock yaw
turn_weight = desired_linear_vel / rotate_to_heading_angular_vel = 0.45 / 0.35 = 1.2857 m/rad
```

The turn term is necessary: a corridor grid has many equal-length Manhattan routes,
and the term selects the one with the least in-place rotation (stop-and-go). Hard
restrictions remove edges; they are never large costs. Corridor `capacity` is design
metadata, not live occupancy. Restriction sources:

| Source | Applies when | Effect |
|---|---|---|
| map validation status ≠ `validated` | always | node/edge excluded |
| `restricted_zones[]` | zone `enabled` | edges whose sweep crosses it excluded |
| `temporary_blocked_zones[]` | active scenario equals the zone's `scenario` | e.g. `aisle_01_block` closes only the aisle_01 south connection in `corridor_blocked` |
| `--block CONNECTION`, `/route_planner/block_connection` | operator | both directions excluded |
| costmap evidence during execution | Nav2 global costmap shows a corridor cross-section with no traversable cell | connection closed for `blocked_connection_ttl_s` (120 s) |

## Search

Dijkstra is the A* search with a zero heuristic. Both search (node, incoming edge)
states so turn costs stay exact. The A* heuristic is `distance_weight ×` straight-line
distance: admissible and consistent because every edge costs at least its straight
length and turn terms are nonnegative. Ties break on (f, g, node id, edge id), so
results are deterministic. Each plan records algorithm, nodes, edges, distance, cost,
expanded states, distinct visited nodes and planning time.

## Coordinate registration

Layout geometry is in `world`; Nav2 is in `map`. They are related by an explicit
SE(2) registration `map = R(yaw)·world + t` stored in `config/map_registration.json`
and **bound to the map content hash and the layout hash**. `calibrate_registration`
fits the real SLAM occupied cells to the compiled static collision geometry (robust
truncated-mean chamfer fit, grid search then coordinate descent) and accepts it only
when the median residual ≤ 0.10 m, ≥ 70 % of cells lie within 0.20 m, and the result
agrees with the controlled initial-pose prior from `maps/provenance.json` (≤ 0.5 m,
≤ 0.05 rad). A local check rejects a node when nearby residuals exceed 0.20 m
(non-rigid SLAM distortion). The record carries its own content id; a changed map,
layout or edited record is rejected. The Phase 4 `world_to_map_for_evaluation`
offset is used only as the prior, never as the transform. The registration is never
published as TF and never initializes AMCL.

## Map validation

`map_validation.py` loads the YAML/PGM with nav2_map_server trinary semantics and
also accepts the live `/map`; both produce the same content version. Every node
and connection is classified in the map frame:

* `validated` — every cell under the padded Nav2 footprint (0.54 m front, 0.52 m back,
  0.39 m half-width, from `nav2_params.yaml`), swept along the edge, plus in-place
  rotation discs at both ends, plus the 0.30 m clearance band, is **known free**;
* `blocked` — occupied cells under the footprint or inside the clearance band;
* `unmapped` — unknown cells, or the sweep leaves the map;
* `invalid` — the sweep intersects compiled static geometry, an endpoint is not
  validated, or the local registration residual is too high.

Unknown cells are never treated as free and no occupancy is synthesized from layout
geometry. Static validation does not prove an edge is clear *now*; dynamic obstacles
are handled by the existing Nav2 costmaps.

## Nav2 integration

**Route Server evaluation.** `ros-jazzy-nav2-route` 1.3.13 is installed (a released
Jazzy package, not a backport) with `ComputeRoute`/`ComputeAndTrackRoute`, GeoJSON
graphs, edge scorers and BT nodes. It was not adopted because (1) its planner is
Dijkstra-only, while this phase requires A* and Dijkstra with expansion metrics;
(2) executing its dense route path through `FollowPath` would replace Phase 4's
validated NavFn replanning around observed obstacles (RPP would stop at any
centreline obstruction); (3) running it beside this planner would create two
routing authorities. It remains a candidate for Phase 6 if the graph is exported to
its GeoJSON format.

**Execution.** `NavigateToPose` per waypoint stops at each objective, and
`FollowWaypoints` needs an additional server. The existing `bt_navigator` now also
loads the standard `NavigateThroughPosesNavigator`, with
`config/navigate_through_poses.xml` adapted from the installed Jazzy tree exactly as
Phase 4 adapted `navigate_to_pose.xml` (three retries, 1 Hz replanning, no `BackUp`).
No server, controller, costmap, footprint or guard parameter changed; no new
lifecycle node was added. `NavigateToPose` is unchanged.

**Objectives.** Only route nodes where the route turns (> 0.35 rad) or changes
corridor/kind become intermediate objectives, plus the dock with its yaw. Nav2
removes passed objectives within 0.7 m, so the robot does not stop at them.
Because NavFn may still take a free-space shortcut between objectives, the
executor checks every `/plan` against the route lanes (each edge's width, extended
over junction squares) and treats a departure as a deviation. The tolerance is
1.0 m. Any other aisle's centreline lies at least 2.7 m beyond a lane edge, so every
aisle bypass is still caught, while Nav2's local detours around people in open floor
are tolerated. A 0.35 m tolerance was measured to turn such a detour into
stop-and-wait inside a busy junction (see the validation record).

## Route executor

`execute_route` is a bounded process like Phase 4's `navigate` client:

```text
IDLE → PLANNING → PLANNED → VALIDATING → EXECUTING → COMPLETED
                                   ↘ REPLANNING ↗        ↘ BLOCKED | FAILED | CANCELED
```

Preconditions (fail closed): artifact, registration and installed sources match;
live `/map` equals the validated map version; AMCL covariance within limits and
`map → base_link` available; the NavigateThroughPoses server is ready; the only
`/cmd_vel` publisher is `navigation_command_guard`. The robot attaches to a validated
node within 0.5 m, or is projected onto the lane of an available edge (respecting
direction). Failing both, it may use a straight connector to a validated node within
3 m. In every case the connector from the robot to the graph is checked on the live
`/map`: it must contain no occupied cells, and no unknown cells except those under the
robot's own footprint. The robot physically occupies that space; for example, the area
behind the spawn was never scanned.

Failure handling is bounded (`max_replans` = 3). A Nav2 abort, a plan deviation or an
operator closure triggers inspection of Nav2's live **global costmap** along the
remaining route: a connection is closed only if some corridor cross-section has no
cell with cost 0–98 (traversable for the robot centre). Without such evidence no
corridor is closed (a Nav2 failure or detour is not proof of a blocked corridor):
the action is cancelled, the robot waits `retry_wait_s` (8 s) stopped, and the same
graph route is retried within the same budget; it never follows the detour. A robot
inside a just-closed connection may only retreat to the node behind it. A repeated
route signature ends in `BLOCKED` (no oscillation). `COMPLETED` requires Nav2
`SUCCEEDED` and the final pose within 0.30 m / 0.20 rad of the dock. Cancellation
(SIGINT/SIGTERM, `--cancel-after`, timeout) cancels the action; the executor never
publishes velocity.

Exit codes: 0 completed, 1 failed, 2 invalid request, 3 blocked / no route, 4 graph,
map or registration incompatible, 5 localization/Nav2 unavailable, 6 canceled, 7 timeout.

## Commands

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
# Calibrate (after any map change) and regenerate the graph; --check detects staleness.
ros2 run amr_route_planner calibrate_registration --source-tree --output src/amr_route_planner/config/map_registration.json
ros2 run amr_route_planner build_graph --source-tree --report-dir /tmp/graph
ros2 run amr_route_planner build_graph --check
# Plan only (no ROS, no motion):
ros2 run amr_route_planner plan_route --from <station|node> --to <station|node> --algorithm both
ros2 run amr_route_planner plan_route --from pickup_station_01 --to dropoff_station_02 --topology-only
# Execute on a running navigation stack (same ROS_DOMAIN_ID / GZ_PARTITION):
ros2 run amr_route_planner execute_route --to <station|node> --timeout 600 --output /tmp/route.json
ros2 topic pub --once /route_planner/block_connection std_msgs/msg/String "{data: '<connection>'}"
# Owned end-to-end run (one simulation, Gateway, Nav2, optional browser):
python3 src/amr_route_planner/scripts/route_integration.py --initial 0 0 0 \
  --step '--to <node>' --browser --output /tmp/route-run
```

## Digital Twin

The existing Gateway gains two read-only endpoints with independent version 1.0
schemas (`shared/schemas/route-graph.v1.schema.json`, `route-status.v1.schema.json`):

* `GET /api/v1/routes/graph` — static world-frame graph, connection statuses,
  stations and the registration; fetched once per graph version, never broadcast.
* `GET /api/v1/routes/status` — the latest `/route_planner/status` message (latched
  `std_msgs/String` carrying the versioned JSON, size-bounded and validated by a
  strict Pydantic model) with availability and age.

WebSocket v1/v2 and their schemas are unchanged. The browser fetches the graph
once, polls the status at 1 Hz, and draws the route through
`mapToWorld(registration, …)`. It refuses to draw (and states why) when the
registration is unverified or the route's graph, map or registration version differs
from the displayed graph. Robot motion still comes only from the Gazebo pose pipeline.

## Phase 5.1 changes (appended)

The design above is unchanged. Phase 5.1 (`PHASE_5_1_REMEDIATION.md`) adds:

- **Candidate maps.** `Paths.candidate(dir)` evaluates a candidate map package
  (`warehouse.yaml`, `map_registration.json`, `warehouse_graph.json`) without touching
  the approved artifacts. `execute_route --candidate`, `route_integration.py
  --candidate` and `observe_obstacles.py --candidate` use it. `evaluate_map.py` reports
  known-free area, global and 4 m-tile residuals, loop return, and graph coverage
  (validated nodes, connections, stations, reachable station pairs, independent
  cycles).
- **Single registration source.** `coordinate_registration.active()` replaces every
  hard-coded `+11 m` offset in the evaluation tools.
- **Deadline.** `execute_route --timeout` counts simulation seconds; `--wall-guard`
  (3000 s) bounds a stalled simulator.
- **Surveys.** Plans live in `config/survey_coverage.json` and
  `config/survey_diagnostics.json`.
  - Run: `extend_map_survey.py --fresh --record --bag --plan …`.
  - Offline SLAM comparison on the recorded bag: `replay_survey.py`.

```bash
# Evaluate a candidate without touching the approved map
ros2 run amr_route_planner calibrate_registration --source-tree --map C/warehouse.yaml --output C/map_registration.json
ros2 run amr_route_planner build_graph --source-tree --map C/warehouse.yaml --registration C/map_registration.json --output C/warehouse_graph.json --report-dir C/graph
python3 src/amr_route_planner/scripts/evaluate_map.py --source-tree --map C/warehouse.yaml --registration C/map_registration.json --graph C/warehouse_graph.json --recording SURVEY/drift-recording.json --output C/evaluation.json
python3 src/amr_route_planner/scripts/route_integration.py --candidate C --scenario normal --initial 0 0 0 --step "--to dropoff_station_02" --output OUT
```
