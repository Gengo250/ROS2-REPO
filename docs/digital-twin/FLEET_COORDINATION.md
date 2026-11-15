# Fleet coordination: shared destinations, narrow corridors and robot conflicts

Three AMRs (`amr_01..03`) with independent Nav2 stacks share the campus. This document records the
reproduced failure, its root cause, the coordination added on top of the existing navigation (no
change to the Nav2 parameter file, the map, the robot model, the engine physics or the command
guard) and the validation.

## 1. Reproduction and root cause

Reproduced on the unmodified branch (`90bd7e5` + the camera fix `8817911`), three AMRs, RTF 1.0:
all three sent to `charger_01`, then reassigned (`amr_01 → pick_north_e1`, `amr_02 → dock_03`,
`amr_03 → receiving_01`).

* Contacts `amr_01 × amr_03` already on arrival (t ≈ 211 s); the three robots ended 0.04–0.10 m
  apart around the charger; `amr_03` succeeded, `amr_01` and `amr_02` failed.
* After the reassignment the robots cycled in contact (`amr_01 × amr_03` repeatedly, `amr_02`
  also against forklift EMP-1); RTF fell to 0.52–0.66; all three missions failed.

Root cause (combination, each confirmed from runtime data or the code):

1. **No coordination of shared destinations.** The Gateway sent three Nav2 goals with the same
   pose; three independent stacks converged on the same coordinates.
2. **Other AMRs are not reliably visible to navigation.** The front scanner covers ±90°; the
   360° scanner (0.48 m) passes over the other robots' bodies (sensed up to 0.455 m). At the
   reproduced contact pose `amr_03` had no return at all from `amr_01` (165° off its front, under
   its top scanner). Both robots were rotating in place to the goal heading (RPP rotate-to-heading;
   spin recovery) and their corners met.
3. **A contact is permanent.** The only collision-free motion out of a contact is usually reverse,
   which the stack forbids on purpose (RPP `allow_reversing: false`, no BackUp in the BT, the
   command guard rejects `v < 0`); rotation is blocked by the contact itself. Nav2 then loops
   through recoveries until the goal fails.

The engine's contact handling (stop at the last collision-free fraction of the step; only motion
away releases) is correct and was not changed.

## 2. Fleet tracking costmap layer

`amr_lite_sim/fleet_scan.py` (`fleet_tracker`, started by `lite_navigation.launch.py` when more
than one AMR is simulated) publishes, per navigated robot, the collision boxes of all *other* AMRs
(+0.10 m margin) from the engine's authoritative state, as seen from that robot's `base_link`:
`<ns>/fleet/robots`, 720 beams over 360°, 10 Hz of simulation time, stamped with the engine state's
simulation time. It is explicit shared tracking data on its own topic, consumed by its own costmap
layer; no scanner topic or scanner geometry is touched. LaserScan is used only because Nav2's
obstacle layer marks *and clears* a range image: beams without a robot clear the layer up to the
range limit, so a robot that drives away never stays behind as a ghost and the map is never
modified.

`amr_navigation/launch_support.py` (`with_fleet_layer`, `navigation.launch.py fleet_obstacles:=true`)
inserts `fleet_layer` (an `ObstacleLayer`, marking + clearing, `observation_persistence: 0`,
`expected_update_rate: 0.5`) before the inflation layer of the local (8 m) and global (25 m)
costmaps, in the per-robot temporary parameter copy. `nav2_params.yaml` itself is unchanged
(its hash feeds the Phase 5 route graph); a single-robot launch uses it exactly as before.

Properties: unique robot ids (engine entity ids), relative poses computed in the world frame from
ground truth (no world/map transform involved), never the observer itself, one publisher per robot
topic, fail-closed freshness (no engine state → no fleet data → the costmap reports a stale source).

## 3. Gateway fleet coordinator

`digital_twin_gateway/world/traffic.py` is pure decision logic; `ros/world.py` (`Commands`)
executes its actions with Nav2 actions only (send goal, cancel goal, send again). It never
publishes a velocity, so each robot's command guard remains the only writer of its `cmd_vel`.
A 2 Hz step runs on the API loop (`coordination_period`), on the latest engine state.

### Stations (single-capacity resources)

* A mission whose station is free (no owner, no queue, no other robot inside the occupancy disc)
  reserves it and its goal is sent. Otherwise the mission is accepted as `waiting`
  (`waiting_for: station`, `queue_position`, `blocking_robot`) and the robot stays where it is.
* The decision is taken before any `await`: simultaneous requests are serialized on the event loop.
* FIFO queue per station, bounded (4); no barging (a free station with a queue goes to the head).
* A station stays occupied while any robot's centre is within 1.5 m of its pose (0.6 m
  circumscribed radius + 0.9 m clearance), even after its mission succeeded; the queue head is
  dispatched only once that robot has physically left.
* Cancel or replace (a new mission for a waiting robot replaces the queued one) releases the
  reservation; mission end (succeeded, failed, canceled, blocked) releases ownership, physical
  presence still counts.
* Circular waits (A waits for B's station while B waits for A's) are broken by dispatching the
  highest-priority robot of the cycle; when it leaves, the next one's station is free.

### Narrow corridors

The scene's navigation corridors with capacity 1 (rack aisles, charger aisle, wall aisles,
production access) are gated: a robot whose Nav2 plan enters one within 6 m of path claims it if
no robot with an open mission is inside and nobody else claimed it; otherwise its goal is held
(cancelled) before the entrance (`waiting_for: corridor`) and sent again once the corridor is clear.
Parked robots hold nobody (the planner routes around them through the fleet layer).

### Persistent conflicts

A driving robot with no progress (0.3 m of motion or of remaining distance) for 12 s of
**simulation** time, with another AMR within 1.0 m of its footprint or in contact, is in a robot
conflict; brief stops never qualify, and the RTF does not change the decision.

* Both robots driving and stalled: the lower-priority one yields (held, `waiting_for: robot`) until
  the other is 2 m away or stops driving; at most 2 yields per mission.
* The winner still cannot pass the yielded robot: roles swapped once.
* Still stuck, or stuck 30 s behind a parked robot: both are stopped (goals cancelled) and reported
  `blocked` ("Bloqueio persistente. Intervenção necessária."). No reverse, no forced motion,
  no teleport, no change of collision checking.

Priority: older mission first, then robot id.

### Contract (world-state v1, additive)

* `Mission.status`: adds `waiting`, `recovering` (Nav2 recovery counter increased; back to
  `active` after 0.5 m of progress) and `blocked`; new optional fields `waiting_for`,
  `waiting_on`, `blocking_robot`, `queue_position`, `yields`.
* `WorldState.stations` (owner, occupants, queue) and `WorldState.corridors` (claimed_by,
  occupants, waiting), non-empty entries only.
* Same mission API; a queued mission is `accepted: true` with `status: waiting`.

### Frontend

The mission panel, the robot panel, the onboard HUD and the event log show Portuguese texts
(e.g. "Aguardando liberação da estação charger_01 (ocupada pelo AMR 01) · posição 1 na fila",
"Aguardando passagem do AMR 01 no corredor charger_aisle", "Recalculando trajetória",
"Bloqueio persistente com o AMR 03. Intervenção necessária.") and the station's owner and queue;
Nav2 error codes are not the primary text.

## 4. Validation

Runner: `src/amr_lite_sim/scripts/run_fleet_missions.py --scenario …` (extends the existing fleet
script; same API path as the browser). Every scenario records trajectories, sampled Nav2 plans,
mission status transitions, station/corridor reservation changes, the minimum distance between
robot **footprints** (collision boxes from engine poses), contact events, time in the engine's
blocked state, recoveries, failures and the RTF, and ends with explicit checks. Evidence:
`evidence-fleet/` (`summary.json`, one JSON report per run, screenshots).

| Scenario | Target RTF | Result | Min footprint gap | Robot–robot contacts |
|---|---|---|---|---|
| 1 Shared destination (3 → charger_01) | 1.0 / 0.5 | passed / passed | 3.41 / 3.41 m | 0 / 0 |
| 2 Reassignment (→ pick_north_e1, dock_03, receiving_01) | 1.0 / 0.5 | passed / passed | 5.77 / 5.41 m | 0 / 0 |
| 3 Head-on through the charger aisle | 1.0 / 0.5 | passed / passed | 2.67 / 3.55 m | 0 / 0 |
| 4 Three simultaneous missions | 1.0 | passed | 0.44 m | 0 |
| 5 Occupied station | 1.0 | passed | 31.1 m | 0 |
| 6 Cancellation (queued, active owner) | 1.0 | passed | 31.1 m | 0 |
| 7 Conflicting missions at low RTF | 0.5 | scenarios 1–3 above | | |
| 8 Single robot (`robots:=amr_01`) | 1.0 | passed (dock_05 in 114 s) | n/a | 0 |

No contact of any kind (robot, structure, person, forklift) in any of these runs; no blocked time.

* **1–2 (the reported sequence).** Only `amr_01` drove to `charger_01`; `amr_02`/`amr_03` were
  queued at request time (positions 1 and 2) and did not move (< 0.3 m); exactly one robot on the
  station. Reassignment: all three accepted (the queued missions replaced), all three reached
  their new stations (< 0.6 m), the charger_01 queue cleared.
* **3.** `amr_02` was held before the charger aisle (`waiting / corridor charger_aisle /
  blocking amr_01`, 24.2 s of simulation at RTF 1.0 and at 0.5) while `amr_01` drove out of it
  southwards, then claimed it, entered and arrived.
* **4.** `amr_03` waited 7.9 s for `charger_02` until `amr_02` had physically left it;
  `amr_02` and `amr_03` then met head-on in the 7 m north cross aisle and passed each other
  0.44 m apart (fleet layer).
* **5.** `amr_02` waited for `dock_05` (blocking `amr_01`) without approaching; it was dispatched
  only after `amr_01` left the occupancy disc; both arrived.
* **6.** A cancelled queued mission left the queue; the owner was cancelled while driving at
  0.6 m/s, stopped and stayed put (0.0 m in the following 10 s); the waiter behind it was
  dispatched in the same step and arrived.
* **8.** One robot: no `fleet_tracker`, costmap plugins exactly the validated ones
  (`static, obstacle, top_obstacle, inflation`).

Unit tests: `digital_twin_gateway/test/test_traffic.py` (14: FIFO and ownership, physical
occupancy, cancel/replace, queue bound, station-swap cycle, corridor hold/claim/priority, parked
robots, stall only after the simulation-time threshold and independent of the sampling pace,
yield → swap → block, contact as conflict), `amr_lite_sim/test/test_fleet_scan.py` (4),
`amr_navigation/test/test_navigation.py` (fleet layer injection). Full suites: gateway 70,
amr_lite_sim + amr_navigation 65, frontend 30; schema and type drift checks clean.

## 5. Performance

* Before (reproduction, unmodified code): RTF 0.99–1.0 while three robots drove, 0.52–0.66 while
  they were cycling in contact after the reassignment.
* After: RTF mean 0.995–1.00 in every RTF-1.0 scenario (three robots navigating, no browser), 0.50 at
  target 0.5.
* `fleet_tracker`: 4 % of one core (raw subscription, only every third 30 Hz engine state
  deserialized; no `/clock` subscription); `fleet_ranges` is 0.5 ms per robot.
* Coordinator: 1.25 ms per step (3 robots, the scene's 47 corridors, 256-point plans) at 2 Hz.
* No new ROS subscription in the Gateway (it reuses the engine state and plans it already has).

## 6. Limitations

* **Person–AMR standoff in picking aisles (pre-existing, not robot–robot).** At `pick_north_e1`
  a worker (COL-07) waiting for the robot to clear the 2.26 m aisle and the robot's safety
  rotation stop (person inside the swept disc) can wait for each other; Nav2 then fails the
  mission after its recoveries (observed twice at RTF 0.5, `rtf05_head_on_aborted_person_amr_standoff.json`).
  The coordinator only handles conflicts between AMRs; this needs a rule in the worker simulation.
* Waiting robots wait where they are when the mission is requested; there are no staging poses.
  A robot asked to wait in the middle of a narrow aisle stays there (other robots route around it
  or are held by the corridor rule).
* Two robots that both *start* inside the same capacity-1 corridor and drive towards each other
  (e.g. swapping two chargers) are not separated by the corridor rule; the conflict monitor yields,
  swaps and, if the aisle cannot fit them, stops both and reports `blocked`.
* Robots already in contact (old state) are only released by Nav2 motions the engine accepts
  (no reverse); the coordinator yields/swaps and otherwise reports `blocked`.
* Nav2 stack bring-up with three robots occasionally fails under CPU load (an `initial_pose` or
  AMCL activation timeout); unrelated to this change (seen in 2 of 8 launches here).

## 7. Commands

```bash
ros2 launch amr_lite_sim lite_navigation.launch.py                 # 3 AMRs: fleet layer + coordinator
ros2 launch amr_lite_sim lite_navigation.launch.py robots:=amr_01  # single robot, original parameters
python3 src/amr_lite_sim/scripts/run_fleet_missions.py --scenario shared_destination --scenario reassign \
    --scenario head_on --scenario three_missions --scenario occupied_station --scenario cancellation --output fleet.json
```
