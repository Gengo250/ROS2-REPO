# Forklift–AMR traffic: EMP-1 / amr_01 conflict, stop-and-go, unexpected routes

Report: forklift `EMP-1` stopped (`veh`) at about (63.4, 23.0) next to `amr_01`; the robot,
reassigned from `pick_south_e1` to `dock_05`, showed *Replanning* and *Interlock: Missing or stale
command*; the forklift appeared to follow the robot and the robot moved intermittently along an
unusual route near racks. This document records the reproduction, the confirmed root causes, the
changes (engine forklift traffic, Gateway coordinator, command guard; no change to Nav2 parameters,
the map, the robot or forklift dimensions, collision checking or the YOLO pipeline) and the
validation. Evidence: `evidence-forklift/` (git-ignored, local).

## 1. Reproduction (unmodified code, `2957dda`)

Full stack (`lite_navigation.launch.py`, 3 AMRs, RTF 1.0), missions through the Gateway API, as the
browser sends them. Recorder: engine state at 5 Hz (poses, speeds, forklift state and reason,
mission state, Nav2 plan, interlock, events) plus ROS topics (`nav_cmd_vel`, `cmd_vel`,
`navigation/diagnostics`, `plan`, `amcl_pose`).

| Run | Setup | Outcome |
|---|---|---|
| `before_parked` (the reported case) | `amr_01` parked at `charger_01` when EMP-1 arrives; then `pick_south_e1`, cancel after 6 s, `dock_05` | EMP-1 stopped mid-turn at (63.5, 22.57), lane 128.6, `veh`, body across the charger aisle mouth. The robot squeezed out between forklift and rack: EMP-1 restarted 6 times (stop-and-go, 24 state changes), footprint gap down to **0.18 m**. `dock_05` **failed** after 10 recoveries (see §2.5). |
| `before1`, `before_L8` | `amr_01` leaves `charger_01` as EMP-1 approaches (lead 18 m / 8 m) | Succeeded; EMP-1 stopped once mid-turn (gap 0.81 m) |
| `setup1` | `amr_01` drives east along the north cross aisle (EMP-1's lane) | EMP-1 followed at 0.56–0.99 m (p10–p50); at the aisle mouth 3 restarts within 7 s alternating `yield`/`veh`/none, gap 0.32 m |

The reported screenshot state (EMP-1 *Stopped · veh* at (63.4, 23.0), robot at `charger_01`) was
reached by itself in run `before_L12` and is shown in `evidence-forklift/ui_before_parked.png`.

## 2. Root causes (confirmed)

### 2.1 The U-turn envelope of the north loop reaches the parking pose of `charger_01`

EMP-1's lane (`north`) turns at x = 63.5 between y = 21 and 24, i.e. through the mouth of
`charger_aisle` (capacity 1, x 61.87–65.1 between the rack end and the charger units, 3.23 m).
Measured by sweeping the forklift footprint (4.55 × 1.98 m) along the lane: minimum clearance to a
robot parked **exactly** at `charger_01` is **0.21 m**, and **0.02–0.41 m** within Nav2's goal
tolerance (±0.2 m; Nav2 parked it at y = 26.86–26.88 in every run). The forklift's vehicle margin is
0.3 m, so it stops — correctly — but only once its look-ahead (≥ 3.4 m) reaches the conflict, which is
inside the turn: lane 128.6–128.8, (63.5, 22.6–22.8), heading north. Its 1.98 m body then closes the
3.23 m mouth, the only way out of the dead-end charger aisle. The forklift waits for the robot to
move away; the robot can only move away through the forklift: a deadlock. The same happens to a
robot leaving `charger_01` late (harness `charger01_then_leave`, before: the robot never got out).

### 2.2 Stop-and-go: no hysteresis, a jumping stopping point, emergency braking on every reduction

* The forklift restarted as soon as *one* look-ahead sample (0.35 m) was free, and stopped again
  at the next one: every small robot motion (Nav2 rotate-to-heading, spin recovery, creeping) made
  it advance (harness *hesitating robot*: 12 restarts in 60 s).
* The stopping distance was the distance of the first touching sample, so it jumped by 0.35 m
  whenever the body ahead crossed a sample: the target speed jerked.
* Any deceleration was applied at the emergency rate (2.3 m/s²) instead of the deceleration
  actually needed: overbraking, re-acceleration, oscillation (harness *following*: 408
  emergency-rate braking steps, 6 stop/restarts in 60 s).
* The reason label switched between `veh`, `follow` and `yield` with the robot's speed around
  0.05 m/s, and between `veh` and `curve` while creeping (25 changes while stuck at `charger_01`).

### 2.3 Right of way over AMRs existed by accident and expired after 12 s

`r.yield_to is q.agent` is `None is None` for an AMR, so forklifts yielded to robots' predicted paths
by accident; after 12 s of waiting the forklift's priority boost (meant for forklift–forklift
deadlocks) disabled that and it pushed towards the robot.

### 2.4 A robot turning in place was only its rectangle

`predicted_samples()` keeps a stationary robot as one sample (its footprint) — stationary robots were
always obstacles (no change needed) — but a robot rotating in place sweeps a 0.61 m disc that the
forklift did not consider (harness: gap 0.195 m next to a turning robot).

### 2.5 Interlock texts: legitimate standstills reported as communication failures; a localization stall

* *Missing or stale command* appears whenever Nav2 is not commanding: no goal, the 5 s `Wait`
  recovery of the behaviour tree, costmap clearing, a goal just ended (0.4 s of *Invalid simulation
  timestamp: command* first). During driving, commands were fresh in every run: `nav_cmd_vel`
  p50 66.6 ms (15 Hz), p95 ≤ 123 ms, no gap > 1 s except the reported recovery waits. Not a timestamp
  or CPU problem; a wording problem.
* Secondary finding in `before_parked`: after the squeeze past the forklift the robot entered the
  rack aisle x = 57 and stopped with *Localization unavailable or uncertain*. AMCL was **correct**
  (6 mm, 0.005 rad from ground truth) but its along-aisle variance was 1.061 m² (> 1.0, the guard's
  limit); stopped, AMCL does not update (`update_min_d` 0.10 m), so the variance never decreases and
  Nav2 exhausted its recoveries. Seen in 1 of 5 baseline runs (the one with the close forklift
  maneuvering); after the fix the maximum along-aisle variance on the same route was 0.46 m².
  The guard's fail-safe was not changed (see Limitations).

### 2.6 The "unusual route" is Nav2's shortest valid path

`NavigateToPose` (NavFn A*) routes `charger_01 → dock_05` down the rack aisle x = 57 (2.26 m, the
robot with padding is 0.78 m wide) and then between the expedition equipment: shortest path,
collision-free in the costmaps, outside the keep-out mask, compatible with the robot (no reversing
needed). Replanning runs at 1 Hz by design (behaviour tree `RateController`). Route topology
(which rack aisle, or a length jump > 10 m) changed only once per run, and only in the two runs where
the moving forklift passed the north mouth of aisle 57 while the robot approached it: 57 → 51 → 57
for 2–3 s (`before1`, `after_L18`): ordinary obstacle avoidance through the global costmap's obstacle
layers, no oscillation. Plans and the executed trajectory agree (`*_traj.png`). What looked erratic in
the report was the squeeze between the stopped forklift and the rack end (2.1) and the stop-and-go
next to it (2.2). Graph routing (`build_campus_graph.py`, which penalises capacity-1
aisles) is not used by the missions (`strategy: direct`); changing that is a routing-architecture
decision, left out (Limitations).

## 3. Changes

### Engine — `amr_lite_sim/agents.py`, `amr_lite_sim/world.py`

Thresholds come from the vehicle data and the simulation, not from enlarged safety distances:

| Constant | Value | Origin |
|---|---|---|
| `SAMPLE_STEP` | 0.35 m | the look-ahead footprint spacing |
| `VEHICLE_MARGIN` | 0.3 m | the existing vehicle hazard margin (unchanged) |
| `resume_room` | v²/(2·b) + 0.35 = 1.59 m (EMP-1) | braking distance from cruise (1.45 m/s, 0.85 m/s²) + one sample |
| `RESUME_DWELL` | 1.0 s | one Nav2 replanning period (`expected_planner_frequency` 1 Hz) |
| `ZONE_STEP` | 0.1 m | sweep resolution of a turn (clearance changes by decimetres within 0.35 m) |
| `FORKLIFT_WAIT_REPORT` | 30 s | persistent-wait report |

1. **Vehicle hold with hysteresis.** Entering: stopped (< 0.06 m/s) for a vehicle that is not driving
   away ahead (`veh`, `yield`). Leaving: the free lane beyond the vehicle-imposed stopping point is at
   least `resume_room`, continuously for `RESUME_DWELL`. One stable reason while held. A leader being
   followed (`follow`) never holds the forklift: it opens the way by itself.
2. **Continuous stopping point** for vehicle hazards: bisection between the last free and the first
   touching sample (< 3 mm), so the braking profile no longer jumps.
3. **Needed deceleration** behind vehicles: v²/(2·free lane), between service (0.85) and emergency
   (2.3 m/s²); people, objects and structure keep the previous emergency-rate behaviour.
4. **Explicit AMR right of way**: a forklift yields to an AMR whose predicted path it would cross,
   independent of its waiting time (the boost only orders forklifts). Label `follow` with hysteresis
   (enter at 0.05 m/s, leave at 0.01 m/s, same vehicle).
5. **Keep-clear lane intervals**: at start-up each forklift computes the lane intervals where its body
   overlaps a capacity-1 corridor of the scene (`navigation.corridors`): EMP-1 lane 125.8–133.0 (the
   charger aisle mouth); EMP-3 `front_passage_w` and the two wall aisles at its U-turns. Before entering
   one, the forklift sweeps the whole interval (0.1 m steps) against every vehicle's body and predicted
   samples (margin 0.3 m) and, if it is not clear, waits at the entry (`yield`). It never stops for a
   vehicle with its body across a narrow corridor's mouth. No geometry was changed.
6. **Turning robot = swept disc**: a robot rotating in place (|v| < 0.05, |ω| > 0.05) is its
   circumscribed disc (0.61 m) in the vehicle views; a stationary robot stays its footprint; no motion
   is invented for either.
7. **Persistent wait reported**: event `vehicle_wait` (forklift, blocking vehicle) after 30 s held, and
   `vehicle_resumed` when it goes on (frontend: Portuguese texts in the event log).

### Gateway coordinator — `world/traffic.py` (+ `api/app.py`, `ros/node.py`, `ros/world.py`)

Forklift-aware conflicts in the existing persistent-conflict step (no second traffic manager): a
driving robot stalled for `stall_seconds` (12 s sim) within `conflict_gap` (1 m) of a **stopped**
forklift yields: its goal is held, so it stands still — a standing robot is no crossing for the
forklift, which passes if its lane is clear — and is sent again when the forklift is `clear_gap`
(2 m) away. If the forklift cannot get away within `block_seconds` (30 s; its lane is closed by this
robot) or after `max_yields` (2) per mission, the robot is stopped and reported `blocked`
("Bloqueio persistente com o EMP-1"). A moving forklift is a passing one. Forklift boxes come from
the scene (`agents.forklifts`).

### Command guard — `amr_navigation/command_guard.py`

Absent or old command stream: Nav2 ends every command stream with an explicit zero twist (goal
reached, cancelled or failed; behaviour finished). Silence after a zero command with its publisher
present is reported as **`Standby: no active Nav2 command`**; silence after a moving command, or a
vanished publisher, keeps **`Missing or stale command`**. Both are not ready and the guard keeps
publishing zero velocity: the fail-safe is unchanged, no command is fabricated.

## 4. Validation

### 4.1 Engine harness (same scripted robot, same seed; original `2957dda` engine vs fixed)

| Scenario | min gap m | restarts near robot | emergency steps | state changes | forklift end (lane d) | robot end |
|---|---|---|---|---|---|---|
| Stationary robot in the lane | 0.88 → **1.00** | 0 → **0** | 31 → **5** | 4 → **4** | 89.1 → **89.0** | (30.0, 21.0) → **(30.0, 21.0)** |
| Hesitating robot in the lane (Nav2-like turns/creeps) | 0.81 → **1.04** | 12 → **0** | 57 → **23** | 46 → **6** | 89.3 → **88.9** | (30.4, 21.3) → **(30.4, 21.3)** |
| Following a robot that slows 0.6→0.15 m/s | 0.99 → **1.01** | 6 → **0** | 408 → **0** | 15 → **3** | 99.2 → **99.0** | (40.3, 21.0) → **(40.3, 21.0)** |
| Robot crossing both lanes (x = 27) | 0.41 → **0.47** | 1 → **1** | 32 → **30** | 7 → **10** | 99.0 → **97.5** | (27.0, 13.4) → **(27.0, 13.4)** |
| Crossing, forklift 9 m away | 0.94 → **0.94** | 1 → **1** | 31 → **31** | 6 → **6** | 112.2 → **110.8** | (27.0, 15.6) → **(27.0, 16.2)** |
| Robot waiting at the charger aisle mouth | 0.20 → **3.26** | 0 → **0** | 25 → **0** | 4 → **4** | 127.7 → **125.8** | (63.4, 25.8) → **(63.4, 25.8)** |
| Robot leaves the aisle mouth after 12 s | 0.52 → **1.20** | 0 → **1** | 25 → **0** | 4 → **10** | 127.7 → **149.7** | (63.4, 26.1) → **(39.1, 24.2)** |
| Robot parked at charger_01, leaves after 30 s (reported case) | 0.57 → **1.27** | 0 → **1** | 32 → **0** | 4 → **10** | 128.6 → **141.8** | (63.4, 26.9) → **(46.9, 24.2)** |
| Robot parked at charger_01 (40 s) | 0.78 → **4.66** | 0 → **0** | 34 → **0** | 25 → **4** | 128.8 → **125.8** | (63.4, 27.0) → **(63.4, 27.0)** |

Lane 125.8 is the keep-clear entry before the U-turn; 126–133 is the turn across the charger aisle
mouth. In the two `charger_01` cases the original forklift ended inside the turn (128.6–128.8) and in
`charger01_then_leave` the robot never got out (it stayed at (63.4, 26.9)); now the forklift waits at
125.8, the robot leaves westwards and the forklift follows it (141.8). No contact in any run. The
remaining emergency-rate steps happen when the space a robot occupies changes at once (it starts
turning in place: its body becomes the 0.61 m disc; a robot crosses in front).

`emergency` = agent steps decelerating at ≥ 95 % of the emergency rate (2.3 m/s²). The scripted robot
stands in for Nav2 (it stops when a forklift body is within 1.2 m ahead). Regression tests:
`amr_lite_sim/test/test_forklift_traffic.py` (8 tests).

### 4.2 Runtime (full stack, Nav2, RTF 1.0 unless noted)

| Run | Code | Scenario | `dock_05` | EMP-1 restarts near robot | EMP-1 state changes | min gap | `nav_cmd_vel` p95 / max gap |
|---|---|---|---|---|---|---|---|
| `before_parked` | original | reported case (5) | **failed**, 10 recoveries | 6 | 24 | 0.18 m | 123 ms / 5.2 s |
| `after_parked` | fixed | reported case (5) | **succeeded**, 0 recoveries | 1 | 8 | 0.62 m | 72 ms / 0.42 s |
| `rtf05_parked` | fixed, RTF 0.5 | reported case (6) | **succeeded**, 0 recoveries | 1 | 9 | 0.61 m | 71 ms / 0.58 s |
| `before1` | original | robot leaves as EMP-1 arrives (3) | succeeded | 1 | 11 | 0.81 m | 72 ms / 0.41 s |
| `after_L18` | fixed | robot leaves as EMP-1 arrives (3) | succeeded | 1 | 10 | 0.62 m | 72 ms / 0.41 s |

* Fixed runs: EMP-1 waited at the keep-clear entry (62.3, 21.0), lane 125.8, `yield`, outside the
  aisle mouth; `vehicle_wait` reported after 30 s; the robot left through the free mouth; one resume
  (`vehicle_resumed`), then EMP-1 drove its turn. At RTF 0.5 the forklift resumed after 17.4 s of
  simulation (18 s at RTF 1.0): decisions run on simulation time.
* No contact in any run. Interlock while idle: `Standby: no active Nav2 command`; while driving,
  `ready` throughout (no stale command).
* Maximum AMCL along-aisle variance on the same route after the fix: 0.20–0.46 m² (limit 1.0).
* Screenshots: `ui_before_parked.png` (EMP-1 stopped in the turn next to the parked robot, as in the
  report) and `ui_after_parked.png` (EMP-1 waiting before the turn, robot leaving for `dock_05`,
  event log "EMP-1 aguardando AMR 01: sem passagem segura").
* Plots per run: `*_traj.png` (trajectories, forklift stops, Nav2 plans), `*_curves.png` (speeds,
  separation, forklift state/reason, mission state), `*_cmd.png` (command inter-arrival).

### 4.3 Multi-AMR regression with forklifts (existing runner)

`run_fleet_missions.py` at RTF 1.0 with forklifts and workers active (fixed code): `three_missions`,
`shared_destination` and `reassign` **passed** (0 contacts, min robot footprint gap 13.0 / 5.1 /
3.1 m, RTF mean 1.0 / 1.0 / 0.99). `reassign` had 6–7 recoveries per robot, all at person–AMR
standoffs (COL-04, COL-07 at `pick_north_e1`, COL-09: the pre-existing limitation in
FLEET_COORDINATION.md §6); none involved a forklift. A same-seed engine comparison (workers, 3
forklifts, 3 parked robots, 300 s) gives the same 22 worker `turned_back` events with the original and
the fixed engine: the change does not alter worker behaviour.

New runner scenario `forklift_parked_reassign` (the reported sequence with explicit checks: EMP-1 held
by the parked robot but never in the aisle mouth, `dock_05` reached, no forklift contact, robot–forklift
gap ≥ 0.3 m, at most one EMP-1 restart per encounter); the runner now also records forklift
transitions, restarts near robots and the minimum robot–forklift gap in every scenario.

### 4.4 Unit tests

* `amr_lite_sim`: 40 passed (32 existing + 8 in `test_forklift_traffic.py`).
* `digital_twin_gateway`: 73 passed (3 new forklift coordinator tests in `test_traffic.py`).
* `amr_navigation`: 35 passed (2 new command guard tests in `test_guard.py`).
* Frontend: `tsc --noEmit` clean.

## 5. Performance

* Engine `World.step` (agents and traffic, no lidar), full operations profile: 2.03 → 2.00 ms per
  step (120 s, robots in the lanes) and 2.11 → 1.99 ms (300 s, EMP-1 and EMP-3 held by parked robots).
  Keep-clear sweeps are computed once at start-up; a held, stopped forklift skips the bisection.
* Runtime RTF 1.0 runs: mean 0.995–1.0 before and after; the dips to 0.55–0.65 in two fixed runs
  coincide with the headless-browser screenshots on the same 4 cores.
* Gateway: forklift footprint gaps are computed only for robots already stalled 12 s. Command guard:
  one publisher query only while the command stream is absent or old.

## 6. Limitations

* **Geometry conflicts remain, now handled explicitly.** A robot parked at `charger_01` (0.02–0.41 m
  from EMP-1's turn) holds EMP-1 before its turn as long as it stays; a robot parked at `receiving_01`
  (≈0.08 m from EMP-3's `dock` lane body) holds EMP-3. Both forklifts stop at a safe distance and report
  `vehicle_wait`; the robots are never trapped. Moving the station poses or the lane turns would
  remove the waits; that is a scene decision, not taken here.
* **AMCL variance in long rack aisles.** The guard's localization limit (variance 1.0) can stop a robot
  whose pose is correct but whose along-aisle variance grew; stopped, AMCL does not update, and the
  mission fails after Nav2's recoveries. Seen once (before the fix, after close maneuvering next to the
  forklift). The fail-safe was not relaxed; a no-motion AMCL update or route preferences would be the
  options.
* **Routing.** Missions use direct `NavigateToPose` (shortest path, through capacity-1 rack aisles);
  the campus route graph is not used by missions.
* **Person–AMR standoffs** (pre-existing, FLEET_COORDINATION.md §6) still cause recoveries and can keep
  a robot at `pick_north_e1`.
* **Following at runtime.** Following an AMR was validated in the engine harness and observed before
  the fix at runtime (`setup1`); a dedicated runtime following run after the fix was interrupted
  (the robot was held at `pick_north_e1` by the person standoff above, then testing was stopped).
* The runner scenario `forklift_parked_reassign` was added but not yet executed end to end.

## 7. Commands

```bash
ros2 launch amr_lite_sim lite_navigation.launch.py [real_time_factor:=0.5]
python3 -m pytest src/amr_lite_sim/test/test_forklift_traffic.py
python3 src/amr_lite_sim/scripts/run_fleet_missions.py --scenario forklift_parked_reassign --output forklift.json
python3 src/amr_lite_sim/scripts/run_fleet_missions.py --scenario three_missions --scenario shared_destination --scenario reassign --output fleet.json
```
