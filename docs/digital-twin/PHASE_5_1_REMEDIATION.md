# Phase 5.1 — SLAM mapping and dynamic-safety remediation

Scope: fix the Phase 5 blockers without redesigning the Digital Twin:

- incomplete or drifting SLAM coverage;
- AMR–worker contacts from outside the sensor field of view.

Nav2, SLAM Toolbox, AMCL, the differential-drive controller, the
`navigation_command_guard`, the graph planner, the Gateway, Three.js and Mosaico are
all preserved.

## Baseline

- Branch `feature/phase5-1-remediation`, created locally from
  `feature/warehouse-graph-routing` at `3bdc33a` after `git fetch`. The tree was clean.
  Phase 4 is still not in `main`.
- No commits or pushes are made.
- The baseline experiments run against an untouched build of `3bdc33a`: a separate
  git worktree and install space in the session scratchpad. That way the "before"
  measurements cannot be contaminated by remediation changes. One attempt
  (`diag-baseline-config-changed-refused`) was correctly refused by the warehouse
  launch's own guard: "AMR dimensions changed; run build_warehouse.sh". That happened
  because the live configuration had been edited while the run was in progress.

## Measurement method

`amr_navigation/scripts/slam_diagnostics.py` (new, read-only) records for every
`LaserScan` stamp:

- the SLAM estimate, `map → base_link` at that stamp;
- wheel odometry, `odom → base_link` at that stamp;
- Gazebo ground truth, interpolated to the same **simulation** stamp.

Ground truth is evaluation data only.

`analyze_slam_diagnostics.py` aligns each labelled segment at its own start
(Δ = P(t₀)⁻¹·P(t)). Drift is therefore measured without any world/map registration.
It reports final, maximum and p90 position error, yaw error and the largest
single-step jump.

`extend_map_survey.py` now runs labelled step plans (moves and pure rotations) with
fresh or resumed SLAM and the recorder. Experiments A–F are in
`evidence-phase5-1/plans/diagnostic-experiments.json`.

`amr_navigation/scripts/validate_lidar_rendering.py` (new) checks the simulated
sensors themselves. It places the AMR at fixed poses in the mapping world (no
workers) and compares every `/scan` and `/top_scan` beam with a ray cast against the
warehouse model's visual geometry, which is what `gpu_lidar` renders. Gazebo poses and
model geometry are evaluation data only.

## Root cause 0 — simulated LiDAR rendering defect (CONFIRMED, decisive)

**Symptom.** Every candidate map contained straight occupied lines through open aisle
space: y ≈ ±6.5 across the warehouse, and x ≈ −2.45 across the main aisle. The
`streak-probe` bag showed real `/scan` beams that passed through rack kick-rails and
returned from empty floor about 4 m away. Floor-marking/pitch, real structure and
scan latency had already been ruled out (below).

**Measurement.** Over all 1,388 scans of `streak-probe`
(`streak-probe/per-beam-no-return.json`), the front scanner returned **nothing** for
the beams at 45–90° left in 100 % of scans, and for 72 % of the beams at −90…−45°.
The central ±45° missed 5 %, which is the open main aisle. Geometry 4–10 m away
(rack kick-rails, the north wall) was simply not seen
(`streak-probe/first-scan-vs-geometry.png`). A static capture at the spawn pose gave
the same pattern for the **untouched Phase 5 robot** (`3bdc33a`) and for the Phase 5.1
robot (`static-scan-capture/summary.json`). The defect therefore predates the new
scanner. The new 360° scanner was blind in 52–68 % of its beams.

**Cause.** The headless EGL sensor renderer ran on Mesa
`zink Vulkan 1.4 (NVIDIA GeForce GTX 1650 (NVK TU117) (MESA_NVK))`, read from
`~/.gz/rendering/ogre2.log`. A `gpu_lidar` wider than 90° is rendered through several
camera faces. Under zink-on-NVK only the forward face is correct: the others return
no hit, or occasionally a wrong depth. Those wrong-depth returns are the phantom
points. They sit at a roughly constant offset to the side of the robot, so they trace
a straight line parallel to its path (y ≈ ±6.5 while driving the cross aisles at
y ≈ ±8.5; x ≈ −2.45 while driving aisle_01). Forcing Mesa's nouveau GL driver
(`NOUVEAU_USE_ZINK=0`, renderer `NV167`) renders every face.

| `validate_lidar_rendering.py`, 4 poses × 5 scans | zink (`lidar-rendering-zink-defect`) | nouveau GL (`lidar-rendering-fixed`) |
|---|---|---|
| front `/scan` beams missing (geometry in range, no return) | **42–50 %** (±45…90° sectors 99–100 %) | **0.0 %** |
| `/top_scan` beams missing | **52–68 %** | **0.3–1.1 %** |
| median error of the agreeing beams | 2–7 mm | 2–5 mm |
| real-time factor at the spawn (static) | 0.62 | 0.76 |

The remaining disagreements with nouveau GL are 0.5–3.9 % of front beams and 2–4 %
of top beams. They lie on cylinders or rotated parts that the ray cast does not
model, and on grazing edges. None is a missing sector.

**Fix (simulation infrastructure, no change to robot, world or perception):**

- `amr_description/launch/sim.launch.py` sets `NOUVEAU_USE_ZINK=0` for the Gazebo
  processes. Other GPU drivers ignore the variable, and an explicit value in the
  caller's environment wins.
- `amr_navigation/audit.py` records each scanner's widest arc with no return during
  the audit window. It fails if the front scanner's widest arc is 30° or more: the
  defect blanks 45°, while the enclosed warehouse leaves at most about 12°, the open
  main aisle. The 360° scanner's arc is only reported, since it can legitimately look
  out of an open door. Unit test: `test_audit_detects_blind_sectors`.
- `validate_lidar_rendering.py` is the acceptance check for this root cause.

**Effect on SLAM** (same 21-step plan, 71 m, 22.1 rad, fresh SLAM, front scanner):

| Run | Renderer | Odometry | SLAM parameters | Rotations, open junction | Rotations, facing wall | Whole run: SLAM final / max | Largest step |
|---|---|---|---|---|---|---|---|
| `diag-baseline` | zink | Phase 5 (−9.35 % yaw) | Phase 5 | 0.017–0.153 m | 0.069–0.731 m | 0.285 / **1.916 m** | 0.741 m |
| `diag-baseline-fixed-render` (untouched `3bdc33a`) | nouveau GL | Phase 5 (−9.35 % yaw) | Phase 5 | 0.002–0.015 m | 0.002–0.010 m | **0.020 / 0.097 m** | 0.087 m |
| `diag-fixed-render-phase5-params` | nouveau GL | calibrated | Phase 5 | 0.001–0.016 m | 0.001–0.023 m | **0.025 / 0.106 m** | 0.044 m |
| `diag-fixed-render-production` | nouveau GL | calibrated | tuned (below, superseded) | 0.016–0.032 m | 0.014–0.033 m | 0.025 / 0.123 m | 0.026 m |

With correct rendering, the untouched Phase 5 stack maps the plan to 2 cm, even with
its 9.35 % odometry rotation error. The rendering defect is therefore the root cause of
Problem A. The matcher tuning below no longer improves anything, so production
`slam_params.yaml` is the unchanged Phase 5 file again. The tuned copy is kept as
evidence (`plans/slam_params_tuned_production_candidate.yaml`).

**Effect on Problem B.** Under zink the front scanner saw only ±45°, not ±90°, and the
360° scanner was mostly blind. Workers approaching from more than 45° off the heading
were invisible to Nav2 and to the guard's sensor checks. Sensor-coverage acceptance
after the fix is in `PHASE_5_1_VALIDATION.md`.

The sections that follow were measured **before** this defect was found. They are
kept as the record of how the investigation got here; their status is updated.

## Root cause 1 — systematic odometry rotation error (CONFIRMED; correction kept, secondary)

Measured under the rendering defect, but wheel odometry does not depend on rendering.
The calibration below is a real kinematics correction for AMCL's motion model and
Nav2's rotation control. It is not needed for SLAM accuracy (table above).

Five in-place rotations of about 90° each and two straight legs, with ground truth
(`diag-baseline-pillar-stop`, `calibrate_odometry.py`):

| Motion | Odometry / truth | Interpretation |
|---|---|---|
| straight, 10.66 m and 8.42 m | 1.000007 (≤ 1.4 × 10⁻⁵ spread) | wheel radius exact (0.12000 m) |
| in-place rotation, 5 × ≈90° | **0.9065** (0.90643–0.90653) | odometry under-reports rotation by **9.35 %** |

The effective wheel separation is 0.58 × 0.9065 = **0.5258 m**. The simulated wheel
cylinders bear on their inner rims (2 × (0.29 − 0.0275) = 0.525 m), not on the
0.58 m centreline. The error is deterministic. Consequences:

- every in-place turn gives SLAM a heading prior off by about 0.145 rad per 90°;
- Nav2's commanded rotation rates are executed about 10 % fast.

During these rotations the robot does not translate (odometry and ground truth both
show 0.000 m), yet the SLAM estimate jumped **0.03–0.20 m**. The scan matcher absorbs
a large heading correction, and in a partial 180° view part of it leaks into
translation.

**Correction (odometry calibration, not a new controller):**
`amr_dimensions.json` gains `odometry_wheel_separation = 0.5258`. The parameter
generator writes it into the existing `diff_drive_controller` `wheel_separation`.
The geometric `track_width` (0.58 m) still places the wheels. A new generator check
requires the value to lie between the wheels' inner and outer rims. The Blender
rebuild test, which deliberately changes the track, now also supplies its own
inner-rim separation.

## Former root cause 2 — false translation during in-place rotation (SUPERSEDED: a symptom of root cause 0)

With correct rendering, the same rotations produce 0.002–0.015 m
(`diag-baseline-fixed-render`). The measurements below are what a ±45° scanner
produces.

The full baseline (`diag-baseline`, untouched `3bdc33a` build, fresh SLAM, front scanner)
ran 21 legs and 70.3 m, with rotations and drives recorded as separate segments.
Plot: `diag-baseline/drift-errors.png`.

| Segment | Distance / rotation | SLAM error in segment | Odometry error in segment |
|---|---|---|---|
| A main aisle straight | 10.66 m | 0.024 m | 0.003 m |
| B north cross aisle straight | 8.36 m | **0.016 m** | 0.007 m |
| C south cross aisle straight | 8.37 m | **0.011 m** | 0.010 m |
| west service aisle straight | 16.47 m | 0.049 m | 0.010 m |
| D rotations in the open junction (4 × 90°) | 1.55–1.57 rad each | 0.017–0.153 m (max 0.233) | 0 m, −0.145 rad each |
| E rotations facing the north wall (4 × 90°) | 1.51–1.56 rad each | **0.069–0.731 m** | 0 m, −0.141…−0.146 rad each |
| turn at the wall, toward the west | 1.63 rad | **0.484 m** | 0 m, −0.152 rad |
| F return into the known main aisle | 7.78 m | 1.346 m, including one **0.741 m step** (loop closure) | 0.009 m |
| whole run (aligned at start) | 70.3 m | final 0.285 m, max 1.916 m | final 17.5 m / −2.05 rad (rotation errors integrated) |

- Straight travel, **including the north and south cross aisles**, is accurate. The
  Phase 5 hypothesis that "repetitive rack ends alias the scan matcher" is not
  supported once turns are separated from drives. Each Phase 5 survey leg began with a
  turn at a junction near the wall, so the error was attributed to the wrong part of
  the leg.
- Every rotation hands SLAM the 9.35 % heading-prior error from root cause 1. In open
  junctions the matcher mostly recovers. Facing a flat wall with a 180° view it
  converts the correction into up to **0.73 m of false translation** while the robot
  is stationary.
- Loop closure later pulls the estimate back (the 0.74 m step). A map saved before
  closure therefore contains ghosted walls, exactly as the rejected Phase 5
  candidates did.

## Former root cause 3 — scan-matcher accumulation during in-place rotation (SUPERSEDED: a symptom of root cause 0)

The "360° scan" run below used a scanner that was 52–68 % blind, so the conclusion
that field of view does not matter is withdrawn.

The same 21-leg plan, rerun after the odometry calibration:

| Run | Odometry yaw error per 90° | SLAM false translation, open junction (D) | SLAM false translation, facing wall (E) |
|---|---|---|---|
| baseline, front scan (`diag-baseline`) | −0.145 rad | 0.017–0.153 m (max 0.233) | 0.069–0.731 m |
| calibrated odometry, front scan (`diag-calibrated-front`) | **0.000 rad** | 0.037–0.402 m | 0.009–0.563 m |
| calibrated odometry, 360° scan for SLAM (`diag-calibrated-360`) | 0.000 rad | **0.064–0.625 m** | not reached (see below) |

- With an exact odometry prior and the robot stationary, SLAM still accumulates
  several decimetres of translation per rotation. A full-circle view does not prevent
  it, so the field-of-view hypothesis is **not supported** for SLAM. The mechanism is
  in the scan matching itself.
- The Phase 4 parameters process a scan every 0.03 rad (≈52 matches per 90°). Each
  match may move the pose within a ±0.5 m window
  (`correlation_search_space_dimension: 1.0`; Karto's documented default is 0.3 m).
  The odometry penalty is weak (`distance_variance_penalty: 0.5`). Each match nudges
  the pose a little and the nudges accumulate.
- In `diag-calibrated-front` the accumulated error left the robot physically at
  y = 9.2 m (ground truth) while SLAM believed y = 7.2 m. The new 360° rotation
  check in `mapping_jog.py` refused the next turn at 0.80 m from the wall: the safety
  addition working on a real hazard. `diag-calibrated-360` stopped at leg 6 on the
  forward-sector stop for the same reason.

## SLAM matcher tuning (measured under the defect; REVERTED)

Reverted: with correct rendering the Phase 5 parameters are as accurate as or more
accurate than this tuning (root-cause-0 table). Production `slam_params.yaml` is the
Phase 5 file.

Experiment copy `plans/slam_params_odometry_trust.yaml`, identical plan
(`diag-calibrated-front-tuned`). The front scanner is unchanged. Only four parameters
changed:

| Parameter | Phase 4 | Phase 5.1 | Reason |
|---|---|---|---|
| `minimum_travel_heading` | 0.03 rad | 0.087 rad (Karto default, 5°) | about 3× fewer match updates per turn |
| `correlation_search_space_dimension` | 1.0 m | 0.3 m (Karto documented default) | a single match can no longer jump ±0.5 m |
| `distance_variance_penalty` | 0.5 | 0.02 | the calibrated odometry translation is accurate (≤ 1.4 × 10⁻⁵ scale error) |
| `angle_variance_penalty` | 1.0 | 0.3 | the calibrated odometry yaw is accurate (0.000 rad per 90°) |

| SLAM error per segment | baseline | calibrated | **calibrated + tuned** |
|---|---|---|---|
| open-junction 90° rotations | 0.017–0.233 m | 0.037–0.402 m | **0.015–0.066 m** |
| wall-facing 90° rotations | 0.069–0.731 m | 0.009–0.563 m | **0.034–0.059 m** |
| turn at the wall | 0.484 m | 0.182 m | **0.019 m** |
| north cross aisle, straight | 0.016 m | 0.015 m | 0.010 m |
| largest single-step jump | ≤ 0.143 m | ≤ 0.120 m | **≤ 0.016 m** |

The production `config/slam_params.yaml` was then temporarily set to exactly the tested
copy (verified identical; now reverted). Residual per-turn errors of a few centimetres still accumulate
without revisits: about 0.5 m after 13 segments. Survey plans therefore return through
already-mapped aisles so loop closure can act. SLAM and AMCL keep using the front
scanner: the 360° scanner did not improve SLAM, and keeping the input unchanged
preserves Phase 4 localization.

## Mapping procedure changes

- **Turns and drives are separate steps.** Previously each leg mixed a turn with a
  drive, which attributed rotation error to corridor travel.
- **Turns happen at junctions only.** Turns near end walls are avoided where possible,
  and plans stop north/south passes at |y| = 8.3 m (8.0 m at aisle_02) so the forward
  stop clears the pillars at (0, ±9.7).
- **`mapping_jog.py` refuses blind rotations.** A rotation needs a fresh 360° scan:
  freshness is judged by simulation stamp (≤ 0.6 s), with a 3 s wall-clock stall
  bound. It also needs no return within 0.80 m of `base_link` throughout the turn.
  Without 360° observation, rotation is refused unless the operator passes
  `--allow-unobserved-rotation`. The forward-sector translation stop is unchanged.
  This check stopped two real hazards in the experiments. A step whose heading is
  already within 0.02 rad commands no rotation and therefore needs no clearance; the
  first survey attempt had stopped there before the 360° scan's first message. The
  survey runner passes `--allow-unobserved-rotation` only when asked explicitly
  (used solely for the Phase 5 robot, which has no 360° scanner) and records it in
  the report.
- **Every survey records synchronized SLAM, odometry and ground truth**
  (`extend_map_survey.py --record`), so a candidate's drift is measured, not inferred
  from the final image.
- Coverage survey `plans/coverage-survey.json`: a fresh map at the spawn, a serpentine
  through every corridor at full length, 163 m, returning through the main aisle.

## Candidate maps

| Survey | Configuration | Result |
|---|---|---|
| `survey-v7-coverage` → `candidate-v7-partial` | calibrated odometry, tuned matcher, Phase 4 loop closure (`loop_search_maximum_distance` 3.0 m, window 8.0 m) | **Rejected** by the registration gate: median residual 0.131 m (> 0.10), inlier fraction 0.62 (< 0.70), p90 0.73 m. Whole-warehouse coverage and a clean north half. One **false loop closure** on entering the south cross aisle from aisle_01 moved the estimate 1.41 m away from ground truth (global error 0.10 → 1.48 m); the south racks and walls are skewed and doubled. Correct closures in the same run reduced the error from 0.53 → 0.014 m and 0.85 → 0.20 m. Leg 30 was stopped by the rotation-clearance check, 0.72 m from an obstacle |

| `survey-v8-coverage` → `candidate-v8` | as v7, plus loop candidates limited to 1.5 m (window 3.0 m) | **Rejected** by the registration gate: median 0.129 m, inlier fraction 0.64 (p90 0.50 m). The survey completed all 34 legs, with no false closure (peak live error 0.86 m, final 0.41 m). The map covers the whole warehouse with racks aligned in both halves, but residuals are uniformly elevated: typical 4 m tiles 0.10–0.20 m, worst 0.53 m. Restricted to cells on structure boundaries (outside cells and cells inside obstacle footprints excluded), the median is 0.147 m against 0.051 m for the approved map: real distributed pose error, not a metric artefact. Occupied streaks cross the aisle openings at the rack ends (y ≈ ±7) |

Two hypotheses for those streaks were tested with real sensors:

- **Floor markings hit by a pitching scan plane: refuted.** The only geometry along
  y = ±7.025 is a 3 mm painted marking, but Gazebo pitch stayed at exactly 0.00°
  at rest, accelerating, braking and rotating (`pitch-probe`), and the straight-ahead
  range never landed on the line.
- **Real structure in the openings: refuted.** Raw `/scan` and `/top_scan` endpoints
  from four placements put essentially nothing inside the aisle_01 opening (one stray
  return; `opening-probe`).

The conclusion drawn at the time, that the streaks were rack-end faces inserted at
slightly wrong poses, was **wrong**. They are phantom returns from the rendering
defect (root cause 0). `opening-probe` saw nothing in the opening because the probe
poses looked at it through the scanner's working forward face.

**Measurement at the time (superseded).** Over the whole 161.5 m / 28.5 rad v8 survey, calibrated
wheel odometry drifted **0.055 m and 0.005 rad**, while the scan-matched SLAM estimate
peaked at **0.87 m**. The same holds on the 27 m diagnostic plan: odometry 0.030 m,
tuned SLAM 0.526 m. In this simulator, once calibrated, scan matching degrades the
pose by more than an order of magnitude, and loop closure adds the risk of false
closures (v7).

Survey v9 therefore used an odometry-pose profile (scan matching and loop closing
disabled). That profile was **withdrawn** once root cause 0 was found. It now lives
only as the evidence copy `plans/slam_params_sim_odometry.yaml`. The loop-closure
narrowing tried in v8 (1.5 m / 3 m window) was reverted with the rest of the tuning.

`survey-v9-coverage-odometry` → `candidate-v9-partial` used the withdrawn odometry-pose
profile and completed 28 of 34 legs. The stop was a TF start-up race in
`mapping_jog.py`, since fixed. The candidate passed the registration gate (median
0.050 m, inlier fraction 0.743, loop return 0.019 m). However, its graph had **0
validated cycles**: phantom streaks blocked 11 connections (13–22 occupied cells under
the footprint), and staging was blocked as well. **Superseded.**

### Candidates after the rendering fix

All use the same 34-step coverage plan (163 m), fresh SLAM, the front scanner,
calibrated odometry and correct rendering. Each survey also records a raw bag
(`--bag`) so SLAM parameters can be compared offline on identical data
(`replay_survey.py`, SLAM Toolbox's standard offline workflow).

| Candidate | SLAM parameters | Survey error vs truth: final / p90 / max; largest step | Registration (median, inlier 0.20) | Worst 4 m tile | Graph: nodes / connections validated; stations; station pairs; cycles | Verdict |
|---|---|---|---|---|---|---|
| `survey-v10-coverage-fixed-render` (live) | Phase 5 (loop candidates 3 m, window 8 m) | 4.19 / 2.89 / **5.23 m**; 0.81 m | **rejected**: 0.148 m, 0.57 | — | — | False loop closures between the identical aisles, 4.2 m apart: steps of 0.66 m on aisle_02, then 0.8 m |
| `replay-v10-phase5-params` (bag of v10) | Phase 5 | 0.46 / 0.98 / 1.58 m; 0.63 m | **rejected**: yaw disagrees with the controlled prior (median 0.095 m, 0.73) | — | — | Same failure, at a different time: async loop closure is timing-sensitive |
| `candidate-v10-replay-loop-narrow` (bag of v10) | loop candidates 1.5 m, window 3 m | 0.145 / 0.41 / 0.54 m; 0.13 m | accepted: 0.069 m, 0.836 | 0.219 m (1 tile > 0.20, 41 cells at the dispatch door) | 29 / 34; all 6; 30 / 30; **6** | Meets loop and global targets; misses the local target by 0.019 m |
| **`candidate-v11`** (`survey-v11-coverage-production`, live) | **production** = loop candidates 1.5 m, window 3 m | **0.003 / 0.42 / 0.55 m**; 0.39 m | accepted: **0.065 m**, 0.841 | **0.170 m** (0 tiles > 0.20) | 27 / 29; all 6 incl. staging; **30 / 30**; **3** | **Meets every engineering target; approved by the operator 2026-10-30 and installed** |

Engineering targets for v11 (they did not change any acceptance threshold):

- loop return 0.003 m / 0.001 rad (target ≤ 0.25 m / 0.05 rad);
- global registration median 0.065 m (≤ 0.10);
- worst local tile 0.170 m (≤ 0.20).

**Promotion.** The operator approved `candidate-v11` on 2026-10-30. It is now
`amr_navigation/maps/warehouse.{yaml,pgm}`; `provenance.json` records the survey,
hashes and quality. The Phase 4 v3 map package is archived in `approved-map-v3/`.
Registration `788d6c32e623626e` was recalibrated and is identical to the candidate's.
The graph was rebuilt as `31fba3aa050bd251`, with the same statuses as the candidate.

Production `slam_params.yaml` therefore differs from Phase 5 in two values only:
`loop_search_maximum_distance` 3.0 → 1.5 m and `loop_search_space_dimension`
8.0 → 3.0 m. Both values are justified by v10 and its replays under correct rendering.

**Blocked graph elements are near-limit design margins, not artefacts.** A node is
validated when the circumscribed disc (0.666 m) plus the required clearance (0.30 m),
0.966 m in total, is free. The candidates block or validate these nodes depending on
centimetre-level differences:

- `north/south_cross_aisle__aisle_02` lie 1.024 m from the real pillar guards at
  (0, ±9.7) in the layout, a 5.8 cm margin;
- `south_cross_aisle__west_service_aisle` lies about 0.99 m from a small real object
  at (−9.75, −8.45), present in every map. It is consistent with the staging-area sign
  mesh, which the ray cast does not model.

v10-replay validated all three; v11 blocks them, which closes 8 connections. No
clearance was reduced and no node was moved. Giving these nodes margin is a layout
change, recommended for Phase 6.

## Sensor architecture

Front-scanner geometry (measured by BVH ray casting against the generated meshes in
`validate_amr.py`): the scanner sits in an open front bay, 82 mm deep, at x = 0.44 m,
z = 0.213 m. Rays stay clear of the robot up to **±98.1°**. An analytic estimate from
the bay geometry gives 97.6°. Beyond that they strike the chassis shoulders, so the
`≤ 180°` rule is a real physical limit, not an arbitrary one. Until root cause 0 was
fixed, the simulated scanner actually delivered only ±45°. The geometric comparison
below holds independently of that defect.

| Option | Coverage | Physical validity | Cost / risk |
|---|---|---|---|
| A — keep the front 180° scanner | front half-plane only. Sides and rear blind. The remaining Phase 5 contacts came from bearings of ≈77° (≈116° from the laser) and −155° | valid | none, but leaves the safety gap |
| B — add a rear scanner in a mirrored rear bay | each end limited to ≈196°. A **lateral blind wedge** remains beside the chassis middle: a worker aimed at the robot's mid-side is invisible within ≈1.8–3.7 m | needs a rear bay cut and changed chassis collisions | chassis geometry change, still blind beside the robot |
| C — elevated 360° scanner on a mast at the platform rear edge | full circle; scan plane at 0.48 m, above every robot part | valid: all 720 beams verified clear of every robot mesh | small mast on the platform (payload must stay below the scan plane); one more sensor stream |

Corner-mounted 270° scanners would need housings outside the chassis outline. The
visual chassis otherwise occludes rays along the sides. They were not pursued.

**Selected: C**, added beside the unchanged front scanner. The front scanner keeps its
role for low obstacles below 0.48 m, and is the Phase 4 SLAM/AMCL source unless the
measurements below justify otherwise. The scanner is added through the existing
pipeline:

- parameters: `top_lidar_*` in `amr_dimensions.json`, with derived geometry and checks
  in `amr_parameters.py`;
- `urdf/sensors/top_lidar.xacro`: links `top_lidar_mast`, `top_lidar_link` and
  `top_laser_frame`;
- Gazebo `gpu_lidar`: 720 samples, a full circle without a duplicated ±π beam, 10 Hz,
  0.12–20 m;
- bridge `/amr/top_scan → /top_scan`;
- Blender meshes, plus a 720-ray self-occlusion validation;
- web GLB regenerated (height 0.474 m, 12 meshes);
- URDF validator updated: 18 links, 4 sensors, `top_laser_frame` at (−0.38, 0, 0.48).

**Nav2:** each scanner has its own obstacle layer (`obstacle_layer` from `/scan`,
`top_obstacle_layer` from `/top_scan`) in both costmaps. The elevated scanner's
clearing rays must not erase low obstacles that only the front scanner observes.

**Command guard:** it now also requires a fresh `/top_scan` with frame
`top_laser_frame`, at least one valid return, and a `base_link → top_laser_frame`
TF. It stays fail-closed: any stale or invalid scanner gives zero velocity. The
sensor audit checks 720 beams, a full circle and 8–11 Hz.

## Pedestrian yielding (HumanSystem)

The physical servo is unchanged: force and torque, braking-limited speed and the
displacement-driven gait. A deterministic yielding layer only caps the target speed.

- At 50 Hz the worker predicts its own path at nominal speed toward its waypoint, and
  the AMR's path from its pose and finite-difference velocity (0.2 s low-pass), over
  a 3 s horizon in 0.1 s steps.
- A conflict is the first predicted time the worker comes within radius + 0.45 m of
  the AMR's oriented envelope (0.50 × 0.35 m half extents, validated ≥ the chassis).
  It counts only if the AMR is ahead of the worker: a robot overtaking from behind is
  the robot's to avoid.
- On a conflict the speed cap is √(2·braking·max(0, s_conflict − 0.10)), so the
  worker stops before the envelope. The gait stops naturally because it is
  displacement-driven.
- Release requires 1.0 s without a conflict and a current clearance above
  radius + margin + 0.15 m (hysteresis).
- Deadlock: if the AMR stays stationary and blocking for more than 25 s, the worker
  turns back to its previous waypoint. That is deterministic and reuses a segment it
  has already walked.
- Configuration comes from `warehouse_layout.json → human_model.yielding`. The
  compiler validates it: positive values, envelope ≥ AMR chassis, and a look-ahead
  longer than stopping distance plus margin.
- The worker uses simulator state for its own decisions only. The state is published
  on gz-transport `/warehouse/humans/<id>/yield` for evaluation and never reaches
  the AMR's perception.
- `warehouse.launch.py human_yielding:=false` (logged as TEST MODE) is used only by
  `validate_crossing_contact.py`. That way the intentional physical-contact test keeps
  its original assertions.

## Route execution: simulation-time deadline

In the first candidate station run (`candidate-v11-nav-stations`), dropoff_02 →
pickup_02 ended `FAILED: Route wall timeout of 600.0 s` after 26.4 m. The run had
zero contacts, a minimum worker–envelope distance of 0.66 m and nine Nav2
recoveries.

- **What happened.** The route's last objective,
  `pickup_station_02_via_west_service_aisle` at (−8.7, 5), lies on human_01's patrol
  line (x = −8.7, y = 3 … 6). The worker walks over it about every 10 s and pauses
  6 s at each end. Whenever the worker was near that objective, `ComputePathThroughPoses`
  failed for the whole remaining route ("Failed to create plan with tolerance of
  0.2"), even with the robot 15 m away, and the robot waited. That is correct, safe
  behaviour.
- **What was wrong.** The executor's 600 s budget was measured in wall-clock time.
  With the full stack and the browser capture running, the real-time factor was 0.36,
  so the robot had only 216 s of simulated time. How long it may wait for a person
  must not depend on host speed.
- **The worker was not the cause.** Its telemetry shows one yield of 11.7 s, then
  release, so its bounded wait already held. HumanSystem is unchanged.
- **Change.** `execute_route --timeout` (still 600 s) now counts **simulation time**.
  A separate `--wall-guard` (3000 s) bounds a stalled simulator. The failure reason
  names which bound expired. No navigation tolerance, clearance or Nav2 parameter
  changed.

`observe_obstacles.py` also gained `--candidate`. In candidate runs it previously
mapped worker positions with the approved map's registration, which put them about
0.3 m off at 15 m (evaluation data only). `route_integration.py --candidate` now
passes the flag.

## Registration consolidation

`coordinate_registration.active()` is now the only way evaluation tools obtain
world → map. It loads the record and verifies it against the installed map's content
version and the layout hash, and raises instead of falling back to an offset.

- `observe_registration.py`, `observe_obstacles.py` and `analyze_results.py` no
  longer use the hard-coded `+11 m`.
- `observe_obstacles.py` also evaluates both scanners and the global costmap.

The registration is never published as TF.
