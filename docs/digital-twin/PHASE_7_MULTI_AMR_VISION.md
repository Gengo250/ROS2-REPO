# Phase 7 — Three AMRs and synthetic-data computer vision

Phase 7 extends the lite twin (Phase 6) from one operational AMR to three (`amr_01`, `amr_02`,
`amr_03`) and adds an object detector for their onboard RGB cameras: YOLO26n, fine-tuned only on
synthetic images rendered from the twin itself, with ground truth taken from an instance-ID render
pass. In the browser, the user selects an AMR, opens its onboard camera and sees the boxes the
trained network predicted on the frames it processed for that robot.

Gazebo stays optional; nothing here depends on it. All numbers below were measured on this machine
(Intel i5-10300H, 4 cores / 8 threads, 16 GB; Intel UHD (display) + GTX 1650 4 GB, NVIDIA driver
595.99.02) on 2026-11-14 unless stated otherwise. Evidence: `docs/digital-twin/evidence-phase7/`
(git-ignored), training run `vision/runs/results/2026-11-14_001/` (git-ignored).

| Acceptance criterion | Result |
|---|---|
| Three real AMR entities in the lite twin | `amr_01..03`, same model, engine physics and sensors; independent topics, TF, commands, Nav2 stacks |
| Identities, commands and sensors isolated | `validate_fleet.py` all checks passed; idle robots moved 0.0 m while another one drove |
| Frontend shows the three robots | robot selector, per-robot routes, per-robot onboard camera (`vision.browser.ts` passed) |
| Onboard cameras produce real RGB images | Three.js render worker: RGB 640×480 + instance-ID pass from the twin's own renderer |
| Reproducible, automatically annotated dataset | 3000 frames, seed 42, labels from visible pixels; validation passed (0 errors) |
| YOLO26n trained on it, results and checkpoints saved | 60 epochs on the GTX 1650, run `2026-11-14_001` |
| Validation and test metrics | test split (480 images, 1759 objects): P 0.889, R 0.752, mAP50 0.838, mAP50-95 0.639 |
| Inference on camera images, per robot and frame | perception service: 3.0 FPS per robot × 3, 0 dropped, frame id + sim stamp + robot on every result |
| User sees genuine boxes in the onboard view | VISÃO COMPUTACIONAL panel; boxes drawn on the processed JPEG itself |
| Simulation and navigation still run without Gazebo | yes; see the performance section for the cost of three Nav2 stacks on this CPU |

## 1. Multi-AMR

### Instances and spawns

The three robots are three active spawns of the authoritative scene
(`src/atlas_scene/compiler/campus_navigation.mjs`, compiled into `atlas_campus.scene.json`); the
engine instantiates one robot per active spawn with the existing AMR model (dimensions, collision
footprint, differential drive, sensors, appearance). No URDF or geometry was duplicated.

| Robot | Spawn (world x, y, yaw) | Why there |
|---|---|---|
| `amr_01` | (2.0, 0.0, π/2) | unchanged: the SLAM map and `campus_registration.json` are anchored at this pose |
| `amr_02` | (−5.0, 0.0, π/2) | 7 m west, outside the spine's forklift lanes (x = −1 and 5) |
| `amr_03` | (9.0, 0.0, π/2) | 7 m east, same reasoning |

All three are ≥ 13 m from every worker path, on free cells of the installed map and ≥ 0.6 m from
structure. `atlas_scene` test `test_three_active_robots_with_separate_clear_spawns` enforces ≥ 3 m
between robots, ≥ 2.5 m from lanes, ≥ 3 m from worker paths. The warehouse geometry was not changed.

### ROS interface (namespaces and frames)

`amr_lite_sim/fleet.py` defines the naming: the root robot `amr_01` keeps the Phase 6 names
(backward compatible), the others are namespaced and frame-prefixed.

| | `amr_01` | `amr_02` (`amr_03` alike) |
|---|---|---|
| Odometry | `/odom` | `/amr_02/odom` |
| Velocity command (TwistStamped) | `/cmd_vel` | `/amr_02/cmd_vel` |
| Front / top LiDAR | `/scan`, `/top_scan` | `/amr_02/scan`, `/amr_02/top_scan` |
| IMU, joints | `/imu/data`, `/joint_states` | `/amr_02/imu/data`, `/amr_02/joint_states` |
| TF | `odom → base_link → …` | `amr_02/odom → amr_02/base_link → …` |
| Camera (perception service) | `/camera/image_raw`, `/camera/camera_info` | `/amr_02/camera/image_raw`, `/amr_02/camera/camera_info` |
| Detections (perception service) | `/perception/detections` | `/amr_02/perception/detections` |

Shared: `/tf`, `/tf_static`, `/clock`, the `map` frame. The engine node creates one `RobotIO` per
robot (own publishers and its own `cmd_vel` subscription: no node can move another robot);
parameters `robots` (default `all`) and `root_robot`. One `robot_state_publisher` per robot with
namespace and frame prefix. `World(..., robots=[...])` instantiates a subset and rejects unknown or
duplicate ids. Robot–robot contact and scanner visibility of other robots already existed in the
engine and now apply between operational robots: on contact a robot advances to the last
collision-free fraction of the step and stops (`blocked`, `contact:<id>`); the front safety scanner
sees other AMRs (the 360° top scanner is mounted above their height).

### Navigation

Each robot gets its own complete Nav2 stack (map server, AMCL, planner, controller, behaviours,
BT navigator, lifecycle managers, command guard, initial pose) in its namespace;
`lite_navigation.launch.py` includes `navigation.launch.py` once per robot.

- Parameters: `amr_navigation/launch_support.py` `rewrite_for_robot` prefixes `base_link`/`odom`
  frame values and namespaces absolute topic values (scan, odom) and nests the tree under the
  namespace. The root robot uses the validated `nav2_params.yaml` unchanged; the others get a
  temporary rewritten copy, so the file that is part of the Phase 5 graph hash is never edited (the
  route graph was not regenerated).
- Initial pose: each spawn converted world → map with the verified registration (`amr_01` lands at
  map (0, 0, 0)).
- `command_guard` and `initial_pose` use relative topics and a `frame_prefix`; the guard's
  conflicting-publisher check looks at its own resolved `cmd_vel`. `launch_checks.exclusive_mode`
  refuses a second stack only inside the same namespace.
- Stacks start 10 s apart (`stack_stagger`): started together, one robot's localization lifecycle
  manager lost an activation response and AMCL stayed inactive.
- Other AMRs are dynamic obstacles through each robot's scanners and costmaps; there is no fleet
  traffic coordinator (robots do not reserve corridors).
- Mapping (`lite_mapping.launch.py`) forces `robots:=amr_01` (other robots would be mapped as walls).

### Gateway and frontend

- Gateway: fleet `amr_01` (root namespace), `amr_02`, `amr_03` (`Settings.fleet`, overridable with
  `TWIN_FLEET` JSON; ids and namespaces must be unique). Per robot: plan topic, navigation
  diagnostics (interlock), one `NavigateToPose` action client; missions are owned per robot
  (`POST /api/v1/missions {robot_id, destination}`, `DELETE /api/v1/missions/{robot}`), unknown
  robots are rejected. World-state `robots[]` carries each robot's interlock, path and mission;
  `connection.navigation` is the least available robot's stack.
- Frontend (LIVE): robot selector in "MISSÕES DOS AMRs" (also follows the AMR clicked in the
  scene), one route and goal per AMR (the minimap highlights the selected robot's), the selected
  robot's operational card, "Ver pela câmera do AMR" opens that robot's onboard view. DEMO unchanged.

### Multi-AMR validation (Nav2 running)

`src/amr_lite_sim/scripts/validate_fleet.py` (`evidence-phase7/multi_amr/validate_fleet_start.json`):
sensor topics independent, one engine `cmd_vel` subscriber per robot, complete and unique TF chains
with a single parent per frame, AMCL within 0.25 m of ground truth (0.008 / 0.024 / 0.021 m) —
all passed.

`run_fleet_missions.py` (simultaneous missions through the Gateway, `multi_amr/fleet_missions.json`,
`fleet_head_on.json`):

| Round | Missions | Result |
|---|---|---|
| Isolation | `amr_02 → pick_south_w2` alone | succeeded (46.5 m); idle `amr_01`, `amr_03` moved 0.0 m |
| 3 simultaneous | `amr_01 → dock_05`, `amr_02 → production_w`, `amr_03 → charger_01` | 3/3 succeeded, goal error 0.05–0.16 m, min gap 6.96 m |
| 3 simultaneous, long | `amr_01 → pick_north_e1`, `amr_02 → dock_03`, `amr_03 → receiving_01` | 3/3 succeeded (108–139 m), goal error 0.05–0.08 m |
| Crossing routes | `amr_02 → receiving_01`, `amr_03 → dock_03` | 2/2, 0 recoveries, min gap 1.55 m |

0 contacts (people, objects or robots) in every round. Unit tests: `atlas_scene` 12, `amr_lite_sim`
28, `amr_navigation` 32, gateway 56, frontend 30 (vitest), all passing; ATLAS DEMO browser
regression passed.

## 2. Computer vision

### Classes

One mapping everywhere (`vision/configs/dataset.yaml`, `atlas_vision/classes.py`, the web worker,
the checkpoint, the panel): `0 person, 1 forklift, 2 amr, 3 pallet, 4 box, 5 cone`. Each exists in
the rendered scene and gets exact per-pixel ground truth: the 16 workers, 3 forklifts, the other
two AMRs, user-placed cones/boxes/pallets (engine obstacles), the floor pallets (`Palete no chão`,
base plus load) and the boxes on the inspection tables. Racked loads are part of the racks (not
labelled): they cannot be separated reliably from the rack instances.

### Camera images (render worker)

The lite engine has no renderer, so a real image source was added that reuses the twin's renderer
instead of duplicating the scene: `web/digital-twin/camera.html` (`src/camera/`) loads the compiled
scene, `InstancedRenderer` (ATLAS shaders), `buildStatic`, `LiveScene` and the rigs, and places a
camera with `cameraPoses.onboardPose` (URDF `base_link → camera_optical_frame`, 75° horizontal FOV,
square pixels). Per frame it renders off screen into render targets of the requested size (default
640×480): an antialiased RGB pass and, on request, an instance-ID pass of the same camera and
geometry (flat per-instance codes, no antialiasing). Only physical things are drawn — no safety
fields, routes, selection, labels or HUD; the capturing robot's own body is hidden; walls and roof
closed. Two modes: *jobs* (render exactly the state supplied: dataset, scenarios) and *LIVE*
(render the newest Gateway world state, stamped with its simulation time and sequence; refused when
the state is not online or the scene hash differs). Every image comes from the renderer: a missing
or failed worker raises; there is no substitute stream.

The worker runs in Playwright's Chromium, started by the Python side (`atlas_vision/render_worker.py`),
so inference does not depend on someone keeping the twin page open. Browser modes (`--browser`):

| Mode | How | RGB + ID frame, 640×480 |
|---|---|---|
| `nvidia` (auto's first choice) | headless Chromium, ANGLE over EGL on the NVIDIA driver | 19 ms round trip, 5 ms render (GTX 1650) |
| `gpu` | minimised window on the display GPU | ~40 ms (Intel UHD) |
| `headless` | SwiftShader on the CPU | ~375 ms |

The `nvidia` mode keeps camera rendering off the Intel GPU, which the twin page already uses at
74–78 % (below). Equivalence check (12 identical states, Intel vs NVIDIA): mean RGB difference 0.04
grey levels (max 0.19), instance-ID passes identical, labels identical — so a model trained on
Intel-rendered frames sees the same pixels at run time.

### Dataset generation and annotation

`vision/scripts/generate_dataset.py` runs the lite engine in-process (one seeded world per
session: 16 workers and 3 forklifts with their own behaviour, 20–150 s warm-up, 3–10 obstacles
placed through the engine's validated `add_obstacle`, day or night shader lighting) and asks the
worker for RGB + ID images of the exact engine state.

- Views: *targeted* (camera robot teleported 1–13 m from a person, forklift, other AMR, obstacle or
  floor pallet/table box, heading jittered ±0.55 rad, pose collision-checked by the engine) and
  *driving* (velocity commands through the engine: moving camera, collisions, safety scanner).
- Labels: for each instance code in the ID pass, the visible pixels give the box (visible part only:
  occlusion and image borders are accounted for by construction; objects behind walls have no
  pixels). Instances under 25 px or with a box side under 4 px are not labelled and are recorded as
  `ignored`. Boxes are written in YOLO format `class xc yc w h` (normalised). Labels never come from
  a model.
- Frame gate: frames whose grey-level standard deviation is below 8 (camera against a wall) or that
  repeat the robot's previous frame (stuck robot) are rejected; the robot is moved to a free pose and
  the frame re-rendered. Added after the first 3000-frame dataset failed validation: ~26
  point-blank wall frames and 2 groups of identical frames from robots stuck in *driving* mode.
- Splits: by session (70/15/15, seeded), never by frame, so consecutive frames never cross
  train/val/test; the test split is used only by `evaluate.py`.
- Metadata: `metadata/frames.jsonl` (per image: split, session and seed, robot, view mode and
  target, simulation time, lighting, camera and base pose, instances, labelled/ignored objects with
  pixels, fill, truncation, distance) and `metadata/generation.json` (configuration and hash, scene
  and geometry hashes, git commit, renderer, versions, session splits, rejected frames).

Dataset in use (`--samples 3000 --robots all --seed 42`, 1877 s, Intel `gpu` mode, config hash
`c81773112beb7102`, scene `2429fd944ac5`, 175 MB, git-ignored):

| | train | val | test |
|---|---|---|---|
| Images (sessions) | 2080 (52) | 440 (11) | 480 (12) |
| person / forklift / amr | 1696 / 1029 / 400 | 336 / 196 / 97 | 355 / 263 / 83 |
| pallet / box / cone | 3035 / 1035 / 285 | 737 / 260 / 57 | 769 / 229 / 60 |
| Empty images | 110 | 28 | 23 |

1040 night images; frames per robot 1018 / 974 / 1008; 2106 targeted, 894 driving; frame gate
rejections 18 uniform + 3 repeated. Object sizes: 6478 small (< 32 px), 2467 medium, 1977 large;
median distance 23 m. Imbalance: cone 11.3× and amr 7.8× fewer than pallet (cones exist only as
user obstacles, AMRs are two other robots).

### Dataset validation

`vision/scripts/validate_dataset.py` (refuses training unless passed): image/label/mask pairing,
label syntax, class ids, coordinates in range, boxes inside the image, **every label equal to the box
recomputed from the saved instance mask** (image–label alignment), metadata ↔ label agreement, exact
duplicates, near-duplicates across splits, session leakage, nearly uniform images, empty-image share,
class balance, size distribution; contact sheets for manual inspection (procedure in
`vision/README.md`). Near-duplicates: dHash ≤ 2 bits as a candidate, confirmed when 64×48 colour
thumbnails differ by < 5 grey levels — dHash alone matched 43 pairs of distinct images made of plain
horizontal bands (wall above floor) whose thumbnails differ by 12–54 levels (consecutive frames of a
moving robot differ by 6.7 at the 5th percentile). Result: **passed, 0 errors**, 0 cross-split
near-duplicates, 0 exact duplicates. Manual inspection of the sample and difficult-object sheets:
boxes tight on visible pixels, truncated and occluded objects boxed on their visible part,
foreground objects of difficult frames labelled.

### Training

```bash
uv run python scripts/train.py --config configs/train.yaml
```

YOLO26n from COCO-pretrained `yolo26n.pt` (transfer learning), 640 px, 60 epochs, batch 8,
patience 15, seed 42, deterministic, augmentations without geometry the fixed camera cannot produce
(no rotation, shear, perspective or vertical flip; horizontal flip, mosaic, scale 0.4, HSV jitter).
GTX 1650: batch 8 = 2.7 GiB in FP32 (AutoBatch profile; 16 does not fit in 3.6 GiB). Ultralytics'
AMP check fails on GTX 16xx cards (FP16 NaNs) and disables AMP, so training ran in FP32: 94.7 min,
~1:35 per epoch. Run directory `runs/results/2026-11-14_001/`: `config.yaml`, `run.json` (dataset
version, seeds, torch 2.15.0+cu132, Ultralytics 8.4.212, GPU), `checkpoints/{best,last}.pt`,
`metrics.csv`, `figures/` (box/cls/L1 losses train+val, precision, recall, mAP50, mAP50-95,
learning rate, confusion matrices, PR, F1, P and R vs confidence), `predictions/`, `reports/`.

Validation split, best epoch 59 of 60: P 0.894, R 0.742, mAP50 0.835, mAP50-95 0.625. Training and
validation losses decrease together (no overfitting); mAP was still rising at epoch 60, so a longer
schedule would likely gain a little.

### Test evaluation (reserved split)

`evaluate.py --run latest` (480 images, 1759 objects; Ultralytics metrics and an operating point at
confidence 0.35, IoU 0.5):

| Class | P | R | mAP50 | mAP50-95 | Operating point F1 |
|---|---|---|---|---|---|
| person | 0.876 | 0.834 | 0.887 | 0.622 | 0.863 |
| forklift | 0.874 | 0.632 | 0.751 | 0.567 | 0.744 |
| amr | 0.951 | 0.925 | 0.965 | 0.808 | 0.927 |
| pallet | 0.878 | 0.676 | 0.806 | 0.583 | 0.754 |
| box | 0.862 | 0.625 | 0.710 | 0.520 | 0.742 |
| cone | 0.891 | 0.817 | 0.912 | 0.735 | 0.870 |
| **all** | **0.889** | **0.752** | **0.838** | **0.639** | **0.788** |

- Operating point: 1262 TP, 184 FP, 497 FN. False positives are almost all on background (pallet 83,
  person 39, forklift 28, box 17); class confusions are rare (7).
- Recall by object size: < 32 px 0.546, 32–96 px 0.938, > 96 px 0.969. By distance: 0–3 m 0.93,
  3–6 m 0.97, 6–10 m 0.94, 10–15 m 0.88, > 15 m 0.58 (62 % of test objects are beyond 15 m, mostly
  tiny people, pallets and forklifts at the far end of aisles). Truncated objects 0.88; objects with
  visible fill < 35 % (heavily occluded) 0.44.
- Day vs night: F1 0.780 vs 0.794.
- Latency, batch 1, 640×480 → 640: GTX 1650 10.6 ms p50 end to end (0.5 preprocess, 8.7 inference,
  1.1 postprocess); CPU (i5-10300H, 3 threads) 47.1 ms p50 (44.9 inference). Same metrics on both.

### Scenario evaluation (real inference)

`evaluate_scenarios.py --run latest`: ten scripted situations built in the engine, rendered by the
worker, run through the checkpoint, scored against the ID-pass ground truth; annotated sheets in
`runs/results/<run>/reports/scenarios/`.

| Scenario | TP / FP / FN | Objects ≥ 32 px detected |
|---|---|---|
| AMR approaching a worker (12 → 2.5 m) | 25 / 3 / 14 | 10 / 11 |
| Worker crossing in front | 27 / 2 / 18 | 13 / 14 |
| Forklift passing through the view | 21 / 2 / 3 | 10 / 11 |
| Pallet partially hidden | 15 / 0 / 7 | 6 / 7 |
| Multiple objects | 17 / 2 / 7 | 13 / 14 |
| Objects at different distances | 23 / 0 / 15 | 8 / 8 |
| Low light (night shading) | 18 / 1 / 7 | 13 / 14 |
| Empty corridor | 0 / 0 / 0 | — (no false positive) |
| Another AMR entering the view | 17 / 0 / 14 | 10 / 11 |
| AMR moving while inferring | 60 / 1 / 5 | 26 / 27 |

The scenario subjects were detected in every frame; the misses are mostly small objects in the
background of the aisle (109 / 117 objects ≥ 32 px detected overall).

## 3. Live perception

`vision/scripts/serve_perception.py` (FastAPI on 127.0.0.1:8100; config `configs/inference.yaml`)
starts and supervises its own camera worker in LIVE mode and runs one shared model instance for all
robots. Per robot: one frame in flight (capture → inference → delivery), rate capped by
`max_inference_fps_per_robot` (default 3), falling by itself when the model is slower; frames that
waited longer than `max_frame_age_s` are dropped (`dropped_stale`); each browser client has a
one-slot queue (newest frame only). The service runs at `nice 10` (config `nice`), inherited by the
camera worker: perception is best effort below navigation (see the performance section). It never
touches the engine or Nav2; stopped or `enabled: false`, the twin behaves exactly as without it.

- Control: `POST /api/v1/perception/config` (`enabled`, `confidence_threshold`,
  `max_inference_fps_per_robot`, `model_path`), `GET /api/v1/perception/status` (per robot: frames,
  dropped, errors, FPS, latency p50/p95, inference time, objects), `GET /api/v1/perception/models`.
- WebSocket `/ws/v1/perception?robot_id=<id>`: uint32 LE header length, JSON header
  (`robot_id`, monotonic `frame_id`, simulation `stamp`, `world_sequence`, `width`, `height`,
  `model`, `confidence_threshold`, `device`, `detections[{class_id, label, confidence, bbox_xyxy}]`
  in image pixels, timings), then the JPEG of the processed frame — image and boxes in one message.
- ROS (when `rclpy` is available): `<ns>/camera/image_raw` (`sensor_msgs/Image`, rgb8, best
  effort), `<ns>/camera/camera_info`, `<ns>/perception/detections` (`vision_msgs/Detection2DArray`),
  stamped with the frame's simulation time, frame `[<ns>/]camera_optical_frame`.

### Web visualization

In LIVE mode, the onboard view of the selected AMR shows the **VISÃO COMPUTACIONAL** panel: enable
detection, bounding boxes, confidence, threshold slider (sent to the service), status
(DESLIGADA / AGUARDANDO / EXECUTANDO / DESATUALIZADA / INDISPONÍVEL), model, robot, frame and
simulation time, inference FPS, device, end-to-end latency (p50/p95), objects per class. While
detection is on, the view shows the processed frame for that robot with its boxes drawn on it (no
reprojection from scene coordinates, so boxes and pixels cannot disagree); messages from another
robot or with an older `frame_id` are dropped; image and overlay are letterboxed together into the
viewport. Ground truth is never shown in this view.

`web/digital-twin/test/vision.browser.ts` (headed Playwright, lite twin + service, trained
checkpoint `2026-11-14_001/best.pt@f58c34c48931`, evidence `evidence-phase7/vision_browser_trained/`):
passed — three robots and three mission options; for each robot the onboard camera follows that
robot (pose within 0.5 m of the backend), the panel reaches EXECUTANDO with `Robô: <id>`, the
overlay matches the viewport within 1.5 px, also after a resize; disabling hides the overlay, Esc
closes the view; no page errors. Each robot showed its own frames (different frame ids, views,
boxes and confidences), 3.0 FPS on `cuda:0`, 67–139 ms end to end.

## 4. Performance

Full system, two identical sequences each from a fresh launch (`evidence-phase7/full_system/`):
`lite_navigation.launch.py` (3 AMRs, 3 Nav2 stacks, Gateway), headed browser on the twin, the same
three simultaneous missions (`amr_01 → dock_05`, `amr_02 → production_w`, `amr_03 → charger_01`),
180 s of `sample_system.py` (per-process-group CPU, Intel and NVIDIA GPU), `monitor_topics.py`,
`measure_fps.mjs --headed`. **A**: perception off, browser on the overview. **B**: perception on
(3 robots × 3 FPS, GPU), browser on `amr_02`'s onboard camera with detection on, plus a 60 s
contract probe.

| | Phase 6 soak (1 AMR) | A: 3 AMRs, vision off | B: 3 AMRs, vision on (nice 10, 2 runs) |
|---|---|---|---|
| Missions | 12/12 | 3/3 | 3/3, 3/3 |
| Machine CPU mean | 55 % | 87 % (max 95) | 93 % (max 98–99) |
| Nav2 / engine / guards / Gateway (% of a core) | 80 / 47 / 33 / 53 | 215 / 134 / 97 / 67 | 201 / 134 / 91 / 68 |
| Browser / perception / camera worker | 128 / — / — | 129 / — / — | 108 / 39 / 24 |
| RTF (`/clock`, mean) | 0.997–1.005 | 0.639 | 0.605, 0.614 |
| Twin page FPS (app) | 60.0 | 54.2 (overview) | 51.3, 49.3 (onboard + overlay, pixel ratio 0.75) |
| Intel GPU busy | 94 % | 78 % | 74 % |
| GTX 1650 | suspended | idle | 7 % util, 255 MiB |
| Memory used | — | 6.1 GB | 7.8 GB |

Perception during B: 3.0 FPS per robot, 0 dropped frames, 0 capture errors, inference 20–22 ms p50
on the GPU, capture-to-detection 45–49 ms p50 (p95 63–71 ms), WebSocket contract probe 0
violations at 2.97–3.0 Hz per robot; the browser panel reported 94–119 ms end to end (p95 160–186).

Findings:

1. **The regression comes from three Nav2 stacks, not from vision.** With vision off, three robots
   with their own Nav2 and the browser saturate this 4-core CPU (87 %) and the engine falls behind
   real time (RTF 0.64; sensor rates scale with it: `/odom` 32 Hz of wall time instead of 50).
   Missions still succeed; simulated time simply runs slower. Vision adds about 6 points of machine
   CPU (perception 39 % + worker 24 % of one core, while the browser uses ~20 % of a core less in the
   onboard view) and the GTX 1650 does the rendering and inference.
2. **Perception needs to yield the CPU to navigation.** With the service at normal priority, Nav2's
   BT navigator timed out waiting 40 ms for an action server to acknowledge a goal
   (`default_server_timeout`) and aborted 2 of 3 and 1 of 3 missions in two runs (0 contacts; the
   robots stopped). The same timeout never appeared without vision. At `nice 10` it did not appear in
   two runs (3/3 and 3/3). Two runs per condition is a small sample: the margin is thin, because the
   CPU is saturated either way. The validated Nav2 parameters were not changed.
3. Camera images on ROS under this load: detections are published at 3.0 Hz per robot, but the 900 kB
   `rgb8` images arrive at 1.5–1.7 Hz at a raw subscriber (best-effort drops under CPU saturation);
   without Nav2 (Phase 7 smoke) delivery was 92–96 %. The WebSocket path (JPEG) lost nothing.

## 5. Commands

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash && source .venv/bin/activate && source install/setup.bash
export PATH=$PWD/.tools/uv/bin:$PWD/.tools/node_modules/.bin:$PATH

# Three AMRs + three Nav2 stacks + Gateway (single robot: robots:=amr_01)
ros2 launch amr_lite_sim lite_navigation.launch.py
# or three AMRs + Gateway without Nav2
ros2 launch amr_lite_sim lite_twin.launch.py

# Frontend (http://127.0.0.1:5173, LIVE ROS); `npm run build` also builds camera.html for the worker
cd web/digital-twin && npm run build && npx vite --host 127.0.0.1

# Vision (from vision/, ROS sourced)
cd vision && uv sync
uv run python scripts/generate_dataset.py --samples 3000 --robots all --seed 42
uv run python scripts/validate_dataset.py
uv run python scripts/train.py --config configs/train.yaml
uv run python scripts/evaluate.py --run latest            # add --device cpu for CPU latency
uv run python scripts/evaluate_scenarios.py --run latest
uv run python scripts/serve_perception.py                 # live inference, http://127.0.0.1:8100
uv run python scripts/probe_perception.py --seconds 20 --output /tmp/probe.json

# Multi-AMR checks
python3 src/amr_lite_sim/scripts/validate_fleet.py --output /tmp/fleet.json
python3 src/amr_lite_sim/scripts/run_fleet_missions.py --round amr_01=dock_05 amr_02=production_w amr_03=charger_01 --output /tmp/missions.json
```

Viewing detections: in the twin (LIVE ROS) select an AMR (click it, or the selector in "MISSÕES DOS
AMRs") → "Ver pela câmera do AMR" (key V) → panel VISÃO COMPUTACIONAL → "Detecção de objetos".

## 6. Known limitations and remaining work

- **CPU budget**: three Nav2 stacks + browser saturate this 4-core laptop; the twin runs at RTF
  ≈ 0.6–0.64 with three robots navigating (1.0 with one). Options: a machine with more cores, Nav2
  for fewer robots (`navigation_robots:=amr_01`), or the twin page closed. Perception must keep its
  lower priority (`nice`); a fleet-level scheduler or lighter guards (three Python command guards
  take ~0.9 core) would recover headroom.
- **Small, distant objects**: recall 0.55 below 32 px (> 15 m). Higher input resolution, a longer
  schedule (mAP still rising at epoch 60) or more targeted far views would help; for an AMR the
  near range (≤ 10 m, recall 0.93–0.97) matters most.
- **Heavily occluded objects** (visible fill < 35 %): recall 0.44.
- **Class balance**: cones (402) and AMRs (580) are the rarest classes; both still score high on the
  test split, but their test counts are small (60 and 83).
- **Racked loads** are not labelled as boxes or pallets (part of the rack instances).
- **Synthetic-only domain**: the model has only seen the twin's flat ATLAS shading; it is not
  expected to transfer to real camera images without real data or domain randomisation.
- **ROS images** degrade to ~50 % delivery under full CPU load (best effort); detections do not.
- **No fleet traffic management**: robots avoid each other only through scanners and costmaps.
- **Training precision**: FP32 on the GTX 1650 (AMP unavailable on GTX 16xx).
- The perception service needs a GPU or a desktop session for usable camera rendering; headless
  SwiftShader (~375 ms per frame) is far too slow for 3 robots.
