# ATLAS vision — synthetic onboard-camera data, YOLO26n, live perception

Object detection for the onboard RGB cameras of the lite twin's three AMRs, trained only on synthetic
images rendered from the twin itself. Design, measurements and limitations:
[docs/digital-twin/PHASE_7_MULTI_AMR_VISION.md](../docs/digital-twin/PHASE_7_MULTI_AMR_VISION.md).

```
lite engine (amr_lite_sim.World) ──state──▶ camera render worker (web camera.html in Chromium:
       │                                    the twin's Three.js renderer, URDF camera mount)
       │                                       ├─ RGB 640×480 ──────────────▶ image
       │                                       └─ instance-ID pass ─────────▶ ground-truth boxes
       ▼                                                                     (dataset only)
dataset (sessions → train/val/test) ──▶ YOLO26n transfer learning ──▶ checkpoint
Gateway world state ──▶ camera worker (LIVE) ──frames──▶ perception service ──▶ browser / ROS topics
```

Classes (`configs/dataset.yaml`, the one mapping used everywhere): `person`, `forklift`, `amr`,
`pallet`, `box`, `cone`.

## Environment

```bash
cd ~/ros2_ws
export PATH=$PWD/.tools/uv/bin:$PWD/.tools/node_modules/.bin:$PATH   # uv 0.13.8 and Node 22 (project tools, git-ignored)
cd web/digital-twin && npm run build && cd ../..                     # builds camera.html (the render worker page)
cd vision && uv sync                                                 # Python 3.12 (system), PyTorch, Ultralytics, OpenCV
```

- `pyproject.toml` pins NumPy < 2 because ROS 2 Jazzy's message modules are compiled against NumPy 1.26:
  the generator and the perception service import the lite engine and `rclpy` in this environment
  (source ROS first: `source /opt/ros/jazzy/setup.bash && source ../install/setup.bash`).
- PyTorch comes from the CUDA 13.2 index (`torch 2.15.0+cu132`, the only CUDA build of 2.15; driver
  595 reports CUDA 13.2; the GTX 1650 is sm_75, included). Every script picks `cuda:0` automatically
  (`device: auto`) and falls back to the CPU (the CUDA wheels also run without a driver). For a
  CPU-only machine, point the `pytorch-cu132` index in `pyproject.toml` at
  `https://download.pytorch.org/whl/cpu` and `uv sync` again.
- Training on the GTX 1650 runs in FP32: Ultralytics' AMP check fails on GTX 16xx cards (FP16 NaNs)
  and disables AMP by itself. Batch 8 uses 2.9 GB of the 3.6 GB available; 16 does not fit.
- The render worker uses Playwright's Chromium (`~/.cache/ms-playwright`, or `$ATLAS_CHROMIUM`).
  `--browser auto` (default) picks, in order: `nvidia` — headless, ANGLE over EGL on the NVIDIA driver
  (about 19 ms per RGB + ID frame on the GTX 1650; leaves the Intel GPU to the twin's page); `gpu` — a
  minimised window on the display GPU (about 40 ms on the Intel UHD); `headless` — SwiftShader on the
  CPU (about 375 ms). Renderers agree: the same states rendered on the Intel UHD and the GTX 1650
  differ by 0.04 grey levels on average (max 0.19) with identical instance-ID passes and labels.
  The dataset in use was rendered with `gpu` (Intel). It runs with `--no-sandbox` (Ubuntu 24.04 blocks Chromium's user-namespace
  sandbox; it only loads this repository's bundle from 127.0.0.1).
- `pytest` (`uv run --group dev pytest`): unit tests that need neither a browser nor a model.

## Commands

```bash
source /opt/ros/jazzy/setup.bash && source ../install/setup.bash     # every command below, from vision/
# 1. synthetic dataset (≈ 31 min for 3000 frames here); refuses to overwrite without --overwrite
uv run python scripts/generate_dataset.py --samples 3000 --robots all --seed 42
# 2. validation (must pass before training): report + contact sheets in data/synthetic/metadata/validation/
uv run python scripts/validate_dataset.py
# 3. training: new directory runs/results/<date>_<nnn>/ per run
uv run python scripts/train.py --config configs/train.yaml
# 4. final evaluation on the reserved test split, and the ten scripted scenarios
uv run python scripts/evaluate.py --run latest              # --device cpu: CPU latency (reports/test_evaluation_<cuda|cpu>.json)
uv run python scripts/evaluate_scenarios.py --run latest
# 5. offline prediction on images
uv run python scripts/predict.py data/synthetic/images/test --output runs/predictions
# 6. live perception (needs the lite twin + Gateway running), then open the twin in the browser
uv run python scripts/serve_perception.py                         # http://127.0.0.1:8100
uv run python scripts/probe_perception.py --seconds 20 --output /tmp/probe.json   # contract check
```

In the browser (LIVE ROS): select an AMR (scene, or the robot selector of "MISSÕES DOS AMRs") →
"Câmera embarcada" (or "Ver pela câmera do AMR", key V) → panel **VISÃO COMPUTACIONAL** → "Detecção
de objetos". The view then shows the frames the model processed for that robot with its boxes,
classes, confidence (optional), model, inference FPS and end-to-end latency; the threshold slider
changes the service's confidence threshold.

## Synthetic images and ground truth

- **Renderer**: `web/digital-twin/src/camera/capture.ts` reuses the twin's `InstancedRenderer`
  (ATLAS shaders), `buildStatic`, `LiveScene` and rigs, and `cameraPoses.onboardPose` (URDF
  `base_link → camera_optical_frame`, 75° horizontal FOV, square pixels). It renders off screen into
  render targets of the requested size (default 640×480): an antialiased RGB pass and an
  instance-ID pass of the same camera and geometry (flat per-instance codes, no antialiasing). Only
  physical things are drawn: no safety fields, routes, selection, placement preview, labels or HUD;
  the capturing robot's own body is not drawn; walls and roof are closed.
- **Labels**: for every instance code in the ID pass, the visible pixels give the box (the visible
  part; occluded parts and parts outside the image do not count). Instances below 25 visible pixels
  or 4 px box side are not labelled (reported as `ignored` in the metadata). Static scene objects are
  labelled from the compiled scene: each floor pallet (`Palete no chão` solid: base plus load) and each
  box on the inspection tables; racked loads are part of the racks. Labels never come from a model.
- **Scenes**: one engine world per session (seeded behaviour of the 16 workers and 3 forklifts,
  20–150 s warm-up, 3–10 cones/boxes/pallets placed through the engine's validated `add_obstacle`,
  day or night lighting). A frame gate rejects nearly uniform frames (grey std < 8: camera against a wall)
  and frames repeating the robot's previous one (stuck robot); the camera robot is then moved to a
  free pose and the frame re-rendered (counts in `generation.json` → `rejected_frames`). Frames alternate between *targeted* views (camera robot placed 1–13 m from
  the outline of a person, forklift, other AMR, user object or floor pallet/table box, heading
  jittered ±0.55 rad, pose collision-checked by the engine) and *driving* views (velocity commands
  through the engine). Splits are assigned per session (70/15/15, seeded), so consecutive frames never
  cross splits; the validator also checks near-duplicates across splits (dHash ≤ 2 bits as a candidate,
  confirmed when 64×48 colour thumbnails differ by < 5 grey levels on average: dHash alone matches
  distinct images made of plain horizontal bands, such as a wall above the floor).
- **Output** (`data/synthetic/`, git-ignored): `images/`, `labels/` (YOLO), `masks/` (uint16 instance
  codes), `metadata/frames.jsonl` (per image: split, session and seed, robot, view mode and target,
  simulation time, lighting, camera and base pose, instances, labelled and ignored objects with pixels,
  visible fill, truncation and distance), `metadata/generation.json` (configuration and its hash, scene
  and geometry hashes, git commit, renderer, versions, session splits).

## Manual inspection procedure

After `validate_dataset.py`, open `data/synthetic/metadata/validation/`:
1. `samples_{train,val,test}.jpg`, `samples_night.jpg`: every visible person, forklift, other AMR,
   floor pallet, table box and user object has a box tight around its visible pixels; nothing else has one.
2. `difficult_objects.jpg` (truncated by the border, visible fill < 35 %, or < 150 px): the box covers
   only the visible part (e.g. a person behind a rack upright, a forklift entering the image).
3. For any doubtful image, compare `images/<split>/<name>.jpg` with `masks/<split>/<name>.png`
   (`cv2.imread(path, cv2.IMREAD_UNCHANGED)`; the codes are listed in `frames.jsonl`): the label must
   equal the extent of that code's pixels (the validator checks this for every label).
4. `class_balance.png`, `box_sizes.png`: classes present in every split, sizes and distances plausible.

## Live inference

`scripts/serve_perception.py` (FastAPI on 127.0.0.1:8100) starts and supervises its own camera
worker (LIVE mode: it renders each robot's camera from the newest Gateway world state and stamps the
frame with that state's simulation time). One model instance serves all robots, one inference at a
time in a worker thread; each robot has at most one frame in flight, frames that waited longer than
`max_frame_age_s` are dropped, the rate per robot is capped (`max_inference_fps_per_robot`) and falls
by itself under load. The service and its camera worker run at `nice 10` (`nice` in the config):
perception is best effort below navigation — at normal priority, with three Nav2 stacks saturating
the CPU, Nav2 missed action-acknowledgement deadlines and aborted missions. Runtime control: `POST /api/v1/perception/config` with `enabled`,
`confidence_threshold`, `max_inference_fps_per_robot`, `model_path`; status and metrics:
`GET /api/v1/perception/status`; checkpoints: `GET /api/v1/perception/models`.

Outputs per robot, all carrying robot id, frame id, simulation stamp, image size and model id:
- WebSocket `/ws/v1/perception?robot_id=<id>`: uint32 LE header length, JSON header (detections in
  image pixels, timings), JPEG of the processed frame — image and boxes in one message.
- ROS (when `rclpy` is importable): `<ns>/camera/image_raw` (`sensor_msgs/Image`, rgb8),
  `<ns>/camera/camera_info`, `<ns>/perception/detections` (`vision_msgs/Detection2DArray`, class
  label and score per detection), stamped with the frame's simulation time, frame
  `<prefix>camera_optical_frame`; `<ns>` is empty for amr_01 and `/amr_02`, `/amr_03` otherwise.

The perception service never touches the engine or Nav2; with it stopped or `enabled: false` the
twin runs exactly as before.
