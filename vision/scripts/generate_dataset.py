#!/usr/bin/env python3
"""Generate the synthetic onboard-camera dataset with automatic ground truth.

    source /opt/ros/jazzy/setup.bash && source ../install/setup.bash   # lite engine + scene
    uv run python scripts/generate_dataset.py --samples 3000 --robots all --seed 42

The lite engine (amr_lite_sim.World) runs inside this process, one world per session; the camera
render worker (web/digital-twin camera.html in Chromium) renders the RGB image and an instance-ID
image of the same frame from the engine's exact state; labels come from the visible pixels of each
instance (atlas_vision/annotate.py). Splits are assigned per session (never per frame), so near-
identical consecutive frames never cross train/val/test. A frame gate (``quality`` in the config)
rejects nearly uniform frames (camera against a wall) and frames repeating the robot's previous one
(stuck robot): the camera robot is moved to a free pose and the frame is rendered again. Output (``--output``):
  images/{train,val,test}/*.jpg, labels/{train,val,test}/*.txt (YOLO), masks/{split}/*.png
  (uint16 instance codes, for audits), metadata/frames.jsonl (one record per image) and
  metadata/generation.json (configuration, hashes, seeds, renderer, versions, session splits).
"""
import argparse
import collections
import hashlib
import json
import math
import platform
import subprocess
import sys
import time
from pathlib import Path
import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from atlas_vision import annotate, classes, scenarios  # noqa: E402
from atlas_vision.paths import REPO, SCENE, VISION  # noqa: E402
from atlas_vision.render_worker import RenderWorker  # noqa: E402

SPLITS = ('train', 'val', 'test')


def assign_splits(sessions, fractions, seed):
    order = np.random.default_rng(seed).permutation(sessions)
    n_train = int(round(fractions['train']*sessions))
    n_val = int(round(fractions['val']*sessions))
    split = {}
    for rank, session in enumerate(order.tolist()):
        split[session] = 'train' if rank < n_train else 'val' if rank < n_train+n_val else 'test'
    return split


def git_commit():
    try:
        return subprocess.run(['git', '-C', str(REPO), 'rev-parse', 'HEAD'], capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def object_positions(session):
    w = session.world
    pos = {r.id: (r.drive.x, r.drive.y) for r in w.robots.values()}
    pos.update({k.id: (k.x, k.y) for k in w.workers})
    pos.update({f.id: (f.pose[0], f.pose[1]) for f in w.forklifts})
    pos.update({o.id: (o.x, o.y) for o in w.obstacles.values()})
    for sid, _, x, y in session.static_targets:
        pos[sid] = (x, y)
    return pos


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--samples', type=int, default=3000)
    p.add_argument('--robots', default='all', help="'all' or comma-separated robot ids (the camera robots)")
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--config', default=str(VISION/'configs/generation.yaml'))
    p.add_argument('--output', default=str(VISION/'data/synthetic'))
    p.add_argument('--browser', default='auto', choices=['auto', 'nvidia', 'gpu', 'headless'])
    p.add_argument('--port', type=int, default=8765)
    p.add_argument('--jpeg-quality', type=int, default=95)
    p.add_argument('--overwrite', action='store_true')
    a = p.parse_args()
    from atlas_scene.scene import Scene
    from amr_lite_sim import robot_model
    config = yaml.safe_load(Path(a.config).read_text())
    config_hash = hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()[:16]
    out = Path(a.output)
    if out.exists() and any(out.rglob('*.jpg')) and not a.overwrite:
        sys.exit(f'{out} already holds images; use --overwrite or another --output')
    for kind in ('images', 'labels', 'masks'):
        for split in SPLITS:
            directory = out/kind/split
            directory.mkdir(parents=True, exist_ok=True)
            for old in directory.iterdir():
                old.unlink()
    (out/'metadata').mkdir(parents=True, exist_ok=True)
    scene = Scene.load(str(SCENE))
    model = robot_model.load()
    spawn_robots = [s['robot_id'] for s in scene.section('spawns') if s['active']]
    robots = spawn_robots if a.robots in ('all', '') else [r.strip() for r in a.robots.split(',')]
    per_session = config['frames_per_session']
    sessions = math.ceil(a.samples/per_session)
    split_of = assign_splits(sessions, config['split'], a.seed)
    width, height = config['image']['width'], config['image']['height']
    ann = config['annotation']
    quality = config['quality']
    rejected = collections.Counter()
    names = classes.names()
    t0 = time.monotonic()
    with RenderWorker(port=a.port, mode=a.browser, log_path=str(out/'metadata/render_worker.log')) as worker:
        if worker.hello['scene_sha256'] != scene.scene_sha256:
            sys.exit(f"render worker scene {worker.hello['scene_sha256'][:12]} != engine scene {scene.scene_sha256[:12]}: "
                     'rebuild the web bundle (cd web/digital-twin && npm run build)')
        print(f"renderer: {worker.hello['webgl']} ({worker.mode}); {sessions} sessions × {per_session} frames; scene {scene.scene_sha256[:12]}")
        generation = {'samples': a.samples, 'seed': a.seed, 'robots': robots, 'config': config, 'config_sha256_16': config_hash,
                      'scene_sha256': scene.scene_sha256, 'geometry_sha256': scene.geometry_sha256, 'git_commit': git_commit(),
                      'renderer': worker.hello, 'browser_mode': worker.mode, 'classes': names,
                      'sessions': {str(s): {'split': split_of[s], 'seed': scenarios.session_seed(a.seed, s)} for s in range(sessions)},
                      'python': platform.python_version(), 'numpy': np.__version__, 'opencv': cv2.__version__,
                      'started': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
        written = 0
        with open(out/'metadata/frames.jsonl', 'w') as records:
            for s in range(sessions):
                seed = scenarios.session_seed(a.seed, s)
                session = scenarios.Session(scene, model, config, seed, robots)
                split = split_of[s]
                generation['sessions'][str(s)].update(night=session.night, obstacles=len(session.world.obstacles))
                previous_gray = {}
                for f in range(per_session):
                    if written >= a.samples:
                        break
                    view = session.next_view()
                    for retry in range(quality['max_retries']+1):
                        frame = worker.render({'type': 'render', 'robot_id': view.robot_id, 'width': width, 'height': height,
                                               'night': session.night, 'ids': True, 'dt': 0.5, 'state': session.state()})
                        issue, gray = annotate.frame_quality_issue(frame.rgb, previous_gray.get(view.robot_id), quality)
                        if issue is None:
                            break
                        rejected[issue] += 1
                        # Point-blank or stuck: move the camera robot to a free pose and look again.
                        session.place_free(view.robot_id)
                        view = scenarios.View(view.robot_id, view.mode, None, None)
                    else:
                        sys.exit(f'session {s} frame {f}: no usable frame after {quality["max_retries"]} retries ({issue})')
                    previous_gray[view.robot_id] = gray
                    labelled, ignored = annotate.boxes(frame, ann['min_pixels'], ann['min_box_side'])
                    positions = object_positions(session)
                    base = frame.header['base']
                    for o in labelled+ignored:
                        key = o['id'].rsplit('_box_', 1)[0].removesuffix('_pallet')
                        if key in positions:
                            o['distance_m'] = round(math.hypot(positions[key][0]-base['x'], positions[key][1]-base['y']), 2)
                    name = f's{s:04d}_f{f:03d}_{view.robot_id}'
                    cv2.imwrite(str(out/'images'/split/f'{name}.jpg'), cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR),
                                [cv2.IMWRITE_JPEG_QUALITY, a.jpeg_quality])
                    (out/'labels'/split/f'{name}.txt').write_text('\n'.join(annotate.yolo_lines(labelled, width, height))+('\n' if labelled else ''))
                    codes = frame.ids.astype(np.uint16)
                    cv2.imwrite(str(out/'masks'/split/f'{name}.png'), codes)
                    records.write(json.dumps({'image': f'images/{split}/{name}.jpg', 'label': f'labels/{split}/{name}.txt',
                                              'mask': f'masks/{split}/{name}.png', 'split': split, 'session': s, 'session_seed': seed,
                                              'frame': f, 'robot_id': view.robot_id, 'view_mode': view.mode, 'target': view.target,
                                              'target_kind': view.target_kind, 'sim_time': round(frame.sim_time, 3), 'night': session.night,
                                              'width': width, 'height': height, 'camera': frame.header['camera'], 'base': base,
                                              'instances': frame.header['instances'], 'objects': labelled, 'ignored': ignored,
                                              'render_ms': round(frame.header['render_ms'], 2), 'scene_sha256': scene.scene_sha256,
                                              'config_sha256_16': config_hash})+'\n')
                    written += 1
                    if written % 100 == 0:
                        rate = written/(time.monotonic()-t0)
                        print(f'{written}/{a.samples} frames, {rate:.1f} frames/s, eta {(a.samples-written)/rate/60:.1f} min', flush=True)
        generation['frames'] = written
        generation['rejected_frames'] = dict(rejected)
        generation['seconds'] = round(time.monotonic()-t0, 1)
        generation['finished'] = time.strftime('%Y-%m-%dT%H:%M:%S%z')
        (out/'metadata/generation.json').write_text(json.dumps(generation, indent=1))
    print(f'{written} frames in {generation["seconds"]:.0f} s → {out}')


if __name__ == '__main__':
    main()
