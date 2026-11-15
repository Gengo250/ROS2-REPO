#!/usr/bin/env python3
"""Scenario evaluation of a trained checkpoint with real model inference (Phase 6.2).

    source /opt/ros/jazzy/setup.bash && source ../install/setup.bash
    uv run python scripts/evaluate_scenarios.py --run latest

Ten scripted situations are built in the lite engine (amr_lite_sim.World; objects placed through its
validated add_obstacle, robots and people moved to explicit poses) and rendered by the camera
worker exactly like the dataset and the live service. Every frame is run through the model; the
rendered instance masks give the ground truth used only to score it. Per scenario: frames, TP / FP
/ FN at the inference threshold (IoU 0.5), and an annotated sheet (predictions in colour with
confidence, ground truth as thin white boxes) in <run>/reports/scenarios/.
"""
import argparse
import json
import math
import sys
from pathlib import Path
import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from atlas_vision import annotate, scenarios  # noqa: E402
from atlas_vision.matching import match, prf  # noqa: E402
from atlas_vision.paths import SCENE, VISION  # noqa: E402
from atlas_vision.perception import Detector  # noqa: E402
from atlas_vision.render_worker import RenderWorker  # noqa: E402

COLOURS = [(80, 80, 255), (0, 200, 255), (120, 255, 60), (90, 160, 230), (255, 120, 220), (0, 130, 255)]
CAMERA = ('amr_01', 2.0, -14.0, math.pi/2)        # open central hall, looking north along the spine


class Stage:
    def __init__(self, scene, model, config, seed=1234, warmup=30.):
        self.session = scenarios.Session(scene, model, config, seed, ['amr_01'], populate=False)
        self.world = self.session.world
        self.session.advance(warmup)
        self.camera(*CAMERA[1:])

    def camera(self, x, y, yaw, robot='amr_01'):
        if not self.session.free(robot, x, y, yaw):
            raise RuntimeError(f'camera pose ({x:.1f}, {y:.1f}) is not free')
        self.session.teleport(robot, x, y, yaw)
        self.pose = (x, y, yaw)

    def ahead(self, distance, lateral=0.):
        x, y, yaw = self.pose
        return x+distance*math.cos(yaw)-lateral*math.sin(yaw), y+distance*math.sin(yaw)+lateral*math.cos(yaw)

    def obstacle(self, kind, distance, lateral=0., yaw=0.):
        for shift in (0., .3, -.3, .6, -.6):
            x, y = self.ahead(distance, lateral+shift)
            o, _ = self.world.add_obstacle(kind, x, y, yaw)
            if o:
                return o
        raise RuntimeError(f'could not place a {kind} {distance} m ahead')

    def clear_people_ahead(self, reach=16., width=8.):
        """Move people out of the camera's corridor (scenarios that need an empty or controlled view)."""
        for k in self.world.workers:
            x, y, yaw = self.pose
            dx, dy = k.x-x, k.y-y
            along, across = dx*math.cos(yaw)+dy*math.sin(yaw), -dx*math.sin(yaw)+dy*math.cos(yaw)
            if 0 < along < reach and abs(across) < width:
                k.x, k.y = k.x+40., k.y          # parked far outside the view (the world is not stepped here)

    def person(self, distance, lateral, yaw):
        k = min(self.world.workers, key=lambda w: (not w.on_floor, w.id))
        k.x, k.y = self.ahead(distance, lateral)
        k.yaw, k.v = yaw, 1.2
        return k

    def robot(self, robot, distance, lateral, yaw):
        x, y = self.ahead(distance, lateral)
        if not self.session.free(robot, x, y, yaw):
            raise RuntimeError(f'{robot} pose not free')
        self.session.teleport(robot, x, y, yaw)


def scenarios_list(scene, model, config):
    """Each yields (name, stage, frame description) frames; built lazily on fresh worlds."""
    def approach():
        s = Stage(scene, model, config); s.clear_people_ahead()
        for d in (12, 9, 7, 5, 3.5, 2.5):
            s.person(d, .3, -math.pi/2+.4)
            yield s, f'worker {d} m ahead'

    def crossing():
        s = Stage(scene, model, config); s.clear_people_ahead()
        for lateral in (3.5, 2.2, 1., -.2, -1.4, -2.6, -3.8):
            s.person(5., lateral, -math.pi)
            yield s, f'worker crossing at 5 m, lateral {lateral:+.1f} m'

    def forklift():
        s = Stage(scene, model, config)
        f = s.world.forklifts[0]
        fx, fy, fyaw = f.pose
        # Camera 7 m beside the forklift's path, 8 m ahead of it, looking at the lane.
        ax, ay = fx+8*math.cos(fyaw), fy+8*math.sin(fyaw)
        for side in (1, -1):
            cx, cy = ax-side*7*math.sin(fyaw), ay+side*7*math.cos(fyaw)
            yaw = math.atan2(ay-cy, ax-cx)
            if s.session.free('amr_01', cx, cy, yaw):
                s.camera(cx, cy, yaw)
                break
        for i in range(10):
            yield s, f'forklift {f.id} t+{i*.6:.1f} s'
            s.session.advance(.6)

    def occluded_pallet():
        s = Stage(scene, model, config); s.clear_people_ahead()
        s.obstacle('pallet', 7., .4)
        s.obstacle('box', 4.5, .1)
        s.obstacle('cone', 9.5, -1.2)
        for lateral in (0., .8, -.8):
            s.camera(CAMERA[1]+lateral, CAMERA[2], CAMERA[3])
            yield s, f'pallet behind a box, camera {lateral:+.1f} m'

    def multiple():
        s = Stage(scene, model, config); s.clear_people_ahead()
        s.obstacle('cone', 3.5, -1.); s.obstacle('box', 5., 1.2); s.obstacle('pallet', 8., -1.5)
        s.person(6.5, .3, 0.); s.robot('amr_02', 9.5, 1.8, -math.pi/2)
        for lateral in (0., .6, -.6):
            s.camera(CAMERA[1]+lateral, CAMERA[2], CAMERA[3])
            yield s, f'five objects, camera {lateral:+.1f} m'

    def distances():
        s = Stage(scene, model, config); s.clear_people_ahead()
        for d in (2, 4, 6, 8, 10, 12):
            s.world.remove_obstacle()
            s.obstacle('cone', d, .2)
            yield s, f'cone at {d} m'

    def low_light():
        for item, text in multiple():
            item.night = True
            yield item, text+' (night)'

    def empty():
        s = Stage(scene, model, config)
        # Rack aisle of the south-west block (between rack rows -46 and -40), robot looking north.
        s.camera(-43., -8., math.pi/2)
        s.clear_people_ahead(reach=60., width=7.)
        for y in (-8., -4., 0.):
            s.camera(-43., y, math.pi/2)
            yield s, f'empty rack aisle, y={y:.0f}'

    def amr_entering():
        s = Stage(scene, model, config); s.clear_people_ahead()
        for lateral in (5.5, 4.2, 3., 1.8, .6, -.6):
            s.robot('amr_02', 6., lateral, -math.pi)
            yield s, f'amr_02 entering at lateral {lateral:+.1f} m'

    def moving():
        s = Stage(scene, model, config)
        s.obstacle('cone', 6., .8); s.obstacle('box', 9., -1.)
        for i in range(10):
            for _ in range(int(.5/s.session.step)):
                s.world.command('amr_01', .5, 0.)
                s.world.step(s.session.step)
            yield s, f'camera driving at {s.world.robots["amr_01"].drive.v:.2f} m/s'

    return {'approaching_worker': approach, 'worker_crossing': crossing, 'forklift_passing': forklift,
            'pallet_partially_hidden': occluded_pallet, 'multiple_objects': multiple, 'object_distances': distances,
            'low_light': low_light, 'empty_corridor': empty, 'amr_entering_view': amr_entering, 'camera_moving': moving}


def annotate_image(rgb, predictions, truths, names):
    image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    for _, box in truths:
        cv2.rectangle(image, tuple(box[:2]), (box[2]-1, box[3]-1), (255, 255, 255), 1)
    for cls, conf, box in predictions:
        b = [int(round(v)) for v in box]
        cv2.rectangle(image, b[:2], b[2:], COLOURS[cls % len(COLOURS)], 2)
        cv2.putText(image, f'{names[cls]} {conf:.2f}', (b[0], max(12, b[1]-4)), cv2.FONT_HERSHEY_SIMPLEX, .45, COLOURS[cls % len(COLOURS)], 1, cv2.LINE_AA)
    return image


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--run', default='latest')
    p.add_argument('--config', default=str(VISION/'configs/inference.yaml'))
    p.add_argument('--generation', default=str(VISION/'configs/generation.yaml'))
    p.add_argument('--browser', default='auto', choices=['auto', 'nvidia', 'gpu', 'headless'])
    p.add_argument('--port', type=int, default=8766)
    a = p.parse_args()
    from atlas_scene.scene import Scene
    from amr_lite_sim import robot_model
    cfg = yaml.safe_load(Path(a.config).read_text())
    gen = yaml.safe_load(Path(a.generation).read_text())
    root = VISION/'runs/results'
    run = sorted(r for r in root.iterdir() if (r/'checkpoints/best.pt').is_file())[-1] if a.run == 'latest' else Path(a.run)
    detector = Detector(run/'checkpoints/best.pt', cfg['device'], cfg['image_size'], cfg['confidence_threshold'], cfg['iou_threshold'],
                        cfg.get('torch_threads'))
    names = detector.names
    out = run/'reports/scenarios'
    out.mkdir(parents=True, exist_ok=True)
    scene, model = Scene.load(str(SCENE)), robot_model.load()
    width, height = gen['image']['width'], gen['image']['height']
    report = {'run': run.name, 'model': detector.identity, 'device': detector.device,
              'confidence_threshold': detector.confidence, 'scenarios': {}}
    with RenderWorker(port=a.port, mode=a.browser) as worker:
        for name, build in scenarios_list(scene, model, gen).items():
            tiles, totals, frames = [], {'tp': 0, 'fp': 0, 'fn': 0}, []
            # Recall split by object size (sqrt of box area, as in evaluate.py): scripted subjects are
            # ≥ 32 px, while the far end of an aisle holds many tiny people, racks' pallets and robots.
            by_size = {'small_lt_32px': [0, 0], 'ge_32px': [0, 0]}     # [detected, objects]
            for stage, text in build():
                state = stage.session.state()
                frame = worker.render({'type': 'render', 'robot_id': 'amr_01', 'width': width, 'height': height,
                                       'night': getattr(stage, 'night', False), 'ids': True, 'dt': .3, 'state': state})
                labelled, _ = annotate.boxes(frame, gen['annotation']['min_pixels'], gen['annotation']['min_box_side'])
                detections, speed = detector.predict(frame.rgb)
                predictions = [(d['class_id'], d['confidence'], d['bbox_xyxy']) for d in detections]
                truths = [(o['class_id'], o['bbox_xyxy']) for o in labelled]
                pairs, fps, fns = match(predictions, truths, .5)
                totals['tp'] += len(pairs); totals['fp'] += len(fps); totals['fn'] += len(fns)
                detected = {j for _, j, _ in pairs}
                for j, (_, (x0, y0, x1, y1)) in enumerate(truths):
                    key = 'small_lt_32px' if ((x1-x0)*(y1-y0))**.5 < 32 else 'ge_32px'
                    by_size[key][0] += j in detected
                    by_size[key][1] += 1
                frames.append({'frame': text, 'ground_truth': [names[c] for c, _ in truths],
                               'detections': [f"{d['label']} {d['confidence']:.2f}" for d in detections],
                               'tp': len(pairs), 'fp': len(fps), 'fn': len(fns), 'inference_ms': speed['total']})
                image = annotate_image(frame.rgb, predictions, truths, names)
                cv2.putText(image, text, (8, height-10), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv2.LINE_AA)
                tiles.append(cv2.resize(image, (426, 320)))
            while len(tiles) % 3:
                tiles.append(np.zeros_like(tiles[0]))
            cv2.imwrite(str(out/f'{name}.jpg'), np.vstack([np.hstack(tiles[i:i+3]) for i in range(0, len(tiles), 3)]))
            precision, recall, f1 = prf(totals['tp'], totals['fp'], totals['fn'])
            report['scenarios'][name] = {**totals, 'precision': round(precision, 3), 'recall': round(recall, 3), 'f1': round(f1, 3),
                                         'recall_by_size': {k: {'detected': d, 'objects': n, 'recall': round(d/n, 3) if n else None}
                                                            for k, (d, n) in by_size.items()},
                                         'frames': frames, 'sheet': f'reports/scenarios/{name}.jpg'}
            big = by_size['ge_32px']
            print(f"{name}: TP {totals['tp']} FP {totals['fp']} FN {totals['fn']} (P {precision:.2f} R {recall:.2f}; "
                  f"objects ≥ 32 px {big[0]}/{big[1]})", flush=True)
    (out/'scenarios.json').write_text(json.dumps(report, indent=1))


if __name__ == '__main__':
    main()
