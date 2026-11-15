#!/usr/bin/env python3
"""Evaluate a trained checkpoint on the reserved test split (final evaluation only).

    uv run python scripts/evaluate.py --run runs/results/<run_id>        # or --run latest

1. Ultralytics validation on ``split=test``: precision, recall, mAP@50, mAP@50:95 (overall and per
   class), confusion matrix, PR / F1 / P / R-confidence curves (reports/test/).
2. Operating point (inference.yaml confidence threshold, IoU 0.5 greedy matching per class, one
   prediction per image like the live service): TP / FP / FN, precision, recall and F1 per class, by
   object size, by distance to the camera robot (generation metadata), by day/night; latency
   (preprocess / inference / postprocess per image, batch 1) on this machine's device.
3. Sheets with the worst images (false positives red, missed objects yellow, matches green).
Results: <run>/reports/test_evaluation_<cuda|cpu>.json (one per device, so CPU and GPU latency
both stay on record) and figures in <run>/figures/test_*.png.
"""
import argparse
import collections
import json
import platform
import sys
import time
from pathlib import Path
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from atlas_vision.classes import names as class_names  # noqa: E402
from atlas_vision.matching import match, percentile, prf  # noqa: E402
from atlas_vision.models import model_identity, resolve_device  # noqa: E402
from atlas_vision.paths import VISION  # noqa: E402

SIZE_BINS = [('small <32 px', 0, 32), ('medium 32-96 px', 32, 96), ('large >96 px', 96, 1e9)]
DISTANCE_BINS = [('0-3 m', 0, 3), ('3-6 m', 3, 6), ('6-10 m', 6, 10), ('10-15 m', 10, 15), ('>15 m', 15, 1e9)]


def run_dir(value):
    root = VISION/'runs/results'
    if value == 'latest':
        return sorted(r for r in root.iterdir() if (r/'checkpoints/best.pt').is_file())[-1]
    p = Path(value)
    return p if p.is_absolute() else (VISION/p if (VISION/p).exists() else p)


def binned(entries, bins, key):
    out = {}
    for label, lo, hi in bins:
        chosen = [e for e in entries if e[key] is not None and lo <= e[key] < hi]
        hit = sum(e['matched'] for e in chosen)
        out[label] = {'objects': len(chosen), 'detected': hit, 'recall': round(hit/len(chosen), 4) if chosen else None}
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--run', default='latest')
    p.add_argument('--checkpoint', default='checkpoints/best.pt')
    p.add_argument('--config', default=str(VISION/'configs/inference.yaml'))
    p.add_argument('--device', default=None)
    p.add_argument('--iou', type=float, default=.5)
    a = p.parse_args()
    from ultralytics import YOLO
    import torch
    run = run_dir(a.run)
    checkpoint = run/a.checkpoint
    cfg = yaml.safe_load(Path(a.config).read_text())
    device = resolve_device(a.device or cfg['device'])
    if device == 'cpu':
        torch.set_num_threads(int(cfg.get('torch_threads', torch.get_num_threads())))
    names = class_names()
    model = YOLO(str(checkpoint))
    if [model.names[i] for i in sorted(model.names)] != names:
        sys.exit(f'checkpoint classes {model.names} differ from configs/dataset.yaml {names}')
    data_yaml = VISION/'configs/dataset.yaml'
    data_root = VISION/yaml.safe_load(data_yaml.read_text())['path']
    # ------------------------------------------------------------ 1. Ultralytics metrics on the test split
    metrics = model.val(data=str(data_yaml), split='test', imgsz=cfg['image_size'], batch=8, device=device, plots=True,
                        project=str(run/'reports'), name='test', exist_ok=True, verbose=False)
    box = metrics.box
    per_class = {names[c]: {'precision': round(float(box.p[i]), 4), 'recall': round(float(box.r[i]), 4),
                            'map50': round(float(box.ap50[i]), 4), 'map50_95': round(float(box.ap[i]), 4)}
                 for i, c in enumerate(box.ap_class_index)}
    f1 = 2*box.mp*box.mr/(box.mp+box.mr) if box.mp+box.mr else 0.
    official = {'precision': round(float(box.mp), 4), 'recall': round(float(box.mr), 4), 'f1': round(float(f1), 4),
                'map50': round(float(box.map50), 4), 'map50_95': round(float(box.map), 4), 'per_class': per_class,
                'speed_ms_per_image_batch8': {k: round(v, 2) for k, v in metrics.speed.items()}}
    # ------------------------------------------------------------ 2. operating point, one image at a time
    records = [json.loads(line) for line in (data_root/'metadata/frames.jsonl').read_text().splitlines() if line.strip()]
    test = [r for r in records if r['split'] == 'test']
    counts = {n: collections.Counter() for n in names}
    objects, timings, worst = [], collections.defaultdict(list), []
    confusion_fp = collections.Counter()
    by_light = {'day': collections.Counter(), 'night': collections.Counter()}
    model.predict(np.zeros((480, 640, 3), np.uint8), imgsz=cfg['image_size'], device=device, verbose=False)   # warm-up
    for r in test:
        image = cv2.imread(str(data_root/r['image']))
        started = time.perf_counter()
        result = model.predict(image, imgsz=cfg['image_size'], conf=cfg['confidence_threshold'], iou=cfg['iou_threshold'],
                               device=device, verbose=False)[0]
        timings['end_to_end'].append((time.perf_counter()-started)*1000)
        for k, v in result.speed.items():
            timings[k].append(v)
        preds = [(int(c), float(s), b.tolist()) for c, s, b in zip(result.boxes.cls, result.boxes.conf, result.boxes.xyxy)]
        truths = [(o['class_id'], o['bbox_xyxy']) for o in r['objects']]
        pairs, fps, fns = match(preds, truths, a.iou)
        light = 'night' if r['night'] else 'day'
        for _, j, _ in pairs:
            counts[names[truths[j][0]]]['tp'] += 1
            by_light[light]['tp'] += 1
        for i in fps:
            counts[names[preds[i][0]]]['fp'] += 1
            by_light[light]['fp'] += 1
            # what was under a false positive: another class (confusion) or background
            others = [(o, o_box) for o, o_box in truths if o != preds[i][0]]
            from atlas_vision.matching import iou
            best = max(((iou(preds[i][2], ob), oc) for oc, ob in others), default=(0, None))
            confusion_fp[f'{names[preds[i][0]]} on {names[best[1]] if best[0] >= a.iou else "background"}'] += 1
        for j in fns:
            counts[names[truths[j][0]]]['fn'] += 1
            by_light[light]['fn'] += 1
        matched = {j for _, j, _ in pairs}
        for j, o in enumerate(r['objects']):
            x0, y0, x1, y1 = o['bbox_xyxy']
            objects.append({'kind': o['kind'], 'size': float(np.sqrt((x1-x0)*(y1-y0))), 'distance': o.get('distance_m'),
                            'matched': j in matched, 'truncated': o['truncated'], 'fill': o['fill']})
        worst.append((len(fps)+len(fns), r, preds, truths, pairs, fps, fns))
    operating = {}
    for n in names:
        c = counts[n]
        pr, rc, f = prf(c['tp'], c['fp'], c['fn'])
        operating[n] = {'tp': c['tp'], 'fp': c['fp'], 'fn': c['fn'], 'precision': round(pr, 4), 'recall': round(rc, 4), 'f1': round(f, 4)}
    total = collections.Counter()
    for c in counts.values():
        total.update(c)
    pr, rc, f = prf(total['tp'], total['fp'], total['fn'])
    light_summary = {k: dict(zip(('precision', 'recall', 'f1'), (round(x, 4) for x in prf(v['tp'], v['fp'], v['fn'])))) | dict(v)
                     for k, v in by_light.items()}
    occluded = [o for o in objects if o['fill'] < .35]
    latency = {k: {'mean': round(float(np.mean(v)), 2), 'p50': round(percentile(v, 50), 2), 'p95': round(percentile(v, 95), 2)}
               for k, v in timings.items()}
    result = {'run': run.name, 'checkpoint': model_identity(checkpoint), 'device': device,
              'device_name': torch.cuda.get_device_name(0) if device.startswith('cuda') else platform.processor() or 'cpu',
              'test_images': len(test), 'test_objects': len(objects), 'official_ultralytics': official,
              'operating_point': {'confidence_threshold': cfg['confidence_threshold'], 'iou_match': a.iou,
                                  'overall': {'tp': total['tp'], 'fp': total['fp'], 'fn': total['fn'], 'precision': round(pr, 4),
                                              'recall': round(rc, 4), 'f1': round(f, 4)},
                                  'per_class': operating, 'false_positive_causes': dict(confusion_fp.most_common()),
                                  'recall_by_size': binned(objects, SIZE_BINS, 'size'),
                                  'recall_by_distance': binned(objects, DISTANCE_BINS, 'distance'),
                                  'recall_truncated': round(np.mean([o['matched'] for o in objects if o['truncated']]), 4) if any(o['truncated'] for o in objects) else None,
                                  'recall_partly_occluded_fill_lt_0_35': round(np.mean([o['matched'] for o in occluded]), 4) if occluded else None,
                                  'by_lighting': light_summary},
              'latency_ms_batch1': latency}
    (run/'reports').mkdir(exist_ok=True)
    tag = 'cuda' if device.startswith('cuda') else 'cpu'
    (run/f'reports/test_evaluation_{tag}.json').write_text(json.dumps(result, indent=1))
    # ------------------------------------------------------------ figures
    figures = run/'figures'
    figures.mkdir(exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, 4))
    x = np.arange(len(names))
    for k, metric in enumerate(('precision', 'recall', 'f1')):
        ax.bar(x+(k-1)*.27, [operating[n][metric] for n in names], .27, label=metric)
    ax.set_xticks(x, names); ax.set_ylim(0, 1.05); ax.legend(); ax.grid(axis='y', alpha=.3)
    ax.set_title(f'Test split, confidence ≥ {cfg["confidence_threshold"]}, IoU ≥ {a.iou}')
    fig.tight_layout(); fig.savefig(figures/'test_per_class.png', dpi=110); plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.6))
    for ax, (key, bins) in zip(axes, (('recall_by_size', SIZE_BINS), ('recall_by_distance', DISTANCE_BINS))):
        values = result['operating_point'][key]
        ax.bar(list(values), [v['recall'] or 0 for v in values.values()])
        for i, v in enumerate(values.values()):
            ax.text(i, (v['recall'] or 0)+.02, f"n={v['objects']}", ha='center', fontsize=8)
        ax.set_ylim(0, 1.1); ax.set_title(key.replace('_', ' ')); ax.tick_params(axis='x', labelsize=8)
    fig.tight_layout(); fig.savefig(figures/'test_recall_bins.png', dpi=110); plt.close(fig)
    worst.sort(key=lambda w: -w[0])
    tiles = []
    for _, r, preds, truths, pairs, fps, fns in worst[:12]:
        image = cv2.imread(str(data_root/r['image']))
        for i, _, _ in pairs:
            b = [int(v) for v in preds[i][2]]; cv2.rectangle(image, b[:2], b[2:], (90, 220, 90), 2)
        for i in fps:
            b = [int(v) for v in preds[i][2]]; cv2.rectangle(image, b[:2], b[2:], (60, 60, 255), 2)
            cv2.putText(image, f'FP {names[preds[i][0]]} {preds[i][1]:.2f}', (b[0], max(12, b[1]-3)), 0, .45, (60, 60, 255), 1)
        for j in fns:
            b = truths[j][1]; cv2.rectangle(image, tuple(b[:2]), (b[2]-1, b[3]-1), (0, 220, 255), 2)
            cv2.putText(image, f'FN {names[truths[j][0]]}', (b[0], min(470, b[3]+12)), 0, .45, (0, 220, 255), 1)
        tiles.append(cv2.resize(image, (320, 240)))
    if tiles:
        while len(tiles) % 4:
            tiles.append(np.zeros_like(tiles[0]))
        cv2.imwrite(str(figures/'test_errors.jpg'), np.vstack([np.hstack(tiles[i:i+4]) for i in range(0, len(tiles), 4)]))
    print(json.dumps({'official': {k: official[k] for k in ('precision', 'recall', 'f1', 'map50', 'map50_95')},
                      'operating_point': result['operating_point']['overall'], 'latency_ms': latency.get('end_to_end')}, indent=1))


if __name__ == '__main__':
    main()
