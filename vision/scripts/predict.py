#!/usr/bin/env python3
"""Run a trained checkpoint on image files and write annotated copies plus a JSON of detections.

    uv run python scripts/predict.py IMAGE_OR_DIR [...] [--model runs/results/latest/checkpoints/best.pt] [--output out/]
"""
import argparse
import json
import sys
from pathlib import Path
import cv2
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from atlas_vision.paths import VISION  # noqa: E402
from atlas_vision.perception import Detector  # noqa: E402

COLOURS = [(80, 80, 255), (0, 200, 255), (120, 255, 60), (90, 160, 230), (255, 120, 220), (0, 130, 255)]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('inputs', nargs='+')
    p.add_argument('--config', default=str(VISION/'configs/inference.yaml'))
    p.add_argument('--model')
    p.add_argument('--confidence', type=float)
    p.add_argument('--output', default=str(VISION/'runs/predictions'))
    a = p.parse_args()
    cfg = yaml.safe_load(Path(a.config).read_text())
    detector = Detector(a.model or cfg['model_path'], cfg['device'], cfg['image_size'], a.confidence or cfg['confidence_threshold'],
                        cfg['iou_threshold'], cfg.get('torch_threads'))
    files = [f for i in a.inputs for f in (sorted(Path(i).glob('*.jpg'))+sorted(Path(i).glob('*.png')) if Path(i).is_dir() else [Path(i)])]
    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=True)
    results = {'model': detector.identity, 'device': detector.device, 'confidence_threshold': detector.confidence, 'images': {}}
    for f in files:
        bgr = cv2.imread(str(f))
        detections, speed = detector.predict(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        for d in detections:
            x0, y0, x1, y1 = (int(round(v)) for v in d['bbox_xyxy'])
            colour = COLOURS[d['class_id'] % len(COLOURS)]
            cv2.rectangle(bgr, (x0, y0), (x1, y1), colour, 2)
            cv2.putText(bgr, f"{d['label']} {d['confidence']:.2f}", (x0, max(12, y0-4)), cv2.FONT_HERSHEY_SIMPLEX, .45, colour, 1, cv2.LINE_AA)
        cv2.imwrite(str(out/f'{f.stem}_pred.jpg'), bgr)
        results['images'][str(f)] = {'detections': detections, 'timing_ms': speed}
    (out/'predictions.json').write_text(json.dumps(results, indent=1))
    print(f'{len(files)} images → {out}')


if __name__ == '__main__':
    main()
