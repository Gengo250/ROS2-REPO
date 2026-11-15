#!/usr/bin/env python3
"""Train YOLO26n on the synthetic dataset; every run gets its own results directory.

    uv run python scripts/train.py --config configs/train.yaml [--epochs N] [--batch N] [--device cpu|cuda:0]

Refuses to start unless the dataset validation report (scripts/validate_dataset.py) exists, passed
and belongs to the current dataset. Writes runs/results/<YYYY-MM-DD>_<nnn>/:
  config.yaml (resolved arguments), run.json (dataset version, seeds, software and hardware),
  checkpoints/{best,last}.pt, metrics.csv (Ultralytics results.csv, one row per epoch),
  figures/ (loss, precision, recall, mAP and learning-rate curves from metrics.csv, plus the
  Ultralytics confusion matrix, PR, F1 and P/R-confidence curves), predictions/ (validation
  batches with labels and predictions), reports/train_summary.json. Nothing is overwritten.
"""
import argparse
import datetime
import json
import platform
import shutil
import sys
import time
from pathlib import Path
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from atlas_vision.paths import VISION  # noqa: E402
from atlas_vision import reporting  # noqa: E402
from atlas_vision.models import resolve_device  # noqa: E402


def new_run_dir(root):
    root.mkdir(parents=True, exist_ok=True)
    today = datetime.date.today().isoformat()
    n = 1+max((int(p.name.rsplit('_', 1)[1]) for p in root.glob(f'{today}_*') if p.name.rsplit('_', 1)[1].isdigit()), default=0)
    run = root/f'{today}_{n:03d}'
    run.mkdir()
    return run


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--config', default=str(VISION/'configs/train.yaml'))
    p.add_argument('--epochs', type=int)
    p.add_argument('--batch', type=int)
    p.add_argument('--device')
    p.add_argument('--runs', default=str(VISION/'runs/results'))
    p.add_argument('--skip-validation-check', action='store_true', help='only for smoke tests on tiny data')
    a = p.parse_args()
    import torch
    import ultralytics
    from ultralytics import YOLO
    config = yaml.safe_load(Path(a.config).read_text())
    for key in ('epochs', 'batch', 'device'):
        if getattr(a, key) is not None:
            config[key] = getattr(a, key)
    data_yaml = (VISION/config['data']).resolve()
    data_cfg = yaml.safe_load(data_yaml.read_text())
    data_root = (VISION/data_cfg['path']).resolve() if not Path(data_cfg['path']).is_absolute() else Path(data_cfg['path'])
    generation = json.loads((data_root/'metadata/generation.json').read_text())
    report_path = data_root/'metadata/validation/report.json'
    if not a.skip_validation_check:
        if not report_path.is_file():
            sys.exit('dataset not validated: run scripts/validate_dataset.py first')
        report = json.loads(report_path.read_text())
        if not report['passed'] or report['config_sha256_16'] != generation['config_sha256_16'] or report['scene_sha256'] != generation['scene_sha256']:
            sys.exit('dataset validation failed or belongs to another dataset version: fix and re-run validate_dataset.py')
    device = resolve_device(config.get('device', 'auto'))
    run = new_run_dir(Path(a.runs))
    args = {k: v for k, v in config.items() if k not in ('model', 'data', 'task')}
    args.update(device=device, data=str(data_yaml), project=str(run), name='ultralytics', exist_ok=False, plots=True, verbose=True)
    (run/'config.yaml').write_text(yaml.safe_dump({'source_config': str(Path(a.config).resolve()), 'model': config['model'],
                                                   'task': config['task'], **args}, sort_keys=False))
    hardware = {'device': device, 'cpu_threads': torch.get_num_threads(), 'machine': platform.machine(), 'processor': platform.processor()}
    if device.startswith('cuda'):
        hardware.update(gpu=torch.cuda.get_device_name(0), gpu_memory_gb=round(torch.cuda.get_device_properties(0).total_memory/2**30, 2),
                        cuda=torch.version.cuda)
    run_info = {'run': run.name, 'started': datetime.datetime.now().astimezone().isoformat(), 'seed': config['seed'],
                'dataset': {'root': str(data_root), 'generation_seed': generation['seed'], 'frames': generation['frames'],
                            'config_sha256_16': generation['config_sha256_16'], 'scene_sha256': generation['scene_sha256'],
                            'git_commit': generation['git_commit'], 'classes': generation['classes']},
                'software': {'python': platform.python_version(), 'torch': torch.__version__, 'ultralytics': ultralytics.__version__},
                'hardware': hardware, 'pretrained': config['model']}
    (run/'run.json').write_text(json.dumps(run_info, indent=1))
    model = YOLO(str(VISION/config['model']))
    started = time.time()
    model.train(**args)
    run_info['train_seconds'] = round(time.time()-started, 1)
    run_info['finished'] = datetime.datetime.now().astimezone().isoformat()
    ultra = run/'ultralytics'
    (run/'checkpoints').mkdir()
    for name in ('best.pt', 'last.pt'):
        shutil.copy2(ultra/'weights'/name, run/'checkpoints'/name)
    shutil.copy2(ultra/'results.csv', run/'metrics.csv')
    figures, predictions = run/'figures', run/'predictions'
    figures.mkdir(); predictions.mkdir(); (run/'reports').mkdir()
    for image in ultra.glob('*.png'):
        shutil.copy2(image, figures/image.name)
    for image in ultra.glob('val_batch*.jpg'):
        shutil.copy2(image, predictions/image.name)
    curves = reporting.training_curves(run/'metrics.csv', figures)
    summary = reporting.summary(run/'metrics.csv')
    run_info.update(best_epoch=summary['best_epoch'], epochs_run=summary['epochs_run'], figures=curves)
    (run/'reports/train_summary.json').write_text(json.dumps(summary, indent=1))
    (run/'run.json').write_text(json.dumps(run_info, indent=1))
    print(json.dumps({'run': str(run), **summary['best']}, indent=1))


if __name__ == '__main__':
    main()
